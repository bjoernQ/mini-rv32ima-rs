#![no_std]
pub const MINIRV32_RAM_IMAGE_OFFSET: u32 = 0x80000000;

pub trait Extension {
    fn load4(&mut self, ofs: u32) -> u32;
    fn store4(&mut self, ofs: u32, value: u32);

    fn load2(&mut self, ofs: u32) -> u16;
    fn store2(&mut self, ofs: u32, value: u16);

    fn load1(&mut self, ofs: u32) -> u8;
    fn store1(&mut self, ofs: u32, value: u8);

    fn csr_read(&mut self, csrno: u32) -> u32;
    fn csr_write(&mut self, csrno: u32, writeval: u32);

    fn mem_load_control(&mut self, rsval: u32) -> u32;
    fn mem_store_control(&mut self, addy: u32, rs2: u32);
}

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
//
// We're going to try to keep the full processor state to 12 x uint4.
#[derive(Debug)]
pub struct MiniRV32IMAState<'a, E>
where
    E: Extension,
{
    pub extension: &'a mut E,
    pub ram_size: u32,

    pub regs: [u32; 32],

    pub pc: u32,
    pub mstatus: u32,
    pub cyclel: u32,
    pub cycleh: u32,

    pub timerl: u32,
    pub timerh: u32,
    pub timermatchl: u32,
    pub timermatchh: u32,

    pub mscratch: u32,
    pub mtvec: u32,
    pub mie: u32,
    pub mip: u32,

    pub mepc: u32,
    pub mtval: u32,
    pub mcause: u32,

    // Note: only a few bits are used.  (Machine = 3, User = 0)
    // Bits 0..1 = privilege.
    // Bit 2 = WFI (Wait for interrupt)
    // Bit 3 = Load/Store has a reservation.
    pub extraflags: u32,
}

macro_rules! CSR {
    ($state: ident, $reg: ident) => {
        $state.$reg
    };
}

macro_rules! REG {
    ($state: ident, $x: expr) => {
        $state.regs[($x) as usize]
    };
}

macro_rules! SETCSR {
    ($state: ident, $reg: ident, $val:expr) => {
        $state.$reg = $val;
    };
}

macro_rules! REGSET {
    ($state: ident, $x: expr, $val:expr) => {
        $state.regs[$x] = $val;
    };
}

impl<'a, E> MiniRV32IMAState<'a, E>
where
    E: Extension,
{
    pub fn new(extension: &'a mut E, ram_size: u32) -> Self {
        Self {
            extension,
            ram_size,
            regs: Default::default(),
            pc: Default::default(),
            mstatus: Default::default(),
            cyclel: Default::default(),
            cycleh: Default::default(),
            timerl: Default::default(),
            timerh: Default::default(),
            timermatchl: Default::default(),
            timermatchh: Default::default(),
            mscratch: Default::default(),
            mtvec: Default::default(),
            mie: Default::default(),
            mip: Default::default(),
            mepc: Default::default(),
            mtval: Default::default(),
            mcause: Default::default(),
            extraflags: Default::default(),
        }
    }

    pub fn mini_rv32_imastep(&mut self, _v_proc_address: u32, elapsed_us: u32, count: u32) -> u32 {
        let new_timer: u32 = CSR!(self, timerl).wrapping_add(elapsed_us);
        if new_timer < CSR!(self, timerl) {
            CSR!(self, timerh) += 1;
        }
        CSR!(self, timerl) = new_timer;

        // Handle Timer interrupt.
        if (CSR!(self, timerh) > CSR!(self, timermatchh)
            || (CSR!(self, timerh) == CSR!(self, timermatchh)
                && CSR!(self, timerl) > CSR!(self, timermatchl)))
            && ((CSR!(self, timermatchh) != 0) || (CSR!(self, timermatchl) != 0))
        {
            CSR!(self, extraflags) &= !4; // Clear WFI
            CSR!(self, mip) |= 1 << 7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
        } else {
            CSR!(self, mip) &= !(1 << 7);
        }

        // If WFI, don't run processor.
        if (CSR!(self, extraflags) & 4) != 0 {
            return 1;
        }

        let mut icount: u32 = 0;

        loop {
            if icount >= count {
                break;
            }

            let ir: u32;
            let mut trap: u32 = 0; // If positive, is a trap or interrupt.  If negative, is fatal error.
            let mut rval: u32 = 0;

            // Increment both wall-clock and instruction count time.  (NOTE: Not strictly needed to run Linux)
            CSR!(self, cyclel) += 1;
            if CSR!(self, cyclel) == 0 {
                CSR!(self, cycleh) += 1;
            }

            let mut pc: u32 = CSR!(self, pc);
            let ofs_pc: u32 = pc - MINIRV32_RAM_IMAGE_OFFSET;

            if ofs_pc >= self.ram_size {
                trap = 1 + 1; // Handle access violation on instruction read.
            } else if (ofs_pc & 3) != 0 {
                trap = 1 + 0; //Handle PC-misaligned access
            } else {
                ir = self.extension.load4(ofs_pc);
                let mut rdid: u32 = (ir >> 7) & 0x1f;

                match ir & 0x7f
			{
				0b0110111 => { // LUI
					rval = ir & 0xfffff000;
                }
				0b0010111 => { // AUIPC
					rval = pc.wrapping_add( ir & 0xfffff000 );
                }
				0b1101111 => { // JAL
					let mut reladdy: i32 = (((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000))) as i32;
					if( reladdy & 0x00100000 ) != 0 { reladdy |= 0xffe00000u32 as i32; } // Sign extension.
					rval = pc + 4;
					pc = (pc as i32 + reladdy - 4) as u32;
				}
				0b1100111 => { // JALR
					let imm: u32 = ir >> 20;
					let imm_se: i32 = imm as i32 | ( if imm & 0x800 != 0 { 0xfffff000u32 as i32 } else { 0} );
					rval = pc + 4;
					pc = (( (REG!(self, (ir >> 15) & 0x1f ) as i32 + imm_se) & !1) - 4) as u32;
				}
				0b1100011 => { // Branch
					let mut immm4: u32 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
					if( immm4 & 0x1000 )!=0 { immm4 |= 0xffffe000; }
					let rs1: i32 = REG!(self, (ir >> 15) & 0x1f) as i32;
					let rs2: i32 = REG!(self, (ir >> 20) & 0x1f) as i32;
					immm4 = pc.wrapping_add(immm4).wrapping_sub(4);
					rdid = 0;
					match ( ir >> 12 ) & 0x7
					{
						// BEQ, BNE, BLT, BGE, BLTU, BGEU 
						0b000 => { if rs1 == rs2 {pc = immm4; } }
						0b001 => { if rs1 != rs2 {pc = immm4;}  }
						0b100 => { if rs1 < rs2 {pc = immm4;}  }
						0b101 => { if rs1 >= rs2 {pc = immm4;}  }//BGE
						0b110 => { if (rs1 as u32) < rs2 as u32 {pc = immm4;  }   }//BLTU
						0b111 => { if rs1 as u32 >= rs2 as u32 {pc = immm4;}   } //BGEU
						_ => { trap = 2+1; }
					}
				}
				0b0000011 => { // Load
					let rs1: u32 = REG!(self, (ir >> 15) & 0x1f);
					let imm: u32 = ir >> 20;
					let imm_se: i32 = (imm | ( if imm & 0x800 != 0 {0xfffff000} else {0} )) as i32;
					let mut rsval: u32 = (rs1 as i32 + imm_se) as u32;

					rsval = rsval.wrapping_sub(MINIRV32_RAM_IMAGE_OFFSET);
					if rsval >= self.ram_size - 3
					{
						rsval = rsval.wrapping_add(MINIRV32_RAM_IMAGE_OFFSET);
						if rsval >= 0x10000000 && rsval < 0x12000000  // UART, CLNT
						{
							if rsval == 0x1100bffc { // https://chromitem-soc.readthedocs.io/en/latest/clint.html
								rval = CSR!(self, timerh );
                            } else if rsval == 0x1100bff8 {
								rval = CSR!(self, timerl );
                            } else {
								rval = self.extension.mem_load_control( rsval );
                            }
						}
						else
						{
							trap = 5+1;
							rval = rsval;
						}
					}
					else
					{
						match ( ir >> 12 ) & 0x7
						{
							//LB, LH, LW, LBU, LHU
							 0b000 => { rval = self.extension.load1( rsval ) as i8 as u32; }
							 0b001 => { rval = self.extension.load2( rsval ) as i16 as u32; }
							 0b010 => { rval = self.extension.load4( rsval ); }
							 0b100 => { rval = self.extension.load1( rsval ) as u32; }
							 0b101 => { rval = self.extension.load2( rsval ) as u32; }
							_ => { trap = 2+1; }
						}
					}
				}
				0b0100011 => // Store
				{
					let rs1: u32 = REG!(self, (ir >> 15) & 0x1f);
					let rs2: u32 = REG!(self, (ir >> 20) & 0x1f);
					let mut addy: u32 = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
					if addy & 0x800 != 0 { addy |= 0xfffff000; }
					addy = addy.wrapping_add(rs1.wrapping_sub(MINIRV32_RAM_IMAGE_OFFSET));
					rdid = 0;

					if addy >= self.ram_size - 3
					{
						addy = addy.wrapping_add(MINIRV32_RAM_IMAGE_OFFSET);
						if addy >= 0x10000000 && addy < 0x12000000
						{
							// Should be stuff like SYSCON, 8250, CLNT
							if addy == 0x11004004  { //CLNT
								CSR!(self, timermatchh ) = rs2;
                            } else if addy == 0x11004000 { //CLNT
								CSR!(self, timermatchl ) = rs2;
                            } else if addy == 0x11100000 //SYSCON (reboot, poweroff, etc.)
							{
								SETCSR!(self, pc, pc + 4 );
								return rs2; // NOTE: PC will be PC of Syscon.
							}
							else {
								self.extension.mem_store_control( addy, rs2 );
                            }
						}
						else
						{
							trap = 7+1; // Store access fault.
							rval = addy;
						}
					}
					else
					{
						match ( ir >> 12 ) & 0x7
						{
							//SB, SH, SW
							 0b000 => { self.extension.store1( addy, (rs2 &0xff) as u8 ); }
							 0b001 => { self.extension.store2( addy, (rs2 & 0xffff) as u16); }
							 0b010 => { self.extension.store4( addy, rs2 ); }
							_ => { trap = 2+1; }
						}
					}
				}
				0b0010011 |  // Op-immediate
				0b0110011 => // Op
				{
					let mut imm: u32 = ir >> 20;
					imm = imm | (if imm & 0x800 != 0 {0xfffff000}else{0});
					let rs1: u32 = REG!(self, (ir >> 15) & 0x1f);
					let is_reg: u32 = !!( ir & 0b100000 );
					let rs2: u32 = if is_reg != 0 { REG!(self, imm & 0x1f) } else { imm };

					if is_reg != 0 && ( ir & 0x02000000 != 0)
					{
						match ir>>12&7 //0x02000000 = RV32M
						{
							 0b000 => { rval = rs1.wrapping_mul(rs2); } // MUL
							 0b001 => { rval = (((rs1 as i32) as i64 * ((rs2 as i32) as i64)) >> 32) as u32; } // MULH
							 0b010 => { rval = (((rs1 as i32) as i64) * rs2 as i64 >> 32) as u32; } // MULHSU // CHECK
							 0b011 => { rval = (((rs1 as u64) * rs2 as u64) >> 32) as u32; } // MULHU
							 0b100 => { if rs2 == 0 {rval = -1i32 as u32; }else {rval = ((rs1 as i32) / (rs2 as i32)) as u32;} } // DIV
							 0b101 => { if rs2 == 0 {rval = 0xffffffff; }else {rval = rs1 / rs2;} } // DIVU
							 0b110 => { if rs2 == 0 {rval = rs1; }else {rval = ((rs1 as i32) % (rs2 as i32)) as u32;} } // REM
							 0b111 => { if rs2 == 0 {rval = rs1; }else {rval = rs1 % rs2;} } // REMU
                             _ => { panic!() }
						}
					}
					else
					{
						match (ir>>12)&7 // These could be either op-immediate or op commands.  Be careful.
						{
							 0b000 => { rval = if is_reg!=0 && (ir & 0x40000000 != 0) { rs1.wrapping_sub(rs2) } else { rs1.wrapping_add(rs2) }; }
							 0b001 => { rval = rs1.wrapping_shl(rs2); }
							 0b010 => { rval = if (rs1 as i32) < rs2 as i32 { 1 } else { 0 }; }
							 0b011 => { rval = if rs1 < rs2 { 1 } else { 0 }; }
							 0b100 => { rval = rs1 ^ rs2; }
							 0b101 => { rval = if ir & 0x40000000 != 0 { ((rs1 as i32).wrapping_shr(rs2)) as u32 } else { rs1.wrapping_shr(rs2) }; }
							 0b110 => { rval = rs1 | rs2; }
							 0b111 => { rval = rs1 & rs2; }
                             _ => { panic!() }
						}
					}
				}
				0b0001111 => {
					rdid = 0;   // fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
                }
				0b1110011 => // Zifencei+Zicsr
				{
					let csrno: u32 = ir >> 20;
					let microop: u32 = ( ir >> 12 ) & 0b111;
					if microop & 3 != 0 // It's a Zicsr function.
					{
						let rs1imm: u32 = (ir >> 15) & 0x1f;
						let rs1: u32 = REG!(self, rs1imm);
						let writeval: u32;

						// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
						// Generally, support for Zicsr
						match csrno {
						0x340 => { rval = CSR!(self, mscratch );  }
						0x305 => { rval = CSR!(self, mtvec );  }
						0x304 => { rval = CSR!(self, mie );  }
						0xC00 => { rval = CSR!(self, cyclel );  }
						0x344 => { rval = CSR!(self, mip );  }
						0x341 => { rval = CSR!(self, mepc );  }
						0x300 => { rval = CSR!(self, mstatus );   }//mstatus
						0x342 => { rval = CSR!(self, mcause );  }
						0x343 => { rval = CSR!(self, mtval );  }
						0xf11 => { rval = 0xff0ff0ff;   }//mvendorid
						0x301 => { rval = 0x40401101;   }//misa (XLEN=32, IMA+X)
						//case 0x3B0: rval = 0; break; //pmpaddr0
						//case 0x3a0: rval = 0; break; //pmpcfg0
						//case 0xf12: rval = 0x00000000; break; //marchid
						//case 0xf13: rval = 0x00000000; break; //mimpid
						//case 0xf14: rval = 0x00000000; break; //mhartid
						_ => {
							rval = self.extension.csr_read( csrno );
                        }
						}

						match microop
						{
							0b001 => { writeval = rs1; }  			//CSRRW
							0b010 => { writeval = rval | rs1; }		//CSRRS
							0b011 => { writeval = rval & !rs1; }		//CSRRC
							0b101 => { writeval = rs1imm; }			//CSRRWI
							0b110 => { writeval = rval | rs1imm; }	//CSRRSI
							0b111 => { writeval = rval & !rs1imm; }	//CSRRCI
                            _ => { panic!() }
						}

						match csrno
						{
						0x340 => { SETCSR!(self, mscratch, writeval ); }
						0x305 => { SETCSR!(self, mtvec, writeval );}
						0x304 => { SETCSR!(self, mie, writeval ); }
						0x344 => { SETCSR!(self, mip, writeval ); }
						0x341 => { SETCSR!(self, mepc, writeval ); }
						0x300 => { SETCSR!(self, mstatus, writeval ); } //mstatus
						0x342 => { SETCSR!(self, mcause, writeval ); }
						0x343 => { SETCSR!(self, mtval, writeval ); }
						//case 0x3a0: break; //pmpcfg0
						//case 0x3B0: break; //pmpaddr0
						//case 0xf11: break; //mvendorid
						//case 0xf12: break; //marchid
						//case 0xf13: break; //mimpid
						//case 0xf14: break; //mhartid
						//case 0x301: break; //misa
						_ => {
							self.extension.csr_write( csrno, writeval );
                            }
						}
					}
					else if microop == 0b000 // "SYSTEM"
					{
						rdid = 0;
						if csrno == 0x105 //WFI (Wait for interrupts)
						{
							CSR!(self, mstatus ) |= 8;    //Enable interrupts
							CSR!(self, extraflags ) |= 4; //Infor environment we want to go to sleep.
							SETCSR!(self, pc, pc + 4 );
							return 1;
						}
						else if ( csrno & 0xff ) == 0x02  // MRET
						{
							//https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							//Table 7.6. MRET then in mstatus/mstatush sets MPV=0, MPP=0, MIE=MPIE, and MPIE=1. La
							// Should also update mstatus to reflect correct mode.
							let startmstatus: u32 = CSR!(self, mstatus );
							let startextraflags: u32 = CSR!(self, extraflags );
							SETCSR!(self, mstatus , (( startmstatus & 0x80) >> 4) | ((startextraflags&3) << 11) | 0x80 );
							SETCSR!(self, extraflags, (startextraflags & !3) | ((startmstatus >> 11) & 3) );
							pc = CSR!(self, mepc ) -4;
						}
						else
						{
							match csrno
							{
							0 => { trap = if CSR!(self, extraflags ) & 3 != 0 { 11+1 } else { 8+1 }; } // ECALL; 8 = "Environment call from U-mode"; 11 = "Environment call from M-mode"
							1 => {	trap = 3+1; } // EBREAK 3 = "Breakpoint"
							_ => { trap = 2+1; } // Illegal opcode.
							}
						}
					}
					else {
						trap = 2+1; 				// Note micrrop 0b100 == undefined.
                    }
				}
				0b0101111 => // RV32A
				{
					let mut rs1: u32 = REG!(self, (ir >> 15) & 0x1f);
					let mut rs2: u32 = REG!(self, (ir >> 20) & 0x1f);
					let irmid: u32 = ( ir>>27 ) & 0x1f;

					rs1 = rs1.wrapping_sub(MINIRV32_RAM_IMAGE_OFFSET);

					// We don't implement load/store from UART or CLNT with RV32A here.

					if rs1 >= self.ram_size - 3
					{
						trap = 7+1; //Store/AMO access fault
						rval = rs1.wrapping_add( MINIRV32_RAM_IMAGE_OFFSET );
					}
					else
					{
						rval = self.extension.load4( rs1 );
						// Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
						let mut dowrite: u32 = 1;
						match irmid
						{
							0b00010 => { dowrite = 0; CSR!(self, extraflags ) |= 8; }  //LR.W
							0b00011 => { rval = 0; /* OKAY? */ /* !(CSR!(self, extraflags ) & 8); */} //SC.W (Lie and always say it's good) 
							0b00001 => { } //AMOSWAP.W 
							0b00000 => { rs2 = rs2.wrapping_add(rval); } //AMOADD.W 
							0b00100 => { rs2 ^= rval; } //AMOXOR.W 
							0b01100 => { rs2 &= rval; } //AMOAND.W 
							0b01000 => { rs2 |= rval; } //AMOOR.W 
							0b10000 => { rs2 = if (rs2 as i32)< rval as i32 {rs2} else {rval}; } //AMOMIN.W 
							0b10100 => { rs2 = if (rs2 as i32) > rval as i32 {rs2} else {rval}; } //AMOMAX.W 
							0b11000 => { rs2 = if rs2<rval {rs2} else {rval}; } //AMOMINU.W 
							0b11100 => { rs2 = if rs2>rval {rs2} else {rval}; } //AMOMAXU.W 
							_ =>{ trap = 2+1; dowrite = 0; } //Not supported.
						}
						if dowrite != 0 { self.extension.store4( rs1, rs2 ); }
					}
				}
				_ => { trap = 2+1; } // Fault: Invalid opcode.
			}

                if trap == 0 {
                    if rdid != 0 {
                        REGSET!(self, rdid as usize, rval);
                    }
                    // Write back register.
                    else if (CSR!(self, mip) & (1 << 7)) != 0
                        && (CSR!(self, mie) & (1 << 7)/*mtie*/) != 0
                        && (CSR!(self, mstatus) & 0x8/*mie*/) != 0
                    {
                        trap = 0x80000007; // Timer interrupt.
                    }
                }
            }

            // MINIRV32_POSTEXEC(pc, ir, trap);

            // Handle traps and interrupts.
            if trap != 0 {
                if trap & 0x80000000 != 0
                // If prefixed with 1 in MSB, it's an interrupt, not a trap.
                {
                    CSR!(self, extraflags) &= !8;
                    SETCSR!(self, mcause, trap);
                    SETCSR!(self, mtval, 0);
                    pc += 4; // PC needs to point to where the PC will return to.
                } else {
                    SETCSR!(self, mcause, trap - 1);
                    SETCSR!(self, mtval, if trap > 5 && trap <= 8 { rval } else { pc });
                }
                SETCSR!(self, mepc, pc); //TRICKY: The kernel advances mepc automatically.
                                         //CSR!(self, mstatus ) & 8 = MIE, & 0x80 = MPIE
                                         // On an interrupt, the system moves current MIE into MPIE
                SETCSR!(
                    self,
                    mstatus,
                    ((CSR!(self, mstatus) & 0x08) << 4) | ((CSR!(self, extraflags) & 3) << 11)
                );
                pc = CSR!(self, mtvec).wrapping_sub(4);

                // XXX TODO: Do we actually want to check here? Is this correct?
                if !(trap & 0x80000000 != 0) {
                    CSR!(self, extraflags) |= 3;
                }
            }

            SETCSR!(self, pc, pc.wrapping_add(4));

            icount += 1;
        }
        return 0;
    }
}
