# Simple Rust port of mini-rv32ima

This is a Rust port of https://github.com/cnlohr/mini-rv32ima

It doesn't add anything substantial and uses some unsafe code. Most probably it even adds a few bugs. To really learn more have a look at the original implementation.

# Run the example

- `cargo run --release --example cli -- --image=baremetal.bin -m=1000`
- `cargo run --release --example cli -- --image=linux_image -m=16000`
