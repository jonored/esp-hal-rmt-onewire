# esp-hal-rmt-onewire

An async driver to use the rmt peripheral to access onewire devices.

## Examples

To run the Embassy-powered thermometer example you now need to pick the chip
feature that matches your board:

```bash
cargo run --target riscv32imc-unknown-none-elf \
	--example thermometer \
	--features example-esp32c3

cargo run --target riscv32imac-unknown-none-elf \
	--example thermometer \
	--features example-esp32c6
```

Only enable one of the `example-esp32c3` or `example-esp32c6` features at a
time.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
