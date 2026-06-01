# Rust Examples

## Available examples

- `sdk_registration_demo.rs`
  - Short ops-style native registration demo.
- `*_registration.rs`
  - Curated native registration examples that mirror the Python example basenames.
- `fake_tf_publisher.rs`
  - Fake TF publisher parity scaffold for local validation loops.
- `e2e_registration_check.rs`
  - Minimal register + metadata verification flow.

## Run

```bash
cd rust
cargo run --example sdk_registration_demo
```

```bash
cd rust
cargo run --example fake_tf_publisher -- --robot-name test_bot --rate-hz 10 --cycles 20
```

```bash
cd rust
cargo run --example e2e_registration_check -- --robot-name SdkBot_E2E
```
