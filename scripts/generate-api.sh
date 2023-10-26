cd src/rapier2d-wrapper
cargo install cbindgen
cbindgen --config cbindgen.toml --crate rapier2d-wrapper --output includes/rapier2d_wrapper.h
cd ../..
