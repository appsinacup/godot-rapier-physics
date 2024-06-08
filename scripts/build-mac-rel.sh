cargo fmt
cargo clippy --fix --allow-dirty
cargo build --release --features="single_dim2,parallel,simd-stable" --no-default-features
cp target/release/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
