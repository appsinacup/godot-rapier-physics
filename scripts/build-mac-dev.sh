cargo fmt
cargo clippy --fix --allow-dirty
cargo build --features="single_dim2,parallel,simd-stable" --no-default-features
cp target/debug/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libgodot_rapier.macos.template_debug.framework/libgodot_rapier.macos.template_debug.dylib

