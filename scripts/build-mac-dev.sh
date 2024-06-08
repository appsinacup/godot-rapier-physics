cargo fmt
cargo clippy --fix --allow-dirty
cargo build --features="single_dim2,parallel,simd-stable" --no-default-features
cp target/debug/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_debug.framework/libphysics_server_rapier2d.macos.template_debug.dylib

