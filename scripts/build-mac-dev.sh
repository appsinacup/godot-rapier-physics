cargo fmt
cargo clippy --fix --allow-dirty
cargo build --features="simd-stable,parallel"
cp target/debug/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_debug.framework/libphysics_server_rapier2d.macos.template_debug.dylib
