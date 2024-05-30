cargo clippy --fix --allow-dirty
cargo build --features="simd-stable,parallel"
cargo build --release --features="simd-stable,parallel"
cp target/debug/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_debug.framework/libphysics_server_rapier2d.macos.template_debug.dylib
cp target/debug/libgodot_rapier.dylib Godot-Physics-Tests/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_debug.framework/libphysics_server_rapier2d.macos.template_debug.dylib

cp target/release/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_release.framework/libphysics_server_rapier2d.macos.template_release.dylib
cp target/release/libgodot_rapier.dylib Godot-Physics-Tests/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_release.framework/libphysics_server_rapier2d.macos.template_release.dylib
