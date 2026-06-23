cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty --features="single-dim3,simd-stable,serde-serialize,experimental-threads,register-docs,api-4-7"
cargo build --release --features="single-dim3,simd-stable,serde-serialize,experimental-threads,register-docs,api-4-7"
rm bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
rm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/release/libgodot_rapier.dylib bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/release/libgodot_rapier.dylib /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib

xattr -rc bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
xattr -rc /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
