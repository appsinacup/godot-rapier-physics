cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build --release --features="single-dim3,simd-stable,parallel,serde-serialize" --no-default-features
rm bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
rm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/release/libgodot_rapier.dylib bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/release/libgodot_rapier.dylib /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib

xattr -rc bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
xattr -rc /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
