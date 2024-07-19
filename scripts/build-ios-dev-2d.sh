cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build --features="single-dim2,parallel,simd-stable,serde-serialize" --no-default-features --target aarch64-apple-ios
rm bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
rm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/debug/libgodot_rapier.dylib /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib

xattr -rc bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
xattr -rc /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
