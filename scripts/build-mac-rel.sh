cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build --release --features="single_dim2,parallel,simd-stable" --no-default-features
cp target/release/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/release/libgodot_rapier.dylib /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib

xattr -rc bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
xattr -rc /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
