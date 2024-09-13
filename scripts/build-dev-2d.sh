cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build --features="build2d,test" --no-default-features
rm bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
rm /Users/dragosdaian/Documents/GitHub/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
cp target/debug/libgodot_rapier.dylib /Users/dragosdaian/Documents/GitHub/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib

xattr -rc bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
xattr -rc /Users/dragosdaian/Documents/GitHub/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
