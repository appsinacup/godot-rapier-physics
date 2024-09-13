cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build --features="build3d,test" --no-default-features
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    echo "Running on macOS"
    rm bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
else
    # Linux
    echo "Running on Linux"
    rm bin3d/addons/godot-rapier3d/bin/libgodot_rapier.linux.so
    cp target/debug/libgodot_rapier.so bin3d/addons/godot-rapier3d/bin/libgodot_rapier.linux.so
fi