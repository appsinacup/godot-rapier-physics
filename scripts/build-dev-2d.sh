cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build --features="build2d,test" --no-default-features
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    # macOS
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
else
    # Linux
    echo "Running on Linux"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.so
    cp target/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.so
fi