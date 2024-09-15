cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="build3d,test" --no-default-features
    echo "Running on macOS"
    rm -f bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
else
    cargo build --features="build3d,test" --no-default-features --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin3d/addons/godot-rapier3d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
