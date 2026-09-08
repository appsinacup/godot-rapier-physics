cargo fmt -- --config-path rustfmt.toml
FEATURES="single-dim3,serde-serialize,test,register-docs,api-4-7"
cargo clippy --fix --allow-dirty --features="$FEATURES"
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="$FEATURES"
    echo "Running on macOS"
    rm -f bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin3d/addons/godot-rapier3d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
elif [ "$OSTYPE" = "cygwin" ]; then
    cargo build --features="$FEATURES" --target=x86_64-pc-windows-msvc
    echo "Running on Windows"
    cp target/x86_64-pc-windows-msvc/debug/godot_rapier.dll bin3d/addons/godot-rapier3d/bin/libgodot_rapier.windows.x86_64-pc-windows-msvc.dll
else
    cargo build --features="$FEATURES" --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin3d/addons/godot-rapier3d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
