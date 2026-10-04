set -eu
cargo fmt -- --config-path rustfmt.toml
FEATURES="double-dim2,serde-serialize,test,parallel,register-docs,api-4-7"
cargo clippy --fix --allow-dirty --features="$FEATURES"
mkdir -p bin2d/addons/godot-rapier2d/bin/editor/libgodot_rapier.macos.framework
cp -R bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/Resources bin2d/addons/godot-rapier2d/bin/editor/libgodot_rapier.macos.framework/
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="$FEATURES"
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/editor/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/editor/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
elif [ "$OSTYPE" = "cygwin" ]; then
    cargo build --features="$FEATURES" --target=x86_64-pc-windows-msvc
    echo "Running on Windows"
    cp target/x86_64-pc-windows-msvc/debug/godot_rapier.dll bin2d/addons/godot-rapier2d/bin/editor/libgodot_rapier.windows.x86_64-pc-windows-msvc.dll
else
    cargo build --features="$FEATURES" --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/editor/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
