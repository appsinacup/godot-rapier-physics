cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
export GODOT4_BIN=/Applications/Godot.app/Contents/MacOS/Godot
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="build2d,test" --no-default-features # --verbose
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.a bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.a
else
    cargo build --features="build2d,test" --no-default-features --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
