cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
export GODOT4_BIN=godot
export LLVM_PATH=/opt/homebrew/opt/llvm/bin
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="build2d-f64,test" --no-default-features
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
else
    cargo build --features="build2d,test" --no-default-features --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
