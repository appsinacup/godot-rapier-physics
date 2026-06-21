cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty --features="double-dim2,serde-serialize,test,parallel,experimental-threads,register-docs,api-4-7,api-custom"
export GDRUST_GODOT_BIN="${GDRUST_GODOT_BIN:-/Applications/Godot.app/Contents/MacOS/Godot}"
export LLVM_PATH=/opt/homebrew/opt/llvm/bin
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="double-dim2,serde-serialize,test,parallel,experimental-threads,register-docs,api-4-7,api-custom"
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
else
    cargo build --features="single-dim2,serde-serialize,test,simd-stable,experimental-threads,register-docs,api-4-7,api-custom" --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
