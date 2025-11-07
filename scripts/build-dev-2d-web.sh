export GODOT4_BIN=/Applications/Godot.app/Contents/MacOS/Godot
export LLVM_PATH=clang
cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo +nightly build --features="experimental-wasm,experimental-wasm-nothreads,single-dim2,serde-serialize,test,api-custom" --no-default-features --target=wasm32-unknown-emscripten -Z build-std=std --verbose
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/godot_rapier.wasm
else
    cargo build --features="build2d,test" --no-default-features --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
