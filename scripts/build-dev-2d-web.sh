cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty --features="single-dim2,serde-serialize,test,simd-stable,experimental-threads,register-docs,api-4-7"
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="experimental-wasm,single-dim2,serde-serialize,test,api-4-7" --target=wasm32-unknown-emscripten -Z build-std=std --verbose
    echo "Running on macOS"
    rm -f bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/debug/libgodot_rapier.dylib bin2d/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
    cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/godot_rapier.wasm
else
    cargo build --features="single-dim2,serde-serialize,test,simd-stable,experimental-threads,register-docs,api-4-7" --target=x86_64-unknown-linux-gnu
    echo "Running on Linux"
    cp target/x86_64-unknown-linux-gnu/debug/libgodot_rapier.so bin2d/addons/godot-rapier2d/bin/libgodot_rapier.linux.x86_64-unknown-linux-gnu.so
fi
