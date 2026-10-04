cargo fmt -- --config-path rustfmt.toml
FEATURES="experimental-wasm,single-dim2,serde-serialize,test,register-docs,api-4-7"
cargo clippy --fix --allow-dirty --features="$FEATURES"
if [ "${OSTYPE#darwin}" != "$OSTYPE" ]; then
    cargo build --features="$FEATURES" --target=wasm32-unknown-emscripten -Z build-std=std --verbose
    echo "Running on macOS"
    mkdir -p bin2d/addons/godot-rapier2d/bin/editor
    cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/editor/godot_rapier.wasm
else
    echo "Unsupported. Use macOS for building wasm32-unknown-emscripten target."
    exit 1
fi
