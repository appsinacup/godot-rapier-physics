cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo +nightly build -Z build-std=panic_abort,std --features="single-dim2,experimental-wasm,wasm-bindgen" --target wasm32-unknown-emscripten --no-default-features --verbose

cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
