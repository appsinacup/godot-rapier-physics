cargo clippy --fix --allow-dirty
cargo +nightly build -Z build-std=panic_abort,std --features="simd-stable,experimental-wasm" --target wasm32-unknown-emscripten
cargo +nightly build -Z build-std=panic_abort,std --release --features="simd-stable,experimental-wasm" --target wasm32-unknown-emscripten
cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm bin/addons/godot-rapier2d/bin/libgodot_rapier.web.template_debug.wasm32.wasm
cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.web.template_debug.wasm32.wasm

cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm bin/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
