cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
#set EMCC_CFLAGS=-s ERROR_ON_UNDEFINED_SYMBOLS=0 --no-entry -gsource-map
#cargo build -Z build-std=panic_abort,std --features="single-dim2,experimental-wasm" --target wasm32-unknown-emscripten --no-default-features --release
cargo +nightly build -Zbuild-std --features="single-dim2,experimental-wasm" --target wasm32-unknown-emscripten --no-default-features

#cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
#cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm

cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/libgodot_rapier.web.wasm32.wasm
