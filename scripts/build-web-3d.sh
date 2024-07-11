cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty
cargo build -Zbuild-std --features="single-dim3,experimental-wasm,serde-serialize" --target wasm32-unknown-emscripten --no-default-features --release

cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm bin3d/addons/godot-rapier3d/bin/godot_rapier.wasm
cp target/wasm32-unknown-emscripten/release/godot_rapier.wasm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier3d/bin/godot_rapier.wasm

#cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm bin2d/addons/godot-rapier2d/bin/godot_rapier.wasm
#cp target/wasm32-unknown-emscripten/debug/godot_rapier.wasm /Users/dragosdaian/Documents/Godot-Physics-Tests/addons/godot-rapier2d/bin/godot_rapier.wasm
