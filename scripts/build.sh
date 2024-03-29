cd src/rapier2d-wrapper
cargo build --release --features="single,simd-stable,serde-serialize"
cd ../..

scons arch=arm64 target=template_debug debug_symbols=yes
rm -rf Godot-Physics-Tests/addons
cp -rf bin/addons Godot-Physics-Tests/addons

