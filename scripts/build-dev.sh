cd src/rapier2d-wrapper
cargo build --features="single,simd-stable,serde-serialize,parallel"
cd ../..

scons arch=arm64 target=template_debug debug_symbols=yes dev_build=yes
rm -rf Godot-Physics-Tests/addons
cp -rf bin/addons Godot-Physics-Tests/addons

