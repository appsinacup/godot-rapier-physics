cd src/rapier2d-wrapper
cargo build --release --features="single,simd-stable,parallel"
cd ../..

scons arch=arm64 target=template_release
rm -rf Godot-Physics-Tests/addons/godot-rapier2d
cp -rf bin/addons/godot-rapier2d Godot-Physics-Tests/addons/godot-rapier2d

