scons arch=arm64 target=template_debug
rm -rf Godot-Physics-Tests/addons
cp -rf bin/addons Godot-Physics-Tests/addons

