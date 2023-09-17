mode=${1:-all}

if [ "$mode" != "release" ]; then
# Debug
echo "*** Building debug lib..."
scons arch=x86_64 platform=macos target=template_debug dev_build=yes debug_symbols=yes --jobs=$(sysctl -n hw.logicalcpu)
fi

if [ "$mode" != "debug" ]; then
# release
echo "*** Building release lib..."
scons arch=x86_64 platform=macos target=template_release dev_build=no debug_symbols=no --jobs=$(sysctl -n hw.logicalcpu)
fi

# copy gdextension files into projects
echo "*** Copy libs into projects..."
mkdir -p projects/physics-test/bin/
cp bin/* projects/physics-test/bin/
