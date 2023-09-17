echo "*** Building debug lib..."
cargo build
echo "*** Building release lib..."
cargo build --release
echo "*** Building header..."
cbindgen --config cbindgen.toml --crate rapier2d-wrapper --output includes/rapier2d_wrapper.h

echo "*** Rename the debug files"
rm -rf target/debug/librapier2d_wrapper.macos.*
mv target/debug/librapier2d_wrapper.a target/debug/librapier2d_wrapper.macos.x86_64.a
mv target/debug/librapier2d_wrapper.d target/debug/librapier2d_wrapper.macos.x86_64.d
echo "*** Rename the release files"
rm -rf target/release/librapier2d_wrapper.macos.*
mv target/release/librapier2d_wrapper.a target/release/librapier2d_wrapper.macos.x86_64.a
mv target/release/librapier2d_wrapper.d target/release/librapier2d_wrapper.macos.x86_64.d
echo "*** Done ..."
