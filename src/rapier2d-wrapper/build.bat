echo *** Building debug lib...
del target\debug\*.lib
cargo build
copy target\debug\rapier2d_wrapper.lib target\debug\librapier2d_wrapper.windows.x86_64.lib
del target\debug\rapier2d_wrapper.lib

echo *** Building release lib...
del target\release\*.lib
cargo build --release
copy target\release\rapier2d_wrapper.lib target\release\librapier2d_wrapper.windows.x86_64.lib
del target\release\rapier2d_wrapper.lib

echo *** Building header...
cbindgen --config cbindgen.toml --crate rapier2d-wrapper --output includes/rapier2d_wrapper.h