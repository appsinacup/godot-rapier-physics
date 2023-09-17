cd src\rapier2d-wrapper
call build.bat
cd ..\..

echo "*** Building debug lib..."
call scons arch=x86_64 platform=windows target=template_debug dev_build=yes debug_symbols=yes --jobs=8

echo "*** Building release lib..."
call scons arch=x86_64 platform=windows target=template_release dev_build=no debug_symbols=no --jobs=8

echo "*** Copy libs into projects..."
if not exist "projects\physics-test\bin" mkdir "projects\physics-test\bin"
copy bin\* projects\physics-test\bin\