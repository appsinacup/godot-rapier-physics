#!/bin/sh
# Package the export and documentation-enabled editor builds from the same job.
set -eu

platform=$1
dimension=$2
arch=$3
folder=${4:-}
addon="bin${dimension}/addons/godot-rapier${dimension}/bin"

for variant in release editor; do
    source="target/$variant"
    destination="$addon"
    if [ "$variant" = editor ]; then
        destination="$addon/editor"
    fi
    mkdir -p "$destination"
    case "$platform" in
        linux|android)
            cp "$source/libgodot_rapier.so" "$destination/libgodot_rapier.$platform.$arch.so"
            cp "$source/libgodot_rapier.a" "$destination/libgodot_rapier.$platform.$arch.a"
            ;;
        windows)
            cp "$source/godot_rapier.dll" "$destination/libgodot_rapier.windows.$arch.dll"
            cp "$source/godot_rapier.lib" "$destination/libgodot_rapier.windows.$arch.a"
            ;;
        web)
            mkdir -p "$destination/$folder"
            cp "$source/godot_rapier.wasm" "$destination/$folder/godot_rapier.wasm"
            cp "$source/libgodot_rapier.a" "$destination/$folder/libgodot_rapier.a"
            ;;
        macos|ios)
            framework="libgodot_rapier.$platform"
            if [ "$platform" = ios ]; then
                framework="$framework.$arch"
            fi
            if [ "$variant" = editor ]; then
                # Keep the original framework metadata, not binaries from a previous build.
                mkdir -p "$destination/$framework.framework"
                if [ "$platform" = macos ]; then
                    cp -R "$addon/$framework.framework/Resources" "$destination/$framework.framework/"
                else
                    cp "$addon/$framework.framework/Info.plist" "$destination/$framework.framework/"
                fi
            fi
            if [ "$platform" = ios ]; then
                install_name_tool -id "@rpath/$framework.framework/$framework" "$source/libgodot_rapier.dylib"
                cp "$source/libgodot_rapier.dylib" "$destination/$framework.framework/$framework"
                cp "$source/libgodot_rapier.a" "$destination/$framework.framework/libgodot_rapier.a"
            else
                cp "$source/libgodot_rapier.dylib" "$destination/$framework.framework/$framework.dylib"
                cp "$source/libgodot_rapier.arm64.a" "$destination/$framework.framework/$framework.arm64.a"
                cp "$source/libgodot_rapier.x86_64.a" "$destination/$framework.framework/$framework.x86_64.a"
            fi
            ;;
        *) echo "Unsupported platform: $platform" >&2; exit 1 ;;
    esac
done
