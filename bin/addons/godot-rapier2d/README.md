<div align="center">
  <h1>Godot Rapier2D</h1> 

  <a href="https://github.com/godotengine/godot/releases/tag/4.1-stable"><img src="https://img.shields.io/badge/Godot-v4.1-%23478cbf?logo=godot-engine&logoColor=white"/></a>
  <a href="https://github.com/dimforge/rapier/releases/tag/v0.17.2"><img src="https://img.shields.io/badge/Rapier2D-v0.17.2-%23478cbf?logoColor=white"/></a>
  <a href="https://github.com/appsinacup/godot-rapier2d/actions/workflows/runner.yml?branch=main">![🔗 Build Status](https://github.com/appsinacup/godot-rapier2d/actions/workflows/runner.yml/badge.svg?branch=main)</a>
</div>

<img src="https://github.com/appsinacup/godot-rapier2d/blob/main/logo.jpg?raw=true"/> 

## Build the Rapier 2D Extension

### Build godot-cpp

Official C++ bindings for Godot API (https://github.com/godotengine/godot-cpp)

Sources are automatically synced into `godot-cpp/` folder.

Steps to build godot-cpp:
- Open a command line prompt in `godot-cpp/` folder
- Run `scons target=[TARGET] platform=[PLATFORM] debug_symbols=[DEBUG] dev_build=[DEBUG] -j[CORES]` with:
`[TARGET]`: `template_debug` for godot debug export and `template_release` for godot release export
`[PLATFORM]`: (optional) target platform if different from the current patform
`[DEBUG]`: (optional) `yes` to generate symbols and disable optimization for a debug build (useful only for debugging the extension)
`[CORES]`: (optional) number of cores to use in order to accelerate the build

Example command line for a debug version for the editor targeting the current platform:
`scons target=editor debug_symbols=yes dev_build=yes -j10`

See [Building the C++ bindings](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/gdextension_cpp_example.html#building-the-c-bindings) from the official documentation for more details about building the bindings.

### Build the Rapier 2D wrapper

Based on official Rapier lib (https://github.com/dimforge/rapier) with some changes.

The rapier we use uses a different parry. Here is a [PR with changes for rapier](https://github.com/appsinacup/rapier/pull/1). Here is a [PR with changes for parry](https://github.com/appsinacup/parry/pull/1).

Rapier sources will be automatically retrieved while building.

Prerequisites:
- Install `cargo` for Rust support

Quick step to build the wrapper:
- Windows: run `src/rapier2d-wrapper/build.bat`
- Mac/Linux: run `src/rapier2d-wrapper/build.sh`

Those scripts will automatically:
- Compile a debug version of the rust Rapier 2D library (in `src/rapier2d-wrapper/target/debug/`)
- Compile a release version of the rust Rapier 2D library (in `src/rapier2d-wrapper/target/release/`)
- Generate the C bindings header file (in `src/rapier2d-wrapper/includes/`)

### Compile the Rapier 2D extension

This step will build the final Godot Rapier extension library.

Steps to compile the extension:
- Open a command line prompt in the root folder
- Run `scons target=[TARGET] platform=[PLATFORM] debug_symbols=[DEBUG] dev_build=[DEBUG] -j[CORES]` with:
`[TARGET]`: `template_debug` for godot debug export and `template_release` for godot release export
`[PLATFORM]`: (optional) target platform if different from the current patform
`[DEBUG]`: (optional) `yes` to generate symbols and disable optimization for a debug build (useful only for debugging the extension)
`[CORES]`: (optional) number of cores to use in order to accelerate the build

Example command line for a debug version targeting the current platform:
`scons target=template_debug -j10`

Example command line for a debug version with debug symbols targeting the current platform:
`scons target=template_debug debug_symbols=yes dev_build=yes -j10`

Example command line for a release version targeting the current platform:
`scons target=template_release -j10`

The library files will be found in `bin/` folder.

## Use the Rapier 2D extension

Copy the `bin/` folder to your project root.

If you need to use a debug version with debug symbols, open `physics_server_rapier2D.gdextension` and rename all `template_debug` to `template_debug.dev`.
This is useful only if you wish to debug the Rapier extension itself.
