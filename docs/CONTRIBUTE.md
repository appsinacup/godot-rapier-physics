# Build the Rapier 2D wrapper

1. Prerequisites:
- Install `cargo`


2. Compile the Godot Rapier library for 2D:

```bash
cargo update
cargo build --features="single_dim2,parallel,simd-stable" --no-default-features --release
// or

cargo build --features="single_dim3,parallel,simd-stable" --no-default-features --release
```

For features, thte following are available:
- single_dim2
- single_dim3
- double_dim2
- double_dim3
- parallel
- simd_stable

3. Copy the output to bin folder of the addon:

Eg. macOS
```
cp target/release/libgodot_rapier.dylib bin2/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
```
Eg. Windows
```
cp target/release/godot_rapier.dll bin2/addons/godot-rapier2d/bin/godot_rapier.windows.dll
```
# How to debug using VSCode

## Clone Godot repo

Take a look at `godot.code-workspace`. It's a workspace in VSCode. It loads both this folder, and another folder `../godot`. For this to work you have to clone the [godot](https://github.com/godotengine/godot) repo there.

## Build Godot

Then, you have to build the godot. Take a look at the tasks in `godot.code-workspace` for help. You can also try just running `scons` command without any arguments, and it will automatically select the platform and architecture based on what you are currently using.

## Run Godot from debugger inside VSCode

Click `Run and Debug` in VSCode on the left. With the workspace opened, select `Debug Scene Rapier`. This will start godot binary `godot.macos.editor.dev.arm64` and run scene from `godot-rapier-2d/Godot-Physics-Tests/test.tscn`. If you use a different binary(eg. windows), change the binary path. Same for the scene path.

Now you should be able to place breakpoints in the extension code. If you also want to place breakpoints in the rust code, you need to build that with debug. Take a look at `scripts/build-dev.sh` for help on how to do that.
