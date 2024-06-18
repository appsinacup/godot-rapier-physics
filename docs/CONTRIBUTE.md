# How Contribute

If you want to contribute to this project:
- make sure you understand the code by reading how it's built chapter.
- create an issue for the thing you want to fix, if there isn't one yet.
- follow steps below to build the project and lint.
- optionally join [discord](https://discord.gg/56dMud8HYn) and ask for help there from maintainers

# How it's build

Godot let's any GDExtension create a new `Physics Server` if they call create a node of type [PhysicsServer2DManager](https://docs.godotengine.org/en/latest/classes/class_physicsserver2dmanager.html) or [PhysicsServer3DManager](https://docs.godotengine.org/en/latest/classes/class_physicsserver3dmanager.html) and on the callback return a new instance of their custom `Physics Server` that extends either [PhysicsServer2DExtension](https://docs.godotengine.org/en/latest/classes/class_physicsserver2dextension.html) or [PhysicsServer3DExtension](https://docs.godotengine.org/en/latest/classes/class_physicsserver3dextension.html).

While these classes aren't that well documented, the original implementation of these classes (eg. `Godot Physics Servers`) are.

A physics server has to implement the `PhysicsServerExtension`, but also other classes that are used by it, such as:
- [PhysicsDirectBodyState2DExtension](https://docs.godotengine.org/en/latest/classes/class_physicsdirectbodystate2dextension.html) or [PhysicsDirectBodyState3DExtension](https://docs.godotengine.org/en/latest/classes/class_physicsdirectbodystate3dextension.html)
- [PhysicsDirectSpaceState2DExtension](https://docs.godotengine.org/en/latest/classes/class_physicsdirectspacestate2dextension.html) or [PhysicsDirectSpaceState3DExtension](https://docs.godotengine.org/en/latest/classes/class_physicsdirectspacestate3dextension.html)

Godot Rapier Physics implements these under the name like this:
- [RapierPhysicsServer2D](../src/servers/rapier_physics_server_2d.rs) and [RapierPhysicsServer3D](../src/servers/rapier_physics_server_3d.rs)
- [RapierDirectBodyState2D](../src/bodies/rapier_direct_body_state_2d.rs) and [RapierDirectBodyState3D](../src/bodies/rapier_direct_body_state_3d.rs)
- [RapierDirectSpaceState2D](../src/spaces/rapier_direct_space_state_2d.rs) and [RapierDirectSpaceState3D](../src/spaces/rapier_direct_space_state_3d.rs)

Note that Godot Rapier Physics also exposes new nodes, [Fluid2D](../src/fluids/fluid_2d.rs) and [Fluid3D](../src/fluids/fluid_3d.rs).
# Build the Rapier 2D wrapper

1. Prerequisites:
- Install [cargo](https://doc.rust-lang.org/cargo/getting-started/installation.html)

2. Update dependencies to latest:

```bash
cargo update
```

3. Build the project
```bash
cargo build --release
```

By default it builds for 2d, if you want to specify additional build features, you can do so with the `--features` flag:

```bash
cargo build --release --features="single-dim3,parallel,simd-stable" --no-default-features
```

Note the `--no-default-features`, because by default it has the `single-dim2` feature

3. Copy the output to bin folder of the addon:

Eg. macOS
```
cp target/release/libgodot_rapier.dylib bin2/addons/godot-rapier2d/bin/libgodot_rapier.macos.framework/libgodot_rapier.macos.dylib
```
Eg. Windows
```
cp target/release/godot_rapier.dll bin2/addons/godot-rapier2d/bin/godot_rapier.windows.x86_64-pc-windows-msvc.dll
```

For the correct path to use inside the bin folder, look inside the `bin2d/addons/godot-rapier2d.gdextension` or the `bin3d/addons/godot-rapier3d.gdextension`.

## Available features

For features, the following are available:
- single-dim2
- single-dim3
- double-dim2
- double-dim3
- parallel
- simd-stable
- enhanced-determinism

The `single` and `double` refer to the [precision](https://docs.godotengine.org/en/stable/tutorials/physics/large_world_coordinates.html) used in Godot.

The `dim2` or `dim3` refers to the rapier and salva versions used, and to what classes will be implemented for Godot and what Physics Server will be replaced(2d or 3d).

The `parallel` version doesn't work on web.

The `enhanced-determinism` usually slows down the simulation.

# How to debug using VSCode

## Run Godot from debugger inside VSCode

Click `Run and Debug` in VSCode on the left. Add a configuration to the `launch.configurations` similar to this (eg. below one is for macOS):
```json
{
    "name": "Launch",
    "program": "path/to/godot/bin/godot.macos.editor.dev.arm64",
    "type": "cppdbg",
    "request": "launch",
    "cwd": "${workspaceFolder:godot-rapier-2d}",
    "osx": {
        "MIMode": "lldb"
    },
    "args": [
        "--path",
        "path/to/project/folder",
        "--debug-collisions",
        "scene-name.tscn"
    ]
},
```

More on godot cli usage [here](https://docs.godotengine.org/en/stable/tutorials/editor/command_line_tutorial.html).
