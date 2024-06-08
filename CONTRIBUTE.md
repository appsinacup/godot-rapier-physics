# Build the Rapier 2D wrapper

1. Prerequisites:
- Install `cargo` for Rust support


2. Compile the rust release Rapier2D library:

```
cargo build
```

3. Copy the output to bin folder:

```
cp target/debug/libgodot_rapier.dylib bin/addons/godot-rapier2d/bin/libphysics_server_rapier2d.macos.template_debug.framework/libphysics_server_rapier2d.macos.template_debug.dylib
```

# Compile the Rapier 2D extension

This step will build the final Godot Rapier extension library.

Steps to compile the extension:
- Open a command line prompt in the root folder
- Run `scons target=[TARGET] platform=[PLATFORM] debug_symbols=[DEBUG] dev_build=[DEBUG] -j[CORES]` with:
`[TARGET]`: `template_debug` for godot debug export and `template_release` for godot release export
`[PLATFORM]`: (optional) target platform if different from the current patform
`[DEBUG]`: (optional) `yes` to generate symbols and disable optimization for a debug build (useful only for debugging the extension)
`[CORES]`: (optional) number of cores to use in order to accelerate the build

```
scons target=template_debug generate_bindings=no
```

The library files will be found in `bin/addons/` folder.

# How to debug using VSCode

## Clone Godot repo

Take a look at `godot.code-workspace`. It's a workspace in VSCode. It loads both this folder, and another folder `../godot`. For this to work you have to clone the [godot](https://github.com/godotengine/godot) repo there.

## Build Godot

Then, you have to build the godot. Take a look at the tasks in `godot.code-workspace` for help. You can also try just running `scons` command without any arguments, and it will automatically select the platform and architecture based on what you are currently using.

## Run Godot from debugger inside VSCode

Click `Run and Debug` in VSCode on the left. With the workspace opened, select `Debug Scene Rapier`. This will start godot binary `godot.macos.editor.dev.arm64` and run scene from `godot-rapier-2d/Godot-Physics-Tests/test.tscn`. If you use a different binary(eg. windows), change the binary path. Same for the scene path.

Now you should be able to place breakpoints in the extension code. If you also want to place breakpoints in the rust code, you need to build that with debug. Take a look at `scripts/build-dev.sh` for help on how to do that.

# test_body_motion

This explains roughly the functionality of test_body_motion and how it's implemented.

## Step 1

We check if the body + margin collide something:
- if true: 
    
    we loop 4 times: 
        
        we depenetrate the body but not of the full penetration, 0,4 
- if false: go to step 2

## Step 2
                
We apply the motion to the body and build a aabb from the body start to the body end. 
We try the colliding distance (lower one). And we need two value: safe (when it does not collide), unsafe (where it collide)

## Step 3
            
- If collided:

    We put the body at the unsafe position to retrieve the collision info, we want the closer collision
- We send the info
