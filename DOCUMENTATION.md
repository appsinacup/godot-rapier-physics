# Documentation

## Build the Rapier 2D Extension

### Build godot-cpp

Official C++ bindings for Godot API (https://github.com/godotengine/godot-cpp)

Steps to build godot-cpp:
- Open a command line prompt in `godot-cpp/` folder
- Run `scons target=[TARGET] platform=[PLATFORM] debug_symbols=[DEBUG] dev_build=[DEBUG] -j[CORES]` with:
`[TARGET]`: `template_debug` for godot debug export and `template_release` for godot release export
`[PLATFORM]`: (optional) target platform if different from the current patform
`[DEBUG]`: (optional) `yes` to generate symbols and disable optimization for a debug build (useful only for debugging the extension)
`[CORES]`: (optional) number of cores to use in order to accelerate the build

Example:

```
cd godot-cpp
scons target=template_debug generate_bindings=yes
```

See [Building the C++ bindings](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/gdextension_cpp_example.html#building-the-c-bindings) from the official documentation for more details about building the bindings.

### Build the Rapier 2D wrapper

Prerequisites:
- Install `cargo` for Rust support
Generate the C bindings header file (in `src/rapier2d-wrapper/includes/`)

Go to `src/rapier2d-wrapper` folder and run `cbindgen`

```
cd src/rapier2d-wrapper
cargo install --force cbindgen
cbindgen --config cbindgen.toml --crate rapier2d-wrapper --output includes/rapier2d_wrapper.h
```

Compile the rust release Rapier2D library:

```
cargo build --release
```

### Compile the Rapier 2D extension

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

## Function Documentation

## test_body_motion

### Step 1

We check if the body + margin collide something:
- if true: 
    
    we loop 4 times: 
        
        we depenetrate the body but not of the full penetration, 0,4 
- if false: go to step 2

### Step 2
                
We apply the motion to the body and build a aabb from the body start to the body end. 
We try the colliding distance (lower one). And we need two value: safe (when it does not collide), unsafe (where it collide)

### Step 3
            
- If collided:

    We put the body at the unsafe position to retrieve the collision info, we want the closer collision
- We send the info
