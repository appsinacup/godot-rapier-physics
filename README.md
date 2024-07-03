<p align="center">
<img src="https://github.com/appsinacup/godot-rapier-physics/blob/main/logo.jpg?raw=true"/>
</p>
<p align="center">
        <img src="https://github.com/appsinacup/godot-rapier-physics/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="Godot Rapier Build"></a>
        <img src="https://img.shields.io/badge/Godot-4.2-%23478cbf?logo=godot-engine&logoColor=white" />
</p>

<p align = "center">
    <strong>
        <a href="https://github.com/appsinacup/godot-rapier-physics/blob/main/docs/DOCUMENTATION.md">Documentation</a> | <a href="https://discord.gg/56dMud8HYn">Discord</a>
    </strong>
</p>


-----

<p align = "center">
<b>2D and 3D physics engine</b>
<i>for the Godot game engine.</i>
</p>

-----

Godot Rapier Physics is a 2D and 3D physics drop-in replacement for the [Godot game engine](https://github.com/godotengine/godot) that adds stability through [rapier](https://github.com/dimforge/rapier) and fluids through [salva](https://github.com/dimforge/salva).

# Performance

Creating objects until FPS drops below 30. Running on a macbook m2 pro. Everything is run inside the godot editor using the [Godot Physics Tests](https://github.com/fabriceci/Godot-Physics-Tests) repository.

Each cell shows max shape count. Higher number is better.

Shape|Godot 2D|Godot 3D|Rapier 2D|Rapier 3D|Jolt 3D
-|-|-|-|-|-
Sphere|5000|3300|5000|4200|8000
Box|3500|3200|4500|4000|7200
Capsule|4500|2700|4500|3700|7400
Convex Polygon|3500|3100|4000|4000|8000

# Note

This plugin was recently rewritten from c++ to rust. There are still some things missing from what it had originally and may have some new bugs that it didn't have before.

The reason it was rewritten is to do easier cross platform determinism part and exports to web.

# Installation

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2267) using the `AssetLib` tab in Godot:
    - [Rapier Physics 2D - Parallel SIMD](https://godotengine.org/asset-library/asset/2267)
    - [Rapier Physics 2D - Cross Platform Determinism](https://godotengine.org/asset-library/asset/2815)
    - [Rapier Physics 3D - Parallel SIMD](https://godotengine.org/asset-library/asset/3084)
    - [Rapier Physics 3D - Cross Platform Determinism](https://godotengine.org/asset-library/asset/3085)

- Manual: Download the [latest github release](https://github.com/appsinacup/godot-rapier-physics/releases/latest) and move only the `addons` folder into your project `addons` folder.

After installing, go to `Advanced Settings` -> `Physics` -> `2D` or `3D`. Change `Physics Engine` to `Rapier2D` or `Rapier3D`.

<p align="center">
<img src="docs/rapier-vid.gif"/>
</p>

# Limitations

- SeparationRayShape2D, ConcavePolygonShape3D, HeightMapShape3D, 3D joints.
- Web exports.
- Android exports.
- Double builds.
- Liquids Missing.
- No support for asymetric collisions (eg. object 1 hitting object 2 but object 2 not hitting object 1). More info here [Rapier Collision groups and solver groups](https://rapier.rs/docs/user_guides/rust/colliders/#collision-groups-and-solver-groups). This is the exact check rapier does: `(A.layer & B.mask) != 0 && (B.layer & A.mask) != 0`
- Friction works differently than it does in Godot. The current formula is: friction is multiplied by other friction, bounce is taken the max value.
- Setting Center of Mass to Custom or Custom Inertia doesn't work right now.

# Platforms

- Windows (x86_64, x86_32)
- macOS (x86-64 + arm64 Universal)
- Linux (x86_64)
- *DISABLED* Android (x86_64, arm64)
- iOS (arm64)
- *DISABLED* Web (wasm32)
