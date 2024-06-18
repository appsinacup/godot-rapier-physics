<p align="center">
<img src="https://github.com/appsinacup/godot-rapier-physics/blob/main/logo.jpg?raw=true" width="256px"/>
</p>
<p align="center">
        <img src="https://github.com/appsinacup/godot-rapier-physics/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="Godot Rapier Build"></a>
        <img src="https://img.shields.io/badge/Godot-4.3-%23478cbf?logo=godot-engine&logoColor=white" />
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>

A 2D and 3D physics drop-in replacement for [godot](https://github.com/godotengine/godot) that adds stability and fluids. Uses [rapier](https://github.com/dimforge/rapier) for physics and [salva](https://github.com/dimforge/salva) for fluids.

<p align="center">
<img src="docs/rapier-vid.gif"/>
</p>

# Features

- Better performance (parallel simulation and SIMD).
- Better stability.
- 2D and 3D.
- Fluids with surface tension, viscosity and elasticity.

# Limitations

- SeparationRayShape2D, ConcavePolygonShape3D, HeightMapShape3D, 3D joints.
- Web exports.
- Android exports.
- Cross platform determinism.
- Double builds.

# Supported Platforms

- Windows (x86_64, x86_32)
- macOS (x86-64 + arm64 Universal)
- Linux (x86_64)
- *DISABLED* Android (x86_64, arm64)
- iOS (arm64)
- *DISABLED* Web (wasm32)

# Installation Rapier 2D

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2267) using the `AssetLib` tab in Godot.

- Manual: Download the github release and move only the `addons` folder into your project `addons` folder.

After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Rapier2D`.

# Installation Rapier 3D

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2267) using the `AssetLib` tab in Godot.

- Manual: Download the github release and move only the `addons` folder into your project `addons` folder.

After installing, go to `Advanced Settings` -> `Physics` -> `3D`. Change `Physics Engine` to `Rapier3D`.

# Contribute

If you want to contribute, view [docs/CONTRIBUTE.md](docs/CONTRIBUTE.md) for more info.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.
