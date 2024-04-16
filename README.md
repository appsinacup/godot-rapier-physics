<p align="center">
<img src="https://github.com/appsinacup/godot-rapier2d/blob/main/logo.jpg?raw=true" width="256px"/>
</p>
<p align="center">
	<a href="https://github.com/appsinacup/godot-rapier-2d/actions/workflows/runner.yml">
        <img src="https://github.com/appsinacup/godot-rapier-2d/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="Godot Rapier2D Build"></a>
    <a href="https://github.com/dimforge/rapier/releases/tag/v0.18.0" alt="Rapier2D Version">
        <img src="https://img.shields.io/badge/Rapier2D-v0.18.0-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/dimforge/salva/releases/tag/v0.7.0" alt="Salva Version">
        <img src="https://img.shields.io/badge/Salva2D-v0.7.0-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/godotengine/godot-cpp" alt="Godot Version">
        <img src="https://img.shields.io/badge/Godot-v4.2-%23478cbf?logo=godot-engine&logoColor=white" /></a>
    <a href="https://github.com/appsinacup/godot-rapier-2d/graphs/contributors" alt="Contributors">
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>

A 2d [rapier](https://github.com/dimforge/rapier) physics server for [Godot Engine v4.2](https://github.com/godotengine/godot), implemented as a GDExtension. Also integrates with salva for 2d fluids.

# Features

- Parallel support (for non enhnanced determinism builds)
- SIMD (Single instruction, multiple data) build.
- Better physics stability.
- Fluids with surface tension, viscousity and elastic liquids.

<p align="center">
<img src="rapier-vid.gif"/>
</p>

<p align="center">
<img src="Fluid2d.gif"/>
</p>

# Limitations

- SeparationRay2D missing [issues/5](https://github.com/appsinacup/godot-rapier-2d/issues/5)
- Web exports not working [issues/23](https://github.com/appsinacup/godot-rapier-2d/issues/23)
- Cross platform determinism isn't working [issues/47](https://github.com/appsinacup/godot-rapier-2d/issues/47)

# Supported Platforms

- Windows (x86_64, x86_32)
- macOS (x86-64 + arm64 Universal)
- Linux (x86_64)
- Android (x86_64, arm64)
- iOS (arm64)
- Web (wasm32)

# Installation

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2267) using the `AssetLib` tab in Godot.

- Manual: Download the github release and move only the `addons` folder into your project `addons` folder.

- Build it yourself. Read more about it in the [documentation](DOCUMENTATION.md).

After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Rapier2D`.

# Contribute

If you want to contribute, view [CONTRIBUTE.md](CONTRIBUTE.md) for more info.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.
