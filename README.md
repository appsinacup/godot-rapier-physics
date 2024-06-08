<p align="center">
<img src="https://github.com/appsinacup/godot-rapier2d/blob/main/logo.jpg?raw=true" width="256px"/>
</p>
<p align="center">
	<a href="https://github.com/appsinacup/godot-rapier-2d/actions/workflows/runner.yml">
        <img src="https://github.com/appsinacup/godot-rapier-2d/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="Godot Rapier Build"></a>
    <a href="https://github.com/dimforge/rapier/releases/tag/v0.19.0" alt="Rapier Version">
        <img src="https://img.shields.io/badge/Rapier-v0.19.0-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/dimforge/salva/releases/tag/v0.7.0" alt="Salva Version">
        <img src="https://img.shields.io/badge/Salva2D-v0.7.0-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/godotengine/godot-cpp" alt="Godot Version">
        <img src="https://img.shields.io/badge/Godot-v4-%23478cbf?logo=godot-engine&logoColor=white" /></a>
    <a href="https://github.com/appsinacup/godot-rapier/graphs/contributors" alt="Contributors">
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>

A [rapier](https://github.com/dimforge/rapier) physics server for [Godot Engine v4](https://github.com/godotengine/godot), implemented as a GDExtension. Also integrates with salva for fluids.

<p align="center">
<img src="rapier-vid.gif"/>
<img src="fluid_shader.gif"/>
</p>

# Features

- Parallel support and SIMD build for better performance.
- Better physics stability.
- Fluids with surface tension, viscosity and elasticity.

# Limitations

- SeparationRay2D missing [issues/5](https://github.com/appsinacup/godot-rapier-2d/issues/5)
- Web exports not working [issues/23](https://github.com/appsinacup/godot-rapier-2d/issues/23)
- Cross platform determinism isn't working [issues/47](https://github.com/appsinacup/godot-rapier-2d/issues/47)
- Double build isn't working [issues/61](https://github.com/appsinacup/godot-rapier-2d/issues/61)

# Supported Platforms

- Windows (x86_64, x86_32)
- macOS (x86-64 + arm64 Universal)
- Linux (x86_64)
- *DISABLED* Android (x86_64, arm64)
- iOS (arm64)
- *DISABLED* Web (wasm32)

# Installation

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2267) using the `AssetLib` tab in Godot.

- Manual: Download the github release and move only the `addons` folder into your project `addons` folder.

- Build it yourself. Read more about it in the [contribute file](CONTRIBUTE.md).

After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Rapier2D`.

# Samples

After installing the addon, the samples are in the `samples/godot-rapier2d` folder. In order to run them, you have to enable the physics engine for some extra features to work (eg. fluids) as described above.

# Contribute

If you want to contribute, view [CONTRIBUTE.md](CONTRIBUTE.md) for more info.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.
