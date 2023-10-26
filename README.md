<div align="center">
  <h1>Godot Rapier2D</h1>
</div>

<p align="center">
	<a href="https://github.com/appsinacup/godot-rapier2d/actions/workflows/runner.yml">
        <img src="https://github.com/appsinacup/godot-rapier2d/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="Godot Rapier2D Build"></a>
    <a href="https://github.com/dimforge/rapier/releases/tag/v0.17.2" alt="Rapier2D Version">
        <img src="https://img.shields.io/badge/Rapier2D-v0.17.2-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/godotengine/godot-cpp" alt="Godot Version">
        <img src="https://img.shields.io/badge/Godot-v4.1-%23478cbf?logo=godot-engine&logoColor=white" /></a>
    <a href="https://github.com/appsinacup/godot-rapier2d/graphs/contributors" alt="Contributors">
        <img src="https://img.shields.io/github/contributors/appsinacup/godot-rapier2d" /></a>
    <a href="https://github.com/appsinacup/godot-rapier2d/pulse" alt="Activity">
        <img src="https://img.shields.io/github/commit-activity/m/appsinacup/godot-rapier2d" /></a>
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>

<img src="https://github.com/appsinacup/godot-rapier2d/blob/main/logo.jpg?raw=true"/> 

A 2d [rapier](https://github.com/dimforge/rapier) physics server for [Godot Engine](https://github.com/godotengine/godot), implemented as a GDExtension.

# Limitations

- One way direction missing for CharacterBody2D (WIP)
- Static Body Constant Speed or Conveyer Belt missing for CharacterBody2D vs StaticBody2D (WIP)
- Spring Joint missing(WIP)
- Shape skewing doesn't work (WIP)

# Build Features

## Float Precision

This plugin is built for both single and double precision builds.

## Rapier Features

This package supports the following features of rapier ([more on this here](https://rapier.rs/docs/user_guides/rust/getting_started)):

- simd-stable: enables explicit SIMD optimizations using the wide crate. Has limited cross-platform support but can be used with a stable version of the Rust compiler.
- enhanced-determinism: enables cross-platform determinism (assuming the rest of your code is also deterministic) across all 32-bit and 64-bit platforms that implements the IEEE 754-2008 standard strictly. This includes most modern processors as well as WASM targets.

# Installation

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2267) using the `AssetLib` tab in Godot.

- Manual: Download the github release and move only the `addons` folder into your project `addons` folder.

- Build it yourself. Read more about it in the [documentation](DOCUMENTATION.MD).

After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Rapier2D`.

# Roadmap

- Fix all other issues from Limitations.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.

# License

All code in this repository is provided under the MIT license. See `LICENSE` for more details and `THIRDPARTY.txt` for third-party licenses.
This repo is a continuation of https://github.com/fabriceci/godot-rapier2d .
