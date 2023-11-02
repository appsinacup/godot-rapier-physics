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
        <img src="https://img.shields.io/badge/Godot-v4.2-%23478cbf?logo=godot-engine&logoColor=white" /></a>
    <a href="https://github.com/appsinacup/godot-rapier2d/graphs/contributors" alt="Contributors">
        <img src="https://img.shields.io/github/contributors/appsinacup/godot-rapier2d" /></a>
    <a href="https://github.com/appsinacup/godot-rapier2d/pulse" alt="Activity">
        <img src="https://img.shields.io/github/commit-activity/m/appsinacup/godot-rapier2d" /></a>
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>

<img src="https://github.com/appsinacup/godot-rapier2d/blob/main/logo.jpg?raw=true"/> 


<p align="center">
<img src="rapier-vid.gif"/>
</p>

A 2d [rapier](https://github.com/dimforge/rapier) physics server for [Godot Engine](https://github.com/godotengine/godot), implemented as a GDExtension.

## Table of Contents

1. [Limitations](#limitations)
2. [Supported Platforms](#supported-platforms)
3. [Installation](#installation)
4. [Features](#features)
5. [Comparison](#comparison)
6. [License](#license)

# Limitations

- SeparationRay2D missing.
- DampedSpringJoint2D missing.
- Shape skew missing.

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

Video Tutorial:

[![Tutorial](https://img.youtube.com/vi/KgKWAZ49T9E/0.jpg)](https://www.youtube.com/watch?v=KgKWAZ49T9E)

# Features

- Single and Double float precision build.
- SIMD (Single instruction, multiple data) build.
- Cross-platform determinism build.

# Comparison

Watch a comparison to Godot Physics 2D and [Box2D](https://github.com/appsinacup/godot-box-2d) physics plugin:

[![Comparison](https://img.youtube.com/vi/wgUiZ7E19eM/0.jpg)](https://www.youtube.com/watch?v=wgUiZ7E19eM)

Or read about it on [appsinacup.com/godot-physics-vs-box2d-vs-rapier2d](https://appsinacup.com/godot-physics-vs-box2d-vs-rapier2d/)


# Roadmap

- Fix all other issues from Limitations.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.

# License

All code in this repository is provided under the MIT license. See `LICENSE` for more details and `THIRDPARTY.txt` for third-party licenses.
This repo is a continuation of https://github.com/fabriceci/godot-rapier2d .
