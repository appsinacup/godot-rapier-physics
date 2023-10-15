<div align="center">
  <h1>Godot Rapier2D</h1>
</div>

<p align="center">
	<a href="https://github.com/fabriceci/godot-rapier2d/actions/workflows/runner.yml">
        <img src="https://github.com/fabriceci/godot-rapier2d/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="Godot Rapier2D Build"></a>
    <a href="https://github.com/dimforge/rapier/releases/tag/v0.17.2" alt="Rapier2D Version">
        <img src="https://img.shields.io/badge/Rapier2D-v0.17.2-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/godotengine/godot-cpp" alt="Godot Version">
        <img src="https://img.shields.io/badge/Godot-v4.1-%23478cbf?logo=godot-engine&logoColor=white" /></a>
    <a href="https://github.com/fabriceci/godot-rapier2d/graphs/contributors" alt="Contributors">
        <img src="https://img.shields.io/github/contributors/fabriceci/godot-rapier2d" /></a>
    <a href="https://github.com/fabriceci/godot-rapier2d/pulse" alt="Activity">
        <img src="https://img.shields.io/github/commit-activity/m/fabriceci/godot-rapier2d" /></a>
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>

<img src="https://github.com/fabriceci/godot-rapier2d/blob/main/logo.jpg?raw=true"/> 

A 2d [rapier](https://github.com/dimforge/rapier) physics server for [Godot Engine](https://github.com/godotengine/godot), implemented as a GDExtension.

# Limitations

- One way direction (WIP)
- Spring Joint (WIP)
- Static Body Constant Speed or Conveyer Belt (WIP)
- Small issues with Character Controller (WIP)

# Installation

- Automatic (WIP)

- Manual: 

  a. Download the github release and move only the `addons` folder into your project `addons` folder. After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Rapier2D`.

  b. Build it yourself. Read more about it in the [documentation](DOCUMENTATION.MD).

## Use the Rapier 2D extension

Copy the `addons/` folder to your project root.

# Roadmap

- Cross Platform Determinism
- Add more types of joints
- Pass all Godot Physics Tests.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.

# License

All code in this repository is provided under the MIT license. See `LICENSE` for more details and `THIRDPARTY.txt` for third-party licenses.
