# Changelog

## v0.7.26

- Fix areas removed issue by [@Ughuuu](https://github.com/Ughuuu) in [#187](https://github.com/appsinacup/godot-rapier-physics/pull/187)

## v0.7.25

- Fixed Ghost collision by [@Ughuuu](https://github.com/Ughuuu) in [#184](https://github.com/appsinacup/godot-rapier-physics/pull/184)

## v0.7.24

- Migrate to latest gdext. Also fix body notification issue. by [@Ughuuu](https://github.com/Ughuuu) in [#179](https://github.com/appsinacup/godot-rapier-physics/pull/179)
- fix segment shape by [@Ughuuu](https://github.com/Ughuuu) in [#182](https://github.com/appsinacup/godot-rapier-physics/pull/182)
- Fix static body constant velocity to not cumulate by [@Ughuuu](https://github.com/Ughuuu) in [#183](https://github.com/appsinacup/godot-rapier-physics/pull/183)

## v0.7.23

- Fix area mask/layer issue by [@Ughuuu](https://github.com/Ughuuu) in [#175](https://github.com/appsinacup/godot-rapier-physics/pull/175)
- Fix joints not waking up bodies by [@Ughuuu](https://github.com/Ughuuu) in [#176](https://github.com/appsinacup/godot-rapier-physics/pull/176)

## v0.7.22

- Fix touchscreen issue by [@Ughuuu](https://github.com/Ughuuu) in [#173](https://github.com/appsinacup/godot-rapier-physics/pull/173)


## v0.7.21

- Parry convergence issues custom branch. by [@Ughuuu](https://github.com/Ughuuu) in [#168](https://github.com/appsinacup/godot-rapier-physics/pull/168)
- Fix area event issues by [@Ughuuu](https://github.com/Ughuuu) in [#172](https://github.com/appsinacup/godot-rapier-physics/pull/172)

## v0.7.20

* fix linear/angular damp by [@Ughuuu](https://github.com/Ughuuu) in [#167](https://github.com/appsinacup/godot-rapier-physics/pull/167)
* Update fluid faucet demo. Also add windows arm64 builds by [@Ughuuu](https://github.com/Ughuuu) in [#165](https://github.com/appsinacup/godot-rapier-physics/pull/165)
* Move to impl class fluid so code is shared by [@Ughuuu](https://github.com/Ughuuu) in [#163](https://github.com/appsinacup/godot-rapier-physics/pull/163)

## v0.7.19

* Update to newest gdext. Remove warnings by [@Ughuuu](https://github.com/Ughuuu) in [#161](https://github.com/appsinacup/godot-rapier-physics/pull/161)
* Liquid nice to have features by [@Ughuuu](https://github.com/Ughuuu) in [#160](https://github.com/appsinacup/godot-rapier-physics/pull/160)
* Liquids impl by [@Ughuuu](https://github.com/Ughuuu) in [#159](https://github.com/appsinacup/godot-rapier-physics/pull/159)

## v0.7.18

* trimesh scaling by [@Ughuuu](https://github.com/Ughuuu) in [#157](https://github.com/appsinacup/godot-rapier-physics/pull/157)


## v0.7.17

* fix 3d concave shape to use multiple of 3 faces by [@Ughuuu](https://github.com/Ughuuu) in [#156](https://github.com/appsinacup/godot-rapier-physics/pull/156)

## v0.7.16

* fix concave shapes for 2d. by [@Ughuuu](https://github.com/Ughuuu) in [#155](https://github.com/appsinacup/godot-rapier-physics/pull/155)

## v0.7.15

* Fix bodies filter method by [@Ughuuu](https://github.com/Ughuuu) in [#152](https://github.com/appsinacup/godot-rapier-physics/pull/152)

## v0.7.13

* Add heightmap and concave shape to 3d. by [@Ughuuu](https://github.com/Ughuuu) in [#148](https://github.com/appsinacup/godot-rapier-physics/pull/148)

## v0.7.12

* Fix area bug crash by [@Ughuuu](https://github.com/Ughuuu) in [#146](https://github.com/appsinacup/godot-rapier-physics/pull/146)
* Add pin joint 3d and character body 3d(initial support) by [@Ughuuu](https://github.com/Ughuuu) in [#124](https://github.com/appsinacup/godot-rapier-physics/pull/124)

## v0.7.11

* update docs by [@Ughuuu](https://github.com/Ughuuu) in [#143](https://github.com/appsinacup/godot-rapier-physics/pull/143)
* use latest emscripten 3.1.62 by [@Ughuuu](https://github.com/Ughuuu) in [#145](https://github.com/appsinacup/godot-rapier-physics/pull/145)
* Re-enable android build - helped by bigmac by [@Ughuuu](https://github.com/Ughuuu) in [#144](https://github.com/appsinacup/godot-rapier-physics/pull/144)

## v0.7.10

* Add missing cylinder shape for 3d by [@Ughuuu](https://github.com/Ughuuu) in [#137](https://github.com/appsinacup/godot-rapier-physics/pull/137)
* Update readme and disable parallel builds by [@Ughuuu](https://github.com/Ughuuu) in [#140](https://github.com/appsinacup/godot-rapier-physics/pull/140)

## v0.7.9

* improve performance by storing gd pointer by [@Ughuuu](https://github.com/Ughuuu) in [#136](https://github.com/appsinacup/godot-rapier-physics/pull/136)

## v0.7.8

* fix determinism locally by deleting whole state if physics world is empty by [@Ughuuu](https://github.com/Ughuuu) in [#135](https://github.com/appsinacup/godot-rapier-physics/pull/135)

## v0.7.7

* Fix shapes translated not having correct center of mass and inertia by [@Ughuuu](https://github.com/Ughuuu) in [#133](https://github.com/appsinacup/godot-rapier-physics/pull/133)

## v0.7.6

* Fix shape scale issue and extra contacts reported. Also update rigidbodies to use collision groups by [@Ughuuu](https://github.com/Ughuuu) in [#128](https://github.com/appsinacup/godot-rapier-physics/pull/128)
* Add enchanced determinism flag for math by [@Ughuuu](https://github.com/Ughuuu) in [#132](https://github.com/appsinacup/godot-rapier-physics/pull/132)

## v0.7.5

* fix raycast not sorting results by [@Ughuuu](https://github.com/Ughuuu) in [#125](https://github.com/appsinacup/godot-rapier-physics/pull/125)

## v0.7.4

* Fix order of results from shape result by [@Ughuuu](https://github.com/Ughuuu) in [#120](https://github.com/appsinacup/godot-rapier-physics/pull/120)
* Fix windows build with missing symbols by [@Ughuuu](https://github.com/Ughuuu) in [#121](https://github.com/appsinacup/godot-rapier-physics/pull/121)

## v0.7.3

* Fix ray crash issue by [@Ughuuu](https://github.com/Ughuuu) in [#119](https://github.com/appsinacup/godot-rapier-physics/pull/119)

## v0.7.2

* Fix not destroying objects by [@Ughuuu](https://github.com/Ughuuu) in [#115](https://github.com/appsinacup/godot-rapier-physics/pull/115)
* Fix borrow issues for joints by [@Ughuuu](https://github.com/Ughuuu) in [#116](https://github.com/appsinacup/godot-rapier-physics/pull/116)
* Small update for areas. Also look into reenabling character body. Also small fix for shapes removed. by [@Ughuuu](https://github.com/Ughuuu) in [#118](https://github.com/appsinacup/godot-rapier-physics/pull/118)

## v0.7.1

* add logo for 2d and 3d by [@Ughuuu](https://github.com/Ughuuu) in [#108](https://github.com/appsinacup/godot-rapier-physics/pull/108)
update logos again by [@Ughuuu](https://github.com/Ughuuu) in [#109](https://github.com/appsinacup/godot-rapier-physics/pull/109)
* update data holder to not be global by [@Ughuuu](https://github.com/Ughuuu) in [#110](https://github.com/appsinacup/godot-rapier-physics/pull/110)
* Re enable web build by [@Ughuuu](https://github.com/Ughuuu) in [#114](https://github.com/appsinacup/godot-rapier-physics/pull/114)

## v0.7.0

* NOTE - this release happens after a big release from c++ to rust. Expect it to be in an alpha state.

* Remove error message when body/area already monitored. by [@Ughuuu](https://github.com/Ughuuu) in [#100](https://github.com/appsinacup/godot-rapier-physics/pull/100)
* Migrate to gdext by [@Ughuuu](https://github.com/Ughuuu) in [#84](https://github.com/appsinacup/godot-rapier-physics/pull/84)
