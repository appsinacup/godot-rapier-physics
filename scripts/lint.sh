cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty --all-targets --features="build2d" --no-default-features
cargo clippy --fix --allow-dirty --all-targets --features="build3d" --no-default-features
cargo clippy --fix --allow-dirty
gdlint bin2d/addons/godot-rapier2d/**.gd
gdlint bin3d/addons/godot-rapier3d/**.gd
