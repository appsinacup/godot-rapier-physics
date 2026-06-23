cargo fmt -- --config-path rustfmt.toml
cargo clippy --fix --allow-dirty --all-targets --features="single-dim2,serde-serialize,simd-stable,experimental-threads,register-docs,api-4-7"
cargo clippy --fix --allow-dirty --all-targets --features="single-dim3,serde-serialize,simd-stable,experimental-threads,register-docs,api-4-7"
gdlint bin2d/addons/godot-rapier2d/**.gd
gdlint bin3d/addons/godot-rapier3d/**.gd
