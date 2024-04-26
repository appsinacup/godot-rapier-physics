use godot::prelude::*;

struct RapierPhysicsServer;

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysicsServer {}
