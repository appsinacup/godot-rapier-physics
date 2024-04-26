use godot::prelude::*;
use godot::engine::{IPhysicsServer2DExtension, PhysicsServer2DExtension};

#[derive(GodotClass)]
#[class(base=PhysicsServer2DExtension)]
pub struct RapierPhysicsServer2D {
    base: Base<PhysicsServer2DExtension>
}


#[godot_api]
impl IPhysicsServer2DExtension for RapierPhysicsServer2D {

    fn init(base: Base<PhysicsServer2DExtension>) -> Self {
        godot_print!("IPhysicsServer2DExtension"); // Prints to the Godot console
        
        Self {
            base
        }
    }
}