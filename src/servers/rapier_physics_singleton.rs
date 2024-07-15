use godot::prelude::*;
use hashbrown::HashMap;

use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::fluids::rapier_fluid::RapierFluid;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::spaces::rapier_space::RapierSpace;
pub type PhysicsShapes = HashMap<Rid, Box<dyn IRapierShape>>;
pub type PhysicsSpaces = HashMap<Rid, RapierSpace>;
pub type PhysicsActiveSpaces = HashMap<WorldHandle, Rid>;
pub type PhysicsCollisionObjects = HashMap<Rid, Box<dyn IRapierCollisionObject>>;
pub type PhysicsJoints = HashMap<Rid, Box<dyn IRapierJoint>>;
pub type PhysicsFluids = HashMap<Rid, RapierFluid>;
#[derive(Default)]
pub struct PhysicsData {
    pub shapes: PhysicsShapes,
    pub spaces: PhysicsSpaces,
    pub active_spaces: PhysicsActiveSpaces,
    pub collision_objects: PhysicsCollisionObjects,
    pub joints: PhysicsJoints,
    pub fluids: PhysicsFluids,
    pub physics_engine: PhysicsEngine,
}
pub fn physics_data() -> &'static mut PhysicsData {
    static mut SINGLETON: Option<PhysicsData> = None;
    unsafe {
        if SINGLETON.is_none() {
            SINGLETON = Some(PhysicsData {
                shapes: HashMap::default(),
                spaces: HashMap::default(),
                active_spaces: HashMap::default(),
                collision_objects: HashMap::default(),
                joints: HashMap::default(),
                fluids: HashMap::default(),
                physics_engine: PhysicsEngine::default(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
