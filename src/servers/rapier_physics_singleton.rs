use godot::prelude::*;
use hashbrown::HashMap;
use rapier::data::Index;
use rapier::prelude::RigidBodyHandle;

use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::fluids::rapier_fluid::RapierFluid;
use crate::joints::rapier_joint::RapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::RapierShape;
use crate::spaces::rapier_space::RapierSpace;
pub type PhysicsShapes = HashMap<Rid, RapierShape>;
pub type PhysicsSpaces = HashMap<Rid, RapierSpace>;
pub type PhysicsActiveSpaces = HashMap<WorldHandle, Rid>;
pub type PhysicsRids = HashMap<Index, Rid>;
pub type PhysicsCollisionObjects = HashMap<Rid, RapierCollisionObject>;
pub type PhysicsJoints = HashMap<Rid, RapierJoint>;
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
    pub rids: PhysicsRids,
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
                rids: HashMap::default(),
            });
        }
        SINGLETON.as_mut().unwrap()
    }
}
pub fn get_rid(handle: Index) -> &'static Rid {
    let physics_data = physics_data();
    return physics_data.rids.get(&handle).unwrap_or(&Rid::Invalid);
}