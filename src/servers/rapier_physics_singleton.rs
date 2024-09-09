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
#[derive(PartialEq, Clone, Debug, Eq, Hash)]
pub enum PhysicsType {
    World,
    RigidBody,
}
pub type PhysicsRids = HashMap<(PhysicsType, Index), Rid>;
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
pub fn get_body_rid(handle: RigidBodyHandle, physics_rids: &PhysicsRids) -> Rid {
    return *physics_rids
        .get(&(PhysicsType::RigidBody, handle.0))
        .unwrap_or(&Rid::Invalid);
}
pub fn get_space_rid(handle: WorldHandle, physics_rids: &PhysicsRids) -> Rid {
    return *physics_rids
        .get(&(PhysicsType::World, handle))
        .unwrap_or(&Rid::Invalid);
}
pub fn insert_body_rid(handle: RigidBodyHandle, rid: Rid, physics_rids: &mut PhysicsRids) {
    physics_rids.insert((PhysicsType::RigidBody, handle.0), rid);
}
pub fn insert_space_rid(handle: WorldHandle, rid: Rid, physics_rids: &mut PhysicsRids) {
    physics_rids.insert((PhysicsType::World, handle), rid);
}
pub fn remove_body_rid(handle: RigidBodyHandle, physics_rids: &mut PhysicsRids) {
    physics_rids.remove(&(PhysicsType::RigidBody, handle.0));
}
pub fn remove_space_rid(handle: WorldHandle, physics_rids: &mut PhysicsRids) {
    physics_rids.remove(&(PhysicsType::World, handle));
}
