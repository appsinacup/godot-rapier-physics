use godot::builtin::Rid;
use hashbrown::HashMap;
use hashbrown::HashSet;
use rapier::prelude::ColliderHandle;
use rapier::prelude::RigidBodyHandle;

use crate::bodies::rapier_collision_object_base::CollisionObjectType;
use crate::types::*;
impl RemovedColliderInfo {
    pub fn new(
        rid: Rid,
        rb_handle: RigidBodyHandle,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) -> Self {
        Self {
            rid,
            rb_handle,
            instance_id,
            shape_index,
            collision_object_type,
        }
    }
}
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RemovedColliderInfo {
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "default_rid"))]
    pub rid: Rid,
    pub rb_handle: RigidBodyHandle,
    pub instance_id: u64,
    pub shape_index: usize,
    pub collision_object_type: CollisionObjectType,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(Debug, PartialEq, Clone, Default)]
pub struct RapierSpaceState {
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) removed_colliders: HashMap<ColliderHandle, RemovedColliderInfo>,
    pub(crate) active_list: HashSet<RigidBodyHandle>,
    pub(crate) mass_properties_update_list: HashSet<RigidBodyHandle>,
    pub(crate) gravity_update_list: HashSet<RigidBodyHandle>,
    pub(crate) state_query_list: HashSet<RigidBodyHandle>,
    pub(crate) force_integrate_query_list: HashSet<RigidBodyHandle>,
    pub(crate) monitor_query_list: HashSet<RigidBodyHandle>,
    pub(crate) area_update_list: HashSet<RigidBodyHandle>,
    pub(crate) body_area_update_list: HashSet<RigidBodyHandle>,
    pub(crate) time_stepped: f32,
    pub(crate) island_count: i32,
    pub(crate) active_objects: i32,
    pub(crate) collision_pairs: i32,
}
