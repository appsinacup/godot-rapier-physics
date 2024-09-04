use godot::prelude::*;
use hashbrown::HashMap;
use rapier::prelude::RigidBodyHandle;

use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::get_body_rid;
use crate::servers::rapier_physics_singleton::insert_shape_rid;
use crate::servers::rapier_physics_singleton::remove_shape_rid;
use crate::servers::rapier_physics_singleton::PhysicsData;
use crate::servers::rapier_physics_singleton::PhysicsRids;
use crate::types::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Clone, Debug, Default)]
pub struct RapierShapeState {
    aabb: Rect,
    owners: HashMap<RigidBodyHandle, i32>,
    handle: ShapeHandle,
}
pub struct RapierShapeBase {
    rid: Rid,
    state: RapierShapeState,
}
impl Default for RapierShapeBase {
    fn default() -> Self {
        Self {
            rid: Rid::Invalid,
            state: RapierShapeState::default(),
        }
    }
}
impl RapierShapeBase {
    pub(super) fn new(rid: Rid) -> Self {
        Self {
            rid,
            state: RapierShapeState::default(),
        }
    }

    pub(super) fn set_handle_and_reset_aabb(
        &mut self,
        handle: ShapeHandle,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        if self.state.handle != ShapeHandle::default() {
            self.destroy_shape(physics_engine, physics_rids);
        }
        let rapier_aabb = physics_engine.shape_get_aabb(handle);
        let vertices = rapier_aabb.vertices();
        self.state.aabb = Rect::new(
            vector_to_godot(vertices[0].coords),
            vector_to_godot(rapier_aabb.extents()),
        );
        self.state.handle = handle;
        insert_shape_rid(handle, self.rid, physics_rids);
    }

    pub fn get_handle(&self) -> ShapeHandle {
        self.state.handle
    }

    pub fn is_valid(&self) -> bool {
        self.state.handle != ShapeHandle::default()
    }

    pub fn call_shape_changed(
        owners: HashMap<RigidBodyHandle, i32>,
        shape_rid: Rid,
        physics_data: &mut PhysicsData,
    ) {
        for (owner, _) in owners {
            if let Some(owner) = physics_data
                .collision_objects
                .get_mut(&get_body_rid(owner, &physics_data.rids))
            {
                owner.shape_changed(
                    shape_rid,
                    &mut physics_data.physics_engine,
                    &mut physics_data.shapes,
                    &mut physics_data.spaces,
                    &physics_data.rids,
                );
            }
        }
    }

    pub fn get_aabb(&self, origin: Vector) -> Rect {
        let mut aabb_clone = self.state.aabb;
        aabb_clone.position += origin;
        aabb_clone
    }

    pub fn add_owner(&mut self, owner: RigidBodyHandle) {
        *self.state.owners.entry(owner).or_insert(0) += 1;
    }

    pub fn remove_owner(&mut self, owner: RigidBodyHandle) {
        if let Some(count) = self.state.owners.get_mut(&owner) {
            *count -= 1;
            if *count == 0 {
                self.state.owners.remove(&owner);
            }
        }
    }

    pub fn get_owners(&self) -> &HashMap<RigidBodyHandle, i32> {
        &self.state.owners
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn destroy_shape(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        if self.state.handle != ShapeHandle::default() {
            remove_shape_rid(self.state.handle, physics_rids);
            physics_engine.shape_destroy(self.state.handle);
            self.state.handle = ShapeHandle::default();
        }
    }

    #[cfg(feature = "serde-serialize")]
    pub fn export_json(&self) -> String {
        match serde_json::to_string_pretty(&self.state) {
            Ok(s) => return s,
            Err(e) => {
                godot_error!("Failed to serialize shape to json: {}", e);
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    pub fn export_binary(&self) -> PackedByteArray {
        let mut buf = PackedByteArray::new();
        match bincode::serialize(&self.state) {
            Ok(binary_data) => {
                buf.resize(binary_data.len());
                for i in 0..binary_data.len() {
                    buf[i] = binary_data[i];
                }
            }
            Err(e) => {
                godot_error!("Failed to serialize shape to binary: {}", e);
            }
        }
        buf
    }

    #[cfg(feature = "serde-serialize")]
    pub fn import_binary(&mut self, data: PackedByteArray) {
        match bincode::deserialize::<RapierShapeState>(data.as_slice()) {
            Ok(import) => {
                self.state = import;
            }
            Err(e) => {
                godot_error!("Failed to deserialize shape from binary: {}", e);
            }
        }
    }
}
impl Drop for RapierShapeBase {
    fn drop(&mut self) {
        if !self.state.owners.is_empty() {
            godot_error!("RapierShapeBase leaked {} owners", self.state.owners.len());
        }
        if self.is_valid() {
            godot_error!("RapierShapeBase leaked");
        }
    }
}
