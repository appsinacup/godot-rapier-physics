use godot::prelude::*;
use hashbrown::HashMap;
use rapier::prelude::SharedShape;

use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::get_id_rid;
use crate::servers::rapier_physics_singleton::PhysicsData;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::*;
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
pub struct ShapeExport<'a> {
    state: &'a RapierShapeState,
    shape: &'a SharedShape,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize))]
pub struct ShapeImport {
    state: RapierShapeState,
    shape: SharedShape,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Clone, Debug, Default)]
pub struct RapierShapeState {
    aabb: Rect,
    #[cfg_attr(
        feature = "serde-serialize",
        serde(
            serialize_with = "rapier::utils::serde::serialize_to_vec_tuple",
            deserialize_with = "rapier::utils::serde::deserialize_from_vec_tuple"
        )
    )]
    owners: HashMap<RapierId, i32>,
    id: RapierId,
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
    pub(super) fn new(id: RapierId, rid: Rid) -> Self {
        Self {
            rid,
            state: RapierShapeState {
                id,
                ..Default::default()
            },
        }
    }

    pub(super) fn reset_aabb(&mut self, physics_engine: &mut PhysicsEngine) {
        let rapier_aabb = physics_engine.shape_get_aabb(self.get_id());
        let vertices = rapier_aabb.vertices();
        self.state.aabb = Rect::new(
            vector_to_godot(vertices[0].coords),
            vector_to_godot(rapier_aabb.extents()),
        );
    }

    pub fn call_shape_changed(
        owners: HashMap<RapierId, i32>,
        shape_id: RapierId,
        physics_data: &mut PhysicsData,
    ) {
        for (owner, _) in owners {
            if let Some(owner) = physics_data
                .collision_objects
                .get_mut(&get_id_rid(owner, &physics_data.ids))
            {
                owner.shape_changed(
                    shape_id,
                    &mut physics_data.physics_engine,
                    &mut physics_data.spaces,
                    &physics_data.ids,
                );
            }
        }
    }

    pub fn get_aabb(&self, origin: Vector) -> Rect {
        let mut aabb_clone = self.state.aabb;
        aabb_clone.position += origin;
        aabb_clone
    }

    pub fn add_owner(&mut self, owner: RapierId) {
        *self.state.owners.entry(owner).or_insert(0) += 1;
    }

    pub fn remove_owner(&mut self, owner: RapierId) {
        if let Some(count) = self.state.owners.get_mut(&owner) {
            *count -= 1;
            if *count == 0 {
                self.state.owners.remove(&owner);
            }
        }
    }

    pub fn get_owners(&self) -> &HashMap<RapierId, i32> {
        &self.state.owners
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn get_id(&self) -> RapierId {
        self.state.id
    }

    pub fn destroy_shape(&mut self, physics_engine: &mut PhysicsEngine) {
        physics_engine.shape_destroy(self.get_id());
        self.state.owners.clear();
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
    pub fn export_binary(&self, physics_engine: &mut PhysicsEngine) -> PackedByteArray {
        let mut buf = PackedByteArray::new();
        if let Some(inner) = physics_engine.get_shape(self.get_id()) {
            let export = ShapeExport {
                state: &self.state,
                shape: inner,
            };
            match bincode::serialize(&export) {
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
        }
        buf
    }

    #[cfg(feature = "serde-serialize")]
    pub fn import_binary(&mut self, data: PackedByteArray, physics_engine: &mut PhysicsEngine) {
        match bincode::deserialize::<ShapeImport>(data.as_slice()) {
            Ok(import) => {
                self.state = import.state;
                physics_engine.insert_shape(import.shape, self.get_id());
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
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_new_rapier_shape_base() {
        let rid = Rid::new(123);
        let shape_base = RapierShapeBase::new(0, rid);
        assert_eq!(shape_base.get_rid(), rid);
        assert!(shape_base.get_owners().is_empty());
    }
    #[test]
    fn test_reset_aabb() {
        let rid = Rid::new(123);
        let mut shape_base = RapierShapeBase::new(0, rid);
        let mut physics_engine = PhysicsEngine::default();
        shape_base.reset_aabb(&mut physics_engine);
    }
    #[test]
    fn test_add_and_remove_owner() {
        let rid = Rid::new(123);
        let mut shape_base = RapierShapeBase::new(0, rid);
        let rb_id = 1;
        shape_base.add_owner(rb_id);
        assert_eq!(shape_base.get_owners().get(&rb_id), Some(&1));
        shape_base.add_owner(rb_id);
        assert_eq!(shape_base.get_owners().get(&rb_id), Some(&2));
        shape_base.remove_owner(rb_id);
        assert_eq!(shape_base.get_owners().get(&rb_id), Some(&1));
        shape_base.remove_owner(rb_id);
        assert!(shape_base.get_owners().get(&rb_id).is_none());
    }
    #[test]
    fn test_destroy_shape() {
        let rid = Rid::new(123);
        let mut shape_base = RapierShapeBase::new(0, rid);
        let mut physics_engine = PhysicsEngine::default();
        shape_base.reset_aabb(&mut physics_engine);
        shape_base.destroy_shape(&mut physics_engine);
    }
}
