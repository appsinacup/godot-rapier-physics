use godot::prelude::*;
use hashbrown::HashMap;
use rapier::prelude::SharedShape;

use crate::bodies::exportable_object::ExportableObject;
use crate::bodies::exportable_object::ImportToExport;
use crate::bodies::exportable_object::ObjectImportState;
use crate::bodies::exportable_object::ExportToImport;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsData;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::servers::rapier_physics_singleton::get_id_rid;
use crate::types::*;
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
#[derive(Debug)]
pub struct ShapeExport<'a> {
    state: &'a RapierShapeState,
    shape: &'a SharedShape,
}
impl<'a> ExportToImport for ShapeExport<'a> {
    type Import = ShapeImport;
    fn into_import(self) -> Self::Import {
        ShapeImport {
            state: self.state.clone(),
            shape: self.shape.clone(),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize, Clone))]
pub struct ShapeImport {
    state: RapierShapeState,
    shape: SharedShape,
}
impl ImportToExport for ShapeImport {
    type Export<'a> = ShapeExport<'a>;

    fn from_import<'a>(&'a self) -> Self::Export<'a> {
        ShapeExport { 
            state: &self.state,
            shape: &self.shape,
        }
    }    
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
#[derive(Debug)]
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
}
impl Drop for RapierShapeBase {
    fn drop(&mut self) {
        if !self.state.owners.is_empty() {
            godot_error!("RapierShapeBase leaked {} owners", self.state.owners.len());
        }
    }
}
#[cfg(feature = "serde-serialize")]
impl ExportableObject for RapierShapeBase {
    type ExportState<'a> = ShapeExport<'a>;

    fn get_export_state<'a>(
        &'a self,
        physics_engine: &'a mut PhysicsEngine,
    ) -> Option<Self::ExportState<'a>> {
        physics_engine
            .get_shape(self.get_id())
            .map(|inner| ShapeExport {
                state: &self.state,
                shape: inner,
            })
    }

    fn import_state(&mut self, physics_engine: &mut PhysicsEngine, data: ObjectImportState) {
        match data {
            crate::bodies::exportable_object::ObjectImportState::ShapeBase(shape_import) => {
                self.state = shape_import.state;
                physics_engine.insert_shape(shape_import.shape, self.get_id());
            }
            _ => {
                godot_error!("Attempted to import invalid state data.");
            }
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
