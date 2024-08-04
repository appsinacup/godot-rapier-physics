#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::HashMap;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsData;
use crate::types::*;
#[cfg_attr(feature = "serde-serialize", typetag::serde(tag = "type"))]
pub trait IRapierShape {
    fn get_base(&self) -> &RapierShapeBase;
    fn get_mut_base(&mut self) -> &mut RapierShapeBase;
    fn get_type(&self) -> ShapeType;
    fn allows_one_way_collision(&self) -> bool;
    fn create_rapier_shape(&mut self, physics_engine: &mut PhysicsEngine) -> ShapeHandle;
    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine);
    fn get_data(&self) -> Variant;
    fn get_handle(&self) -> ShapeHandle;
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierShapeBase {
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "invalid_rid"))]
    rid: Rid,
    aabb: Rect,
    // TODO serialize this
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    owners: HashMap<Rid, i32>,
    handle: ShapeHandle,
}
impl Default for RapierShapeBase {
    fn default() -> Self {
        Self {
            rid: Rid::Invalid,
            aabb: Rect::default(),
            owners: HashMap::default(),
            handle: ShapeHandle::default(),
        }
    }
}
impl RapierShapeBase {
    pub(super) fn new(rid: Rid) -> Self {
        Self {
            rid,
            aabb: Rect::default(),
            owners: HashMap::default(),
            handle: ShapeHandle::default(),
        }
    }

    pub(super) fn set_handle(&mut self, handle: ShapeHandle, physics_engine: &mut PhysicsEngine) {
        if self.handle != ShapeHandle::default() {
            self.destroy_shape(physics_engine);
        }
        let rapier_aabb = physics_engine.shape_get_aabb(handle);
        let vertices = rapier_aabb.vertices();
        self.aabb = Rect::new(
            vector_to_godot(vertices[0].coords),
            vector_to_godot(rapier_aabb.extents()),
        );
        self.handle = handle;
    }

    pub fn get_handle(&self) -> ShapeHandle {
        self.handle
    }

    pub fn is_valid(&self) -> bool {
        self.handle != ShapeHandle::default()
    }

    pub fn call_shape_changed(
        owners: HashMap<Rid, i32>,
        shape_rid: Rid,
        physics_data: &mut PhysicsData,
    ) {
        for (owner, _) in owners {
            if let Some(owner) = physics_data.collision_objects.get_mut(&owner) {
                owner.shape_changed(
                    shape_rid,
                    &mut physics_data.physics_engine,
                    &mut physics_data.shapes,
                    &mut physics_data.spaces,
                );
            }
        }
    }

    pub fn get_aabb(&self, origin: Vector) -> Rect {
        let mut aabb_clone = self.aabb;
        aabb_clone.position += origin;
        aabb_clone
    }

    #[cfg(feature = "dim2")]
    pub fn get_aabb_area(&self) -> real {
        self.aabb.area()
    }

    #[cfg(feature = "dim3")]
    pub fn get_aabb_area(&self) -> real {
        self.aabb.volume()
    }

    pub fn add_owner(&mut self, owner: Rid) {
        *self.owners.entry(owner).or_insert(0) += 1;
    }

    pub fn remove_owner(&mut self, owner: Rid) {
        if let Some(count) = self.owners.get_mut(&owner) {
            *count -= 1;
            if *count == 0 {
                self.owners.remove(&owner);
            }
        }
    }

    pub fn get_owners(&self) -> &HashMap<Rid, i32> {
        &self.owners
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn destroy_shape(&mut self, physics_engine: &mut PhysicsEngine) {
        if self.handle != ShapeHandle::default() {
            physics_engine.shape_destroy(self.handle);
            self.handle = ShapeHandle::default();
        }
    }
}
impl Drop for RapierShapeBase {
    fn drop(&mut self) {
        if !self.owners.is_empty() {
            godot_error!("RapierShapeBase leaked {} owners", self.owners.len());
        }
        if self.is_valid() {
            godot_error!("RapierShapeBase leaked");
        }
    }
}
