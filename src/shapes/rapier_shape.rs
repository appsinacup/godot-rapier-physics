use std::any::Any;

#[cfg(feature = "dim2")]
use godot::engine::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::engine::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::HashMap;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::bodies_singleton;
use crate::Angle;
use crate::Rect;
use crate::Vector;
pub trait IRapierShape: Any {
    fn get_base(&self) -> &RapierShapeBase;
    fn get_mut_base(&mut self) -> &mut RapierShapeBase;
    fn get_type(&self) -> ShapeType;
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> Angle;
    fn allows_one_way_collision(&self) -> bool;
    fn create_rapier_shape(&mut self) -> Handle;
    fn set_data(&mut self, data: Variant);
    fn get_data(&self) -> Variant;
    fn get_handle(&mut self) -> Handle;
}
#[derive(Serialize, Deserialize, Debug)]
pub struct RapierShapeBase {
    rid: Rid,
    aabb: Rect,
    owners: HashMap<Rid, i32>,
    handle: Handle,
}
impl RapierShapeBase {
    pub(super) fn new(rid: Rid) -> Self {
        Self {
            rid,
            aabb: Rect::default(),
            owners: HashMap::default(),
            handle: invalid_handle(),
        }
    }

    pub(super) fn set_handle(&mut self, handle: Handle, aabb: Rect) {
        if self.handle.is_valid() {
            self.destroy_rapier_shape();
        }
        self.aabb = aabb;
        self.handle = handle;
    }

    pub fn get_handle(&self) -> Handle {
        self.handle
    }

    pub fn call_shape_changed(owners: HashMap<Rid, i32>, shape_rid: Rid) {
        for (owner, _) in owners {
            let owner = bodies_singleton().collision_objects.get_mut(&owner);
            if let Some(owner) = owner {
                owner._shape_changed(shape_rid);
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

    fn destroy_rapier_shape(&mut self) {
        if self.handle.is_valid() {
            shape_destroy(self.handle);
            self.handle = invalid_handle();
        }
    }
}
impl Drop for RapierShapeBase {
    fn drop(&mut self) {
        if !self.owners.is_empty() {
            godot_error!("RapierShapeBase leaked {} owners", self.owners.len());
        }
        if self.handle.is_valid() {
            self.destroy_rapier_shape();
        }
    }
}
