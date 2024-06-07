use crate::{
    rapier_wrapper::{
        handle::{invalid_handle, Handle},
        shape::shape_destroy,
    },
    servers2d::rapier_physics_singleton_2d::bodies_singleton,
};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};
use std::{any::Any, collections::HashMap};

pub trait IRapierShape2D: Any {
    fn get_base(&self) -> &RapierShapeBase2D;
    fn get_mut_base(&mut self) -> &mut RapierShapeBase2D;
    fn get_type(&self) -> ShapeType;
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32;
    fn allows_one_way_collision(&self) -> bool;
    fn create_rapier_shape(&mut self) -> Handle;
    fn set_data(&mut self, data: Variant);
    fn get_data(&self) -> Variant;
    fn get_rapier_shape(&mut self) -> Handle;
}

#[derive(Debug)]
pub struct RapierShapeBase2D {
    rid: Rid,
    aabb: Rect2,
    configured: bool,
    owners: HashMap<Rid, i32>,
    handle: Handle,
}

impl RapierShapeBase2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            aabb: Rect2::default(),
            configured: false,
            owners: HashMap::new(),
            handle: invalid_handle(),
        }
    }
    pub fn set_handle(&mut self, handle: Handle) {
        self.handle = handle;
    }
    pub fn get_handle(&self) -> Handle {
        self.handle
    }
    pub fn configure(&mut self, aabb: Rect2) {
        self.aabb = aabb;
        self.configured = true;
    }
    pub fn call_shape_changed(owners: HashMap<Rid, i32>, shape_rid: Rid) {
        for (owner, _) in owners {
            let owner = bodies_singleton().collision_objects.get_mut(&owner);
            if let Some(owner) = owner {
                owner._shape_changed(shape_rid);
            }
        }
    }

    pub fn get_aabb(&self, origin: Vector2) -> Rect2 {
        let mut aabb_clone = self.aabb;
        aabb_clone.position += origin;
        aabb_clone
    }

    pub fn is_configured(&self) -> bool {
        self.configured
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
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

    pub fn is_owner(&self, owner: Rid) -> bool {
        self.owners.contains_key(&owner)
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn destroy_rapier_shape(&mut self) {
        if self.handle.is_valid() {
            shape_destroy(self.handle);
            self.handle = invalid_handle();
        }
    }
}

impl Drop for RapierShapeBase2D {
    fn drop(&mut self) {
        if !self.owners.is_empty() {
            godot_error!("RapierShapeBase2D leaked {} owners", self.owners.len());
        }
        if self.handle.is_valid() {
            self.destroy_rapier_shape();
        }
    }
}
