use crate::{
    rapier2d::handle::Handle,
    servers::rapier_physics_singleton_2d::physics_collision_objects_singleton_mutex_guard,
};
use godot::{engine::physics_server_2d::ShapeType, prelude::*};
use std::collections::HashMap;

pub trait IRapierShape2D {
    fn get_type(&self) -> ShapeType;
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32;
    fn allows_one_way_collision(&self) -> bool;
    fn create_rapier_shape(&mut self) -> Handle;
    fn set_data(&mut self, data: Variant);
    fn get_data(&self) -> Variant;
    fn get_rapier_shape(&mut self) -> Handle;
    fn destroy_rapier_shape(&mut self);
}

pub struct RapierShapeBase2D {
    rid: Rid,
    aabb: Rect2,
    configured: bool,
    owners: HashMap<Rid, i32>,
}

impl RapierShapeBase2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            aabb: Rect2::default(),
            configured: false,
            owners: HashMap::new(),
        }
    }
    pub fn configure(&mut self, aabb: Rect2) {
        self.aabb = aabb;
        self.configured = true;
        for (owner, _) in self.owners.iter() {
            let guard = physics_collision_objects_singleton_mutex_guard();
            let owner = guard.get(owner);
            if let Some(owner) = owner {
                owner.shape_changed(self.rid);
            }
        }
    }

    pub fn get_aabb(&self, origin: Vector2) -> Rect2 {
        let mut aabb_clone = self.aabb.clone();
        aabb_clone.position += origin;
        aabb_clone
    }

    pub fn is_configured(&self) -> bool {
        self.configured
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

    pub fn is_owner(&self, owner: Rid) -> bool {
        self.owners.contains_key(&owner)
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }
}

impl Drop for RapierShapeBase2D {
    fn drop(&mut self) {
        if self.owners.len() > 0 {
            godot_error!("RapierShapeBase2D leaked {} owners", self.owners.len());
        }
    }
}
