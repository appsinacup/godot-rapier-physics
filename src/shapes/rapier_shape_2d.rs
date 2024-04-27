use std::collections::HashMap;

use godot::{prelude::*};
use crate::rapier2d::*;
use crate::rapier2d::handle::*;

use crate::rapier2d::shape::*;

pub trait RapierShapeOwner2D {
    fn shape_changed(&self, p_shape: &RapierShape2D);
    fn remove_shape(&self, p_shape: &RapierShape2D);
}

pub trait IRapierShape2D {
    fn create_rapier_shape(&mut self) -> Handle;
}

pub struct RapierShape2D {
    rid: Rid,
    aabb: Rect2,
    configured: bool,
    owners: HashMap<&'static dyn RapierShapeOwner2D, i32>,
    shape_handle: Handle,
    shape_interface: dyn IRapierShape2D,
}

impl RapierShape2D {
    pub fn configure(&mut self, p_aabb: Rect2) {
        self.aabb = p_aabb;
        self.configured = true;
        for (owner, _) in self.owners.iter() {
            owner.shape_changed(self);
        }
    }

    pub fn destroy_rapier_shape(&mut self) {
        if is_handle_valid(self.shape_handle) {
            shape_destroy(self.shape_handle);
            self.shape_handle = invalid_handle();
        }
    }

    pub fn get_rapier_shape(&mut self) -> Handle {
        if !is_handle_valid(self.shape_handle) {
            self.shape_handle = self.shape_interface.create_rapier_shape();
        }
        self.shape_handle
    }

    pub fn add_owner(&mut self, p_owner: &'static dyn RapierShapeOwner2D) {
        if self.owners.contains_key(p_owner) {
            
        }
        *self.owners.entry(p_owner).or_insert(0) += 1;
    }

    pub fn remove_owner(&mut self, p_owner: &'static dyn RapierShapeOwner2D) {
        if let Some(count) = self.owners.get_mut(p_owner) {
            *count -= 1;
            if *count == 0 {
                self.owners.remove(p_owner);
            }
        }
    }

    pub fn is_owner(&self, p_owner: &'static dyn RapierShapeOwner2D) -> bool {
        self.owners.contains_key(p_owner)
    }

    pub fn get_owners(&self) -> &HashMap<&'static dyn RapierShapeOwner2D, i32> {
        &self.owners
    }

    pub fn set_data(&mut self, _p_data: Variant) {
    }

    pub fn get_data(&self) -> Variant {
        Variant::nil()
    }

    pub fn get_moment_of_inertia(&self, _p_mass: f32, _p_scale: Vector2) -> f32 {
        0.0
    }
}

impl Drop for RapierShape2D {
    fn drop(&mut self) {
        assert_eq!(self.owners.len(), 0);
        self.destroy_rapier_shape();
    }
}
