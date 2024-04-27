use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use godot::{prelude::*};
use crate::rapier2d::*;
use crate::rapier2d::handle::*;

use crate::rapier2d::shape::*;

pub trait IRapierShape2D {
    fn create_rapier_shape(&mut self) -> Handle;
    fn get_moment_of_inertia(&self, _p_mass: f32, _p_scale: Vector2) -> f32;
    fn set_data(&mut self, _p_data: Variant);
    fn get_data(&self) -> Variant;
}

pub struct RapierShape2D {
    rid: Rid,
    aabb: Rect2,
    configured: bool,
    // RapierShapeOwner2D
    owners: HashMap<Rid, i32>,
    shape_handle: Handle,
    shape_interface: dyn IRapierShape2D,
}

impl RapierShape2D {
    pub fn configure(&mut self, p_aabb: Rect2) {
        self.aabb = p_aabb;
        self.configured = true;
        for (owner, _) in self.owners.iter() {
            //owner.shape_changed(self.rid);
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

    pub fn add_owner(&mut self, p_owner: Rid) {
        *self.owners.entry(p_owner).or_insert(0) += 1;
    }

    pub fn remove_owner(&mut self, p_owner: Rid) {
        if let Some(count) = self.owners.get_mut(&p_owner) {
            *count -= 1;
            if *count == 0 {
                self.owners.remove(&p_owner);
            }
        }
    }

    pub fn is_owner(&self, p_owner: Rid) -> bool {
        self.owners.contains_key(&p_owner)
    }

    pub fn get_owners(&self) -> HashMap<Rid, i32> {
        self.owners.clone()
    }
}

impl Drop for RapierShape2D {
    fn drop(&mut self) {
        assert_eq!(self.owners.len(), 0);
        self.destroy_rapier_shape();
    }
}
