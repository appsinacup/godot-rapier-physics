use godot::{engine::physics_server_2d::ShapeType, prelude::*};
use std::{borrow::Borrow, cell::RefCell, collections::HashMap, rc::Rc};

use crate::rapier2d::handle::Handle;

pub trait IRapierShapeOwner2D {
    fn shape_changed(&self, shape: Rid);
    fn remove_shape(&self, shape: Rid);
}

pub trait IRapierShape2D {
    fn create_rapier_shape(&mut self) -> Handle;
    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32;
    fn set_data(&mut self, data: Variant);
    fn get_data(&self) -> Variant;
    fn get_rapier_shape(&mut self) -> Handle;
    fn get_type(&self) -> ShapeType;
    fn destroy_rapier_shape(&mut self);
}

pub struct RapierShapeBase2D {
    rid: Rid,
    aabb: Rect2,
    configured: bool,
    owners_count: HashMap<Rid, i32>,
    pub owners: HashMap<Rid, Rc<RefCell<dyn IRapierShapeOwner2D>>>,
}

impl RapierShapeBase2D {
    pub fn configure(&mut self, aabb: Rect2) {
        self.aabb = aabb;
        self.configured = true;
        for (_, owner) in self.owners.iter() {
            let owner: &RefCell<(dyn IRapierShapeOwner2D + 'static)> = owner.borrow();
            owner.borrow().shape_changed(self.rid);
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
        *self.owners_count.entry(owner).or_insert(0) += 1;
    }

    pub fn remove_owner(&mut self, owner: Rid) {
        if let Some(count) = self.owners_count.get_mut(&owner) {
            *count -= 1;
            if *count == 0 {
                self.owners_count.remove(&owner);
            }
        }
    }

    pub fn is_owner(&self, owner: Rid) -> bool {
        self.owners.contains_key(&owner)
    }
}


impl Drop for RapierShapeBase2D {
    fn drop(&mut self) {
        if self.owners.len() > 0 {
            godot_error!("RapierShapeBase2D leaked {} owners", self.owners.len());
        }
    }
}