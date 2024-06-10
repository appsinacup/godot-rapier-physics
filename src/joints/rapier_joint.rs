use std::any::Any;

use crate::rapier_wrapper::{
    handle::{invalid_handle, Handle},
    joint::{joint_change_disable_collision, joint_destroy},
};
use godot::{builtin::Rid, engine::physics_server_2d};

use super::{
    rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D, rapier_pin_joint_2d::RapierPinJoint2D,
};

pub trait IRapierJoint: Any {
    fn get_base(&self) -> &RapierJointBase;
    fn get_mut_base(&mut self) -> &mut RapierJointBase;
    fn get_type(&self) -> physics_server_2d::JointType;
    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D>;
    fn get_pin(&self) -> Option<&RapierPinJoint2D>;
    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D>;
    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D>;
}

pub struct RapierJointBase {
    max_force: f32,
    rid: Rid,
    handle: Handle,
    space_handle: Handle,
    disabled_collisions_between_bodies: bool,
}

impl RapierJointBase {
    pub fn new(space_handle: Handle, handle: Handle, rid: Rid) -> Self {
        Self {
            max_force: f32::MAX,
            rid,
            handle,
            space_handle,
            disabled_collisions_between_bodies: true,
        }
    }
    pub fn get_handle(&self) -> Handle {
        self.handle
    }
    pub fn get_space_handle(&self) -> Handle {
        self.space_handle
    }
    pub fn set_max_force(&mut self, force: f32) {
        self.max_force = force;
    }
    pub fn get_max_force(&self) -> f32 {
        self.max_force
    }
    pub fn is_valid(&self) -> bool {
        self.space_handle.is_valid() && self.handle.is_valid()
    }
    pub fn disable_collisions_between_bodies(&mut self, disabled: bool) {
        self.disabled_collisions_between_bodies = disabled;
        if self.is_valid() {
            joint_change_disable_collision(
                self.space_handle,
                self.handle,
                self.disabled_collisions_between_bodies,
            );
        }
    }
    pub fn is_disabled_collisions_between_bodies(&self) -> bool {
        self.disabled_collisions_between_bodies
    }
    pub fn copy_settings_from(&mut self, joint: &RapierJointBase) {
        self.set_max_force(joint.get_max_force());
        self.disable_collisions_between_bodies(joint.is_disabled_collisions_between_bodies());
    }
}

impl Drop for RapierJointBase {
    fn drop(&mut self) {
        if self.is_valid() {
            joint_destroy(self.space_handle, self.handle);
        }
    }
}

pub struct RapierEmptyJoint {
    base: RapierJointBase,
}

impl RapierEmptyJoint {
    pub fn new(rid: Rid) -> Self {
        Self {
            base: RapierJointBase::new(invalid_handle(), invalid_handle(), rid),
        }
    }
}

impl IRapierJoint for RapierEmptyJoint {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::MAX
    }
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }
    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        None
    }
    fn get_pin(&self) -> Option<&RapierPinJoint2D> {
        None
    }
    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }
    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D> {
        None
    }
}
