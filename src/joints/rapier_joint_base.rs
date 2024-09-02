use servers::rapier_physics_singleton::get_rid;

use crate::rapier_wrapper::prelude::*;
use crate::*;
pub struct RapierJointBase {
    max_force: f32,
    handle: JointHandle,
    space_handle: WorldHandle,
    disabled_collisions_between_bodies: bool,
}
impl Default for RapierJointBase {
    fn default() -> Self {
        Self::new(WorldHandle::default(), JointHandle::default())
    }
}
impl RapierJointBase {
    pub fn new(space_handle: WorldHandle, handle: JointHandle) -> Self {
        Self {
            max_force: f32::MAX,
            handle,
            space_handle,
            disabled_collisions_between_bodies: true,
        }
    }

    pub fn get_handle(&self) -> JointHandle {
        self.handle
    }

    pub fn get_space_handle(&self) -> WorldHandle {
        self.space_handle
    }

    pub fn get_space(&self) -> Rid {
        *get_rid(self.space_handle)
    }

    pub fn set_max_force(&mut self, force: f32) {
        self.max_force = force;
    }

    pub fn get_max_force(&self) -> f32 {
        self.max_force
    }

    pub fn is_valid(&self) -> bool {
        self.space_handle != WorldHandle::default() && self.handle != JointHandle::default()
    }

    pub fn disable_collisions_between_bodies(
        &mut self,
        disabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.disabled_collisions_between_bodies = disabled;
        if self.is_valid() {
            physics_engine.joint_change_disable_collision(
                self.space_handle,
                self.handle,
                self.disabled_collisions_between_bodies,
            );
        }
    }

    pub fn is_disabled_collisions_between_bodies(&self) -> bool {
        self.disabled_collisions_between_bodies
    }

    pub fn copy_settings_from(
        &mut self,
        joint: &RapierJointBase,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.set_max_force(joint.get_max_force());
        self.disable_collisions_between_bodies(
            joint.is_disabled_collisions_between_bodies(),
            physics_engine,
        );
    }

    pub fn destroy_joint(&mut self, physics_engine: &mut PhysicsEngine) {
        physics_engine.destroy_joint(self.space_handle, self.handle);
        self.handle = JointHandle::default();
    }
}
