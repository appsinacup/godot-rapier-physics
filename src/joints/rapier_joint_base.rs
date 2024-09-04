use servers::rapier_physics_singleton::get_rid;
use servers::rapier_physics_singleton::PhysicsRids;

use crate::rapier_wrapper::prelude::*;
use crate::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(Default, Debug, PartialEq, Clone, Copy)]
pub struct RapierJointBaseState {
    handle: JointHandle,
    space_handle: WorldHandle,
}
pub struct RapierJointBase {
    max_force: f32,
    disabled_collisions_between_bodies: bool,
    state: RapierJointBaseState,
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
            disabled_collisions_between_bodies: true,
            state: RapierJointBaseState {
                handle,
                space_handle,
            },
        }
    }

    pub fn get_handle(&self) -> JointHandle {
        self.state.handle
    }

    pub fn get_space_handle(&self) -> WorldHandle {
        self.state.space_handle
    }

    pub fn get_space(&self, physics_rids: &PhysicsRids) -> Rid {
        get_rid(self.state.space_handle, physics_rids)
    }

    pub fn set_max_force(&mut self, force: f32) {
        self.max_force = force;
    }

    pub fn get_max_force(&self) -> f32 {
        self.max_force
    }

    pub fn is_valid(&self) -> bool {
        self.state.space_handle != WorldHandle::default()
            && self.state.handle != JointHandle::default()
    }

    pub fn disable_collisions_between_bodies(
        &mut self,
        disabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.disabled_collisions_between_bodies = disabled;
        if self.is_valid() {
            physics_engine.joint_change_disable_collision(
                self.state.space_handle,
                self.state.handle,
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

    pub fn destroy_joint(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        physics_engine.destroy_joint(self.state.space_handle, self.state.handle);
        physics_rids.remove(&self.state.handle.index);
        self.state.handle = JointHandle::default();
    }

    #[cfg(feature = "serde-serialize")]
    pub fn export_json(&self) -> String {
        match serde_json::to_string_pretty(&self.state) {
            Ok(s) => return s,
            Err(e) => {
                godot_error!("Failed to serialize joint to json: {}", e);
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    pub fn export_binary(&self) -> PackedByteArray {
        let mut buf = PackedByteArray::new();
        match bincode::serialize(&self.state) {
            Ok(binary_data) => {
                buf.resize(binary_data.len());
                for i in 0..binary_data.len() {
                    buf[i] = binary_data[i];
                }
            }
            Err(e) => {
                godot_error!("Failed to serialize joint to binary: {}", e);
            }
        }
        buf
    }

    #[cfg(feature = "serde-serialize")]
    pub fn import_binary(&mut self, data: PackedByteArray) {
        match bincode::deserialize::<RapierJointBaseState>(data.as_slice()) {
            Ok(import) => {
                self.state = import;
            }
            Err(e) => {
                godot_error!("Failed to deserialize joint from binary: {}", e);
            }
        }
    }
}
