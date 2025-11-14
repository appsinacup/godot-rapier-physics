use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;

use crate::rapier_wrapper::prelude::*;
use crate::*;
#[derive(Default, Debug, PartialEq, Clone, Copy)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum RapierJointType {
    #[default]
    Impulse,
    MultiBody,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(Default, Debug, PartialEq, Clone, Copy)]
pub struct RapierJointBaseState {
    id: RapierId,
    handle: JointHandle,
    space_handle: WorldHandle,
    space_id: RapierId,
    joint_type: RapierJointType,
}
pub struct RapierJointBase {
    rid: Rid,
    max_force: f32,
    disabled_collisions_between_bodies: bool,
    state: RapierJointBaseState,
}
impl Default for RapierJointBase {
    fn default() -> Self {
        Self::new(
            RapierId::default(),
            Rid::Invalid,
            RapierId::default(),
            WorldHandle::default(),
            JointHandle::default(),
            RapierJointType::default(),
        )
    }
}
impl RapierJointBase {
    pub fn new(
        id: RapierId,
        rid: Rid,
        space_id: RapierId,
        space_handle: WorldHandle,
        handle: JointHandle,
        joint_type: RapierJointType,
    ) -> Self {
        Self {
            rid,
            max_force: f32::MAX,
            disabled_collisions_between_bodies: true,
            state: RapierJointBaseState {
                id,
                handle,
                space_handle,
                space_id,
                joint_type,
            },
        }
    }

    pub fn get_handle(&self) -> JointHandle {
        self.state.handle
    }

    pub fn set_handle(&mut self, handle: JointHandle) {
        self.state.handle = handle;
    }

    pub fn get_id(&self) -> RapierId {
        self.state.id
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn get_space_id(&self) -> WorldHandle {
        self.state.space_handle
    }

    pub fn get_space(&self, physics_ids: &PhysicsIds) -> Rid {
        get_id_rid(self.state.space_id, physics_ids)
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

    pub fn get_joint_type(&self) -> RapierJointType {
        self.state.joint_type
    }

    pub fn set_joint_type(&mut self, joint_type: RapierJointType) {
        self.state.joint_type = joint_type;
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
        // Copy the disabled collisions flag without immediately applying it
        // The actual physics engine update will happen later when the joint is fully initialized
        self.disabled_collisions_between_bodies = joint.is_disabled_collisions_between_bodies();
        self.set_joint_type(joint.get_joint_type());
        self.state.id = joint.get_id();
        self.rid = joint.get_rid();
        // Now apply the collision settings to the physics engine if the joint is valid
        if self.is_valid() {
            physics_engine.joint_change_disable_collision(
                self.state.space_handle,
                self.state.handle,
                self.disabled_collisions_between_bodies,
            );
        }
    }

    pub fn destroy_joint(&mut self, physics_engine: &mut PhysicsEngine) {
        physics_engine.destroy_joint(self.state.space_handle, self.state.handle);
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
