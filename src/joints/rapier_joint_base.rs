use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;

use crate::bodies::exportable_object::ExportableObject;
use crate::rapier_wrapper::prelude::*;
use crate::*;
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
#[derive(Debug)]
pub struct JointExport<'a> {
    state: &'a RapierJointBaseState,
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
        )
    }
}
#[cfg(feature = "serde-serialize")]
impl ExportableObject for RapierJointBase {
    type ExportState<'a> = JointExport<'a>;

    fn get_export_state<'a>(&'a self, _: &'a mut PhysicsEngine) -> Option<Self::ExportState<'a>> {
        Some(JointExport {
            state: &self.state,
        })  
    }
}
impl RapierJointBase {
    pub fn new(
        id: RapierId,
        rid: Rid,
        space_id: RapierId,
        space_handle: WorldHandle,
        handle: JointHandle,
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
            },
        }
    }

    pub fn get_handle(&self) -> JointHandle {
        self.state.handle
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
        self.state.id = joint.get_id();
        self.rid = joint.get_rid();
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
    pub fn export_binary(&self) -> Vec<u8> {
        match bincode::serialize(&self.state) {
            Ok(binary_data) => {
                return binary_data
            }
            Err(e) => {
                godot_error!("Failed to serialize joint to binary: {}", e);
            }
        }
        Vec::new()
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
