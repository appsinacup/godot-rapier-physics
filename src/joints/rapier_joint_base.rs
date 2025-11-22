use rapier::prelude::*;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;

use crate::bodies::exportable_object::ExportableObject;
use crate::bodies::exportable_object::ImportToExport;
use crate::bodies::exportable_object::ObjectImportState;
use crate::bodies::exportable_object::ExportToImport;
use crate::rapier_wrapper::prelude::*;
use crate::*;
#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum RapierJointType {
    #[default]
    Impulse,
    MultiBody,
    MultiBodyKinematic,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
#[derive(Debug)]
pub struct JointExport<'a> {
    state: &'a RapierJointBaseState,
}
impl<'a> ExportToImport for JointExport<'a> {
    type Import = JointImport;
    fn into_import(self) -> Self::Import {
        JointImport { state: *self.state }
    }
}
#[derive(serde::Deserialize, Clone)]
pub struct JointImport {
    state: RapierJointBaseState,
}
impl ImportToExport for JointImport {
    type Export<'a> = JointExport<'a>;

    fn from_import<'a>(&'a self) -> Self::Export<'a> {
        JointExport { state: &self.state }
    }    
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
    pub custom_ik_options: InverseKinematicsOption,
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
#[cfg(feature = "serde-serialize")]
impl ExportableObject for RapierJointBase {
    type ExportState<'a> = JointExport<'a>;

    fn get_export_state<'a>(&'a self, _: &'a mut PhysicsEngine) -> Option<Self::ExportState<'a>> {
        Some(JointExport { state: &self.state })
    }

    fn import_state(&mut self, _: &mut PhysicsEngine, data: ObjectImportState) {
        match data {
            bodies::exportable_object::ObjectImportState::JointBase(joint_import) => {
                self.state = joint_import.state;
            }
            _ => {
                godot_error!("Attempted to import invalid state data.");
            }
        }
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
            custom_ik_options: InverseKinematicsOption::default(),
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
        self.disable_collisions_between_bodies(
            joint.is_disabled_collisions_between_bodies(),
            physics_engine,
        );
        self.set_joint_type(joint.get_joint_type());
        self.state.id = joint.get_id();
        self.rid = joint.get_rid();
    }

    pub fn destroy_joint(&mut self, physics_engine: &mut PhysicsEngine) {
        physics_engine.destroy_joint(self.state.space_handle, self.state.handle);
        self.state.handle = JointHandle::default();
    }
}
