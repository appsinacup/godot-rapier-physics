#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::classes::ProjectSettings;
use godot::prelude::*;
use rapier::dynamics::RigidBodyHandle;
use rapier::geometry::ColliderHandle;
use servers::rapier_physics_singleton::PhysicsShapes;
use servers::rapier_physics_singleton::PhysicsSpaces;
use servers::rapier_project_settings::RapierProjectSettings;

use super::rapier_area::RapierArea;
use super::rapier_body::RapierBody;
use crate::rapier_wrapper::prelude::*;
use crate::types::*;
use crate::*;
#[cfg(feature = "dim2")]
const SLEEP_THRESHOLD_LINEAR: &str = "physics/2d/sleep_threshold_linear";
#[cfg(feature = "dim3")]
const SLEEP_THRESHOLD_LINEAR: &str = "physics/3d/sleep_threshold_linear";
#[cfg(feature = "dim2")]
const SLEEP_THRESHOLD_ANGULAR: &str = "physics/2d/sleep_threshold_angular";
#[cfg(feature = "dim3")]
const SLEEP_THRESHOLD_ANGULAR: &str = "physics/3d/sleep_threshold_angular";
#[cfg(feature = "dim2")]
const TIME_BEFORE_SLEEP: &str = "physics/2d/time_before_sleep";
#[cfg(feature = "dim3")]
const TIME_BEFORE_SLEEP: &str = "physics/3d/time_before_sleep";
//#[cfg_attr(feature = "serde-serialize", typetag::serde(tag = "type"))]
pub trait IRapierCollisionObject: Sync {
    fn get_base(&self) -> &RapierCollisionObject;
    fn get_mut_base(&mut self) -> &mut RapierCollisionObject;
    fn get_body(&self) -> Option<&RapierBody>;
    fn get_area(&self) -> Option<&RapierArea>;
    fn get_mut_body(&mut self) -> Option<&mut RapierBody>;
    fn get_mut_area(&mut self) -> Option<&mut RapierArea>;
    fn set_space(
        &mut self,
        space: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn recreate_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
    );
    fn add_shape(
        &mut self,
        p_shape: Rid,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn set_shape(
        &mut self,
        shape_idx: usize,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn set_shape_transform(
        &mut self,
        shape_idx: usize,
        transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn set_shape_disabled(
        &mut self,
        shape_idx: usize,
        disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn remove_shape_idx(
        &mut self,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn remove_shape_rid(
        &mut self,
        shape_rid: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    );
    fn create_shape(
        &mut self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
    ) -> ColliderHandle;
    fn init_material(&self) -> Material;
    fn shapes_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    );
    fn shape_changed(
        &mut self,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
    );
}
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum CollisionObjectType {
    Area,
    Body,
}
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct CollisionObjectShape {
    pub xform: Transform,
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "invalid_rid"))]
    pub shape: Rid,
    pub disabled: bool,
    pub one_way_collision: bool,
    pub one_way_collision_margin: real,
    pub collider_handle: ColliderHandle,
}
impl Default for CollisionObjectShape {
    fn default() -> Self {
        Self {
            xform: Transform::IDENTITY,
            shape: Rid::Invalid,
            disabled: false,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: ColliderHandle::invalid(),
        }
    }
}
// TODO deserialize
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
pub struct RapierCollisionObject {
    user_flags: u32,
    collision_object_type: CollisionObjectType,
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "invalid_rid"))]
    rid: Rid,
    instance_id: u64,
    canvas_instance_id: u64,
    pickable: bool,
    pub(crate) shapes: Vec<CollisionObjectShape>,
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "invalid_rid"))]
    space: Rid,
    transform: Transform,
    inv_transform: Transform,
    collision_mask: u32,
    collision_layer: u32,
    pub(crate) is_debugging_contacts: bool,
    // TODO serialize this
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) mode: BodyMode,
    body_handle: RigidBodyHandle,
    space_handle: WorldHandle,
    pub(crate) activation_angular_threshold: real,
    pub(crate) activation_linear_threshold: real,
    pub(crate) activation_time_until_sleep: real,
}
impl Default for RapierCollisionObject {
    fn default() -> Self {
        Self::new(Rid::Invalid, CollisionObjectType::Body)
    }
}
impl RapierCollisionObject {
    pub fn new(rid: Rid, collision_object_type: CollisionObjectType) -> Self {
        let mut mode = BodyMode::RIGID;
        if collision_object_type == CollisionObjectType::Area {
            mode = BodyMode::STATIC;
        }
        let project_settings = ProjectSettings::singleton();
        let activation_angular_threshold = project_settings
            .get_setting_with_override(SLEEP_THRESHOLD_ANGULAR.into())
            .try_to()
            .unwrap_or_default();
        let length_unit = RapierProjectSettings::get_length_unit();
        let mut activation_linear_threshold = project_settings
            .get_setting_with_override(SLEEP_THRESHOLD_LINEAR.into())
            .try_to()
            .unwrap_or_default();
        activation_linear_threshold /= length_unit;
        let activation_time_until_sleep = project_settings
            .get_setting_with_override(TIME_BEFORE_SLEEP.into())
            .try_to()
            .unwrap_or_default();
        Self {
            user_flags: 0,
            collision_object_type,
            rid,
            instance_id: 0,
            canvas_instance_id: 0,
            pickable: true,
            shapes: Vec::new(),
            space: Rid::Invalid,
            transform: Transform::IDENTITY,
            inv_transform: Transform::IDENTITY,
            collision_mask: 1,
            collision_layer: 1,
            is_debugging_contacts: false,
            mode,
            body_handle: RigidBodyHandle::invalid(),
            space_handle: WorldHandle::default(),
            activation_angular_threshold,
            activation_linear_threshold,
            activation_time_until_sleep,
        }
    }

    pub(super) fn create_shape(
        &self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        mat: Material,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
    ) -> ColliderHandle {
        if shape.collider_handle != ColliderHandle::invalid() {
            godot_error!("collider is valid");
        }
        let mut handle = ColliderHandle::invalid();
        if let Some(shape_object) = physics_shapes.get_mut(&shape.shape) {
            let shape_handle = shape_object.get_handle();
            if shape_handle == ShapeHandle::default() {
                return handle;
            }
            let mut user_data = UserData::default();
            self.set_collider_user_data(&mut user_data, p_shape_index);
            match self.collision_object_type {
                CollisionObjectType::Body => {
                    handle = physics_engine.collider_create_solid(
                        self.space_handle,
                        shape_handle,
                        &mat,
                        self.body_handle,
                        &user_data,
                    );
                }
                CollisionObjectType::Area => {
                    handle = physics_engine.collider_create_sensor(
                        self.space_handle,
                        shape_handle,
                        &mat,
                        self.body_handle,
                        &user_data,
                    );
                }
            }
        }
        handle
    }

    pub(super) fn update_shapes_indexes(&mut self, physics_engine: &mut PhysicsEngine) {
        if !self.is_valid() {
            return;
        }
        for (shape_index, shape) in self.shapes.iter().enumerate() {
            let mut user_data = UserData::default();
            self.set_collider_user_data(&mut user_data, shape_index);
            physics_engine.collider_set_user_data(
                self.space_handle,
                shape.collider_handle,
                &user_data,
            );
        }
    }

    pub(super) fn destroy_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        if let Some(space) = physics_spaces.get_mut(&self.space) {
            for (i, shape) in self.shapes.iter_mut().enumerate() {
                if shape.collider_handle == ColliderHandle::invalid() {
                    // skip
                    continue;
                }
                // Keep track of body information for delayed removal
                space.add_removed_collider(
                    shape.collider_handle,
                    self.rid,
                    self.instance_id,
                    i,
                    self.collision_object_type,
                );
                physics_engine.collider_destroy(self.space_handle, shape.collider_handle);
                shape.collider_handle = ColliderHandle::invalid();
            }
        }
    }

    pub(super) fn destroy_shape(
        &self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_spaces: &mut PhysicsSpaces,
        physics_engine: &mut PhysicsEngine,
    ) -> ColliderHandle {
        if shape.collider_handle != ColliderHandle::invalid() {
            if let Some(space) = physics_spaces.get_mut(&self.space) {
                // Keep track of body information for delayed removal
                space.add_removed_collider(
                    shape.collider_handle,
                    self.rid,
                    self.instance_id,
                    p_shape_index,
                    self.get_type(),
                );
            }
            physics_engine.collider_destroy(self.space_handle, shape.collider_handle);
        }
        ColliderHandle::invalid()
    }

    pub(super) fn update_shape_transform(
        &self,
        shape: &CollisionObjectShape,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
    ) {
        if !self.is_space_valid() || shape.collider_handle == ColliderHandle::invalid() {
            return;
        }
        if let Some(rapier_shape) = physics_shapes.get_mut(&shape.shape) {
            let shape_handle = rapier_shape.get_handle();
            if shape_handle == ShapeHandle::default() {
                godot_error!("Rapier shape is invalid");
                return;
            }
            let scale = transform_scale(&self.transform);
            let mut shape_info = shape_info_from_body_shape(shape_handle, shape.xform);
            shape_info.scale = vector_to_rapier(vector_to_godot(shape_info.scale) * scale);
            let position = shape_info
                .transform
                .translation
                .vector
                .component_mul(&vector_to_rapier(scale));
            shape_info.transform.translation.vector = position;
            physics_engine.collider_set_transform(
                self.space_handle,
                shape.collider_handle,
                shape_info,
            );
        }
    }

    pub(super) fn update_transform(&mut self, physics_engine: &mut PhysicsEngine) {
        if !self.is_valid() {
            return;
        }
        let position = physics_engine.body_get_position(self.space_handle, self.body_handle);
        let rotation = physics_engine.body_get_angle(self.space_handle, self.body_handle);
        self.transform = transform_update(&self.transform, rotation, vector_to_godot(position));
        self.inv_transform = transform_inverse(&self.transform);
    }

    pub(super) fn set_space(
        &mut self,
        p_space: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        // previous space
        if self.is_space_valid() {
            if self.is_body_valid() {
                // This call also destroys the colliders
                physics_engine.body_destroy(self.space_handle, self.body_handle);
                self.body_handle = RigidBodyHandle::invalid();
            }
            self.destroy_shapes(physics_engine, physics_spaces);
            // Reset area detection counter to keep it consistent for new detections
            if let Some(space) = physics_spaces.get_mut(&self.space) {
                space.reset_space_if_empty(physics_engine);
            }
        }
        self.space = p_space;
        if let Some(space) = physics_spaces.get_mut(&self.space) {
            self.space_handle = space.get_handle();
            self.is_debugging_contacts = space.is_debugging_contacts();
        } else {
            self.space_handle = WorldHandle::default();
            self.space = Rid::Invalid;
            return;
        }
        let user_data = UserData {
            part1: self.rid.to_u64(),
            ..Default::default()
        };
        let position = vector_to_rapier(self.transform.origin);
        let angle = transform_rotation_rapier(&self.transform);
        if self.mode == BodyMode::STATIC {
            self.body_handle = physics_engine.body_create(
                self.space_handle,
                position,
                angle,
                &user_data,
                BodyType::Static,
                self.activation_angular_threshold,
                self.activation_linear_threshold,
                self.activation_time_until_sleep,
            );
        } else if self.mode == BodyMode::KINEMATIC {
            self.body_handle = physics_engine.body_create(
                self.space_handle,
                position,
                angle,
                &user_data,
                BodyType::Kinematic,
                self.activation_angular_threshold,
                self.activation_linear_threshold,
                self.activation_time_until_sleep,
            );
        } else {
            self.body_handle = physics_engine.body_create(
                self.space_handle,
                position,
                angle,
                &user_data,
                BodyType::Dynamic,
                self.activation_angular_threshold,
                self.activation_linear_threshold,
                self.activation_time_until_sleep,
            );
        }
    }

    pub fn get_space_handle(&self) -> WorldHandle {
        self.space_handle
    }

    pub fn is_valid(&self) -> bool {
        self.is_body_valid() && self.is_space_valid()
    }

    pub fn is_body_valid(&self) -> bool {
        self.body_handle != RigidBodyHandle::invalid()
    }

    pub fn is_space_valid(&self) -> bool {
        self.space_handle != WorldHandle::default()
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn set_instance_id(&mut self, p_instance_id: u64) {
        self.instance_id = p_instance_id;
    }

    pub fn get_instance_id(&self) -> u64 {
        self.instance_id
    }

    pub fn get_body_handle(&self) -> RigidBodyHandle {
        self.body_handle
    }

    pub fn set_canvas_instance_id(&mut self, p_canvas_instance_id: u64) {
        self.canvas_instance_id = p_canvas_instance_id;
    }

    pub fn get_canvas_instance_id(&self) -> u64 {
        self.canvas_instance_id
    }

    pub fn set_collider_user_data(&self, r_user_data: &mut UserData, p_shape_index: usize) {
        r_user_data.part1 = self.rid.to_u64();
        r_user_data.part2 = p_shape_index as u64;
    }

    pub fn get_collider_user_data(p_user_data: &UserData) -> (Rid, usize) {
        (Rid::new(p_user_data.part1), p_user_data.part2 as usize)
    }

    pub fn get_type(&self) -> CollisionObjectType {
        self.collision_object_type
    }

    pub fn get_shape_count(&self) -> i32 {
        self.shapes.len() as i32
    }

    pub fn get_shape(&self, idx: usize) -> Rid {
        if let Some(shape) = self.shapes.get(idx) {
            return shape.shape;
        }
        Rid::Invalid
    }

    pub fn get_shape_transform(&self, idx: usize) -> Transform {
        if let Some(shape) = self.shapes.get(idx) {
            return shape.xform;
        }
        Transform::IDENTITY
    }

    pub fn set_transform(
        &mut self,
        p_transform: Transform,
        wake_up: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        let teleport = self.transform == Transform::IDENTITY;
        self.transform = p_transform;
        self.inv_transform = transform_inverse(&self.transform);
        if !self.is_valid() {
            return;
        }
        let origin = self.transform.origin;
        let position = vector_to_rapier(origin);
        let rotation = transform_rotation_rapier(&self.transform);
        physics_engine.body_set_transform(
            self.space_handle,
            self.body_handle,
            position,
            rotation,
            teleport,
            wake_up,
        );
    }

    pub fn get_transform(&self) -> Transform {
        self.transform
    }

    pub fn get_inv_transform(&self) -> Transform {
        self.inv_transform
    }

    pub fn get_space(&self) -> Rid {
        self.space
    }

    pub fn is_shape_disabled(&self, idx: usize) -> bool {
        if let Some(shape) = self.shapes.get(idx) {
            return shape.disabled;
        }
        true
    }

    pub fn set_shape_as_one_way_collision(
        &mut self,
        p_idx: usize,
        p_one_way_collision: bool,
        p_margin: real,
    ) {
        if let Some(shape) = self.shapes.get_mut(p_idx) {
            shape.one_way_collision = p_one_way_collision;
            shape.one_way_collision_margin = p_margin;
        }
    }

    pub fn is_shape_set_as_one_way_collision(&self, p_idx: usize) -> bool {
        if let Some(shape) = self.shapes.get(p_idx) {
            return shape.one_way_collision;
        }
        false
    }

    pub fn get_shape_one_way_collision_margin(&self, p_idx: usize) -> real {
        if let Some(shape) = self.shapes.get(p_idx) {
            return shape.one_way_collision_margin;
        }
        0.0
    }

    pub fn set_collision_mask(&mut self, p_mask: u32, physics_engine: &mut PhysicsEngine) {
        self.collision_mask = p_mask;
        if self.is_valid() {
            let material = Material::new(self.collision_layer, self.collision_mask);
            physics_engine.body_update_material(self.space_handle, self.body_handle, &material);
        }
    }

    pub fn get_collision_mask(&self) -> u32 {
        self.collision_mask
    }

    pub fn set_collision_layer(&mut self, p_layer: u32, physics_engine: &mut PhysicsEngine) {
        self.collision_layer = p_layer;
        if self.is_valid() {
            let material = Material::new(self.collision_layer, self.collision_mask);
            physics_engine.body_update_material(self.space_handle, self.body_handle, &material);
        }
    }

    pub fn get_collision_layer(&self) -> u32 {
        self.collision_layer
    }

    pub fn get_mode(&self) -> BodyMode {
        self.mode
    }

    pub fn set_pickable(&mut self, p_pickable: bool) {
        self.pickable = p_pickable;
    }

    pub fn destroy_body(&mut self, physics_engine: &mut PhysicsEngine) {
        if self.body_handle != RigidBodyHandle::invalid() {
            physics_engine.body_destroy(self.space_handle, self.body_handle);
            self.body_handle = RigidBodyHandle::invalid();
        }
    }

    pub fn set_user_flags(&mut self, p_flags: u32) {
        self.user_flags = p_flags;
    }

    pub fn get_user_flags(&self) -> u32 {
        self.user_flags
    }
}
impl Drop for RapierCollisionObject {
    fn drop(&mut self) {
        if self.body_handle != RigidBodyHandle::invalid() {
            godot_error!("Body leaked");
        }
    }
}
