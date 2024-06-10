use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::*;
use crate::*;
#[cfg(feature = "dim2")]
use godot::engine::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::engine::physics_server_3d::*;
use godot::prelude::*;
use std::any::Any;

use super::{rapier_area::RapierArea, rapier_body::RapierBody};

pub trait IRapierCollisionObject: Any {
    fn get_base(&self) -> &RapierCollisionObject;
    fn get_mut_base(&mut self) -> &mut RapierCollisionObject;
    fn get_body(&self) -> Option<&RapierBody>;
    fn get_area(&self) -> Option<&RapierArea>;
    fn get_mut_body(&mut self) -> Option<&mut RapierBody>;
    fn get_mut_area(&mut self) -> Option<&mut RapierArea>;
    fn set_space(&mut self, space: Rid);
    fn recreate_shapes(&mut self);
    fn add_shape(&mut self, p_shape: Rid, p_transform: Transform, p_disabled: bool);
    fn set_shape(&mut self, shape_idx: usize, p_shape: Rid);
    fn set_shape_transform(&mut self, shape_idx: usize, transform: Transform);
    fn set_shape_disabled(&mut self, shape_idx: usize, disabled: bool);
    fn remove_shape_idx(&mut self, shape_idx: usize);
    fn remove_shape_rid(&mut self, shape_rid: Rid);
    fn create_shape(&mut self, shape: CollisionObjectShape, p_shape_index: usize) -> Handle;

    fn _init_material(&self) -> Material;
    fn _shapes_changed(&mut self);

    fn _shape_changed(&mut self, p_shape: Rid);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CollisionObjectType {
    Area,
    Body,
}

#[derive(Clone, Copy)]
pub struct CollisionObjectShape {
    pub xform: Transform,
    pub shape: Rid,
    pub disabled: bool,
    pub one_way_collision: bool,
    pub one_way_collision_margin: real,
    pub collider_handle: Handle,
}

impl Default for CollisionObjectShape {
    fn default() -> Self {
        Self {
            xform: Transform::default(),
            shape: Rid::Invalid,
            disabled: false,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: invalid_handle(),
        }
    }
}

pub struct RapierCollisionObject {
    collision_object_type: CollisionObjectType,
    rid: Rid,
    instance_id: u64,
    canvas_instance_id: u64,
    pickable: bool,
    pub(crate) shapes: Vec<CollisionObjectShape>,
    space: Rid,
    transform: Transform,
    inv_transform: Transform,
    collision_mask: u32,
    collision_layer: u32,
    collision_priority: real,
    pub(crate) mode: BodyMode,
    body_handle: Handle,
    space_handle: Handle,
    pub(crate) area_detection_counter: u32,
}

impl RapierCollisionObject {
    pub fn new(rid: Rid, collision_object_type: CollisionObjectType) -> Self {
        Self {
            collision_object_type,
            rid,
            instance_id: 0,
            canvas_instance_id: 0,
            pickable: true,
            shapes: Vec::new(),
            space: Rid::Invalid,
            transform: Transform::default(),
            inv_transform: Transform::default(),
            collision_mask: 1,
            collision_layer: 1,
            collision_priority: 1.0,
            mode: BodyMode::RIGID,
            body_handle: invalid_handle(),
            space_handle: invalid_handle(),
            area_detection_counter: 0,
        }
    }

    pub(crate) fn _create_shape(
        &self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        mat: Material,
    ) -> Handle {
        if shape.collider_handle.is_valid() {
            godot_error!("collider is valid");
        }
        let mut handle = invalid_handle();
        if let Some(shape_object) = shapes_singleton().shapes.get_mut(&shape.shape) {
            let shape_handle = shape_object.get_rapier_shape();
            if !shape_handle.is_valid() {
                return handle;
            }

            let mut user_data = UserData::default();
            self.set_collider_user_data(&mut user_data, p_shape_index);

            match self.collision_object_type {
                CollisionObjectType::Body => {
                    handle = collider_create_solid(
                        self.space_handle,
                        shape_handle,
                        &mat,
                        self.body_handle,
                        &user_data,
                    );
                }
                CollisionObjectType::Area => {
                    handle = collider_create_sensor(
                        self.space_handle,
                        shape_handle,
                        self.body_handle,
                        &user_data,
                    );
                }
            }
        }
        handle
    }
    pub(crate) fn _destroy_shapes(&mut self) {
        let mut i = 0;
        for shape in &mut self.shapes {
            let mut shape_rid = Rid::Invalid;
            if !self.space_handle.is_valid() || !shape.collider_handle.is_valid() {
                // skip
                continue;
            }
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.space) {
                if self.area_detection_counter > 0 {
                    // Keep track of body information for delayed removal
                    space.add_removed_collider(
                        shape.collider_handle,
                        self.rid,
                        self.instance_id,
                        i,
                        self.collision_object_type,
                    );
                }

                collider_destroy(self.space_handle, shape.collider_handle);
                shape_rid = shape.shape;
                shape.collider_handle = invalid_handle();
            }
            if let Some(shape) = shapes_singleton().shapes.get_mut(&shape_rid) {
                shape.get_mut_base().destroy_rapier_shape();
            }
            i += 1;
        }
    }
    pub(crate) fn _destroy_shape(
        &self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
    ) -> Handle {
        let shape_rid = shape.shape;
        if self.space_handle.is_valid() && shape.collider_handle.is_valid() {
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.space) {
                if self.area_detection_counter > 0 {
                    // Keep track of body information for delayed removal
                    space.add_removed_collider(
                        shape.collider_handle,
                        self.rid,
                        self.instance_id,
                        p_shape_index,
                        self.get_type(),
                    );
                }

                collider_destroy(self.space_handle, shape.collider_handle);
            }
        }
        if let Some(shape) = shapes_singleton().shapes.get_mut(&shape_rid) {
            shape.get_mut_base().destroy_rapier_shape();
        }
        invalid_handle()
    }

    pub(crate) fn update_shape_transform(&self, shape: &CollisionObjectShape) {
        if !self.space_handle.is_valid() || !shape.collider_handle.is_valid() {
            return;
        }
        let origin = shape.xform.origin;
        let position = vector_to_rapier(origin);
        let angle = shape.xform.rotation();
        let scale = shape.xform.scale();
        if let Some(rapier_shape) = shapes_singleton().shapes.get_mut(&shape.shape) {
            let shape_handle = rapier_shape.get_rapier_shape();
            if !shape_handle.is_valid() {
                godot_error!("Rapier shape is invalid");
                return;
            }
            let shape_info = ShapeInfo {
                handle: shape_handle,
                pixel_position: position,
                rotation: angle,
                skew: shape.xform.skew(),
                scale: vector_to_rapier(scale),
            };
            collider_set_transform(self.space_handle, shape.collider_handle, shape_info);
        }
    }

    pub(crate) fn update_transform(&mut self) {
        if !self.is_valid() {
            return;
        }

        let position = body_get_position(self.space_handle, self.body_handle);
        let angle = body_get_angle(self.space_handle, self.body_handle);
        let origin = Vector2::new(position.x, position.y);
        self.transform = Transform::from_angle_scale_skew_origin(
            angle,
            self.transform.scale(),
            self.transform.skew(),
            origin,
        );

        self.inv_transform = self.transform.affine_inverse();
    }
    pub(crate) fn _set_space(&mut self, p_space: Rid) {
        // previous space
        if self.space_handle.is_valid() {
            if self.body_handle.is_valid() {
                // This call also destroys the colliders
                body_destroy(self.space_handle, self.body_handle);
                self.body_handle = invalid_handle();
            }

            self._destroy_shapes();

            // Reset area detection counter to keep it consistent for new detections
            self.area_detection_counter = 0;
        }

        self.space = p_space;
        if let Some(space) = spaces_singleton().spaces.get(&self.space) {
            self.space_handle = space.get_handle();
        } else {
            self.space_handle = invalid_handle();
            self.space = Rid::Invalid;
        }
        if self.space_handle.is_valid() {
            let mut user_data = UserData::default();
            user_data.part1 = self.rid.to_u64();

            let position = vector_to_rapier(self.transform.origin);
            let angle = self.transform.rotation();
            if self.mode == BodyMode::STATIC {
                self.body_handle = body_create(
                    self.space_handle,
                    position,
                    angle,
                    &user_data,
                    BodyType::Static,
                );
            } else if self.mode == BodyMode::KINEMATIC {
                self.body_handle = body_create(
                    self.space_handle,
                    position,
                    angle,
                    &user_data,
                    BodyType::Kinematic,
                );
            } else {
                self.body_handle = body_create(
                    self.space_handle,
                    position,
                    angle,
                    &user_data,
                    BodyType::Dynamic,
                );
            }
        }
    }

    pub fn get_space_handle(&self) -> Handle {
        self.space_handle
    }

    pub fn is_valid(&self) -> bool {
        self.body_handle.is_valid() && self.space_handle.is_valid()
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

    pub fn get_body_handle(&self) -> Handle {
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
        self.shapes[idx].shape
    }

    pub fn get_shape_transform(&self, idx: usize) -> Transform {
        self.shapes[idx].xform
    }

    pub fn set_transform(&mut self, p_transform: Transform, wake_up: bool) {
        self.transform = p_transform;
        self.inv_transform = self.transform.affine_inverse();
        if !self.is_valid() {
            return;
        }

        let origin = self.transform.origin;
        let position = vector_to_rapier(origin);
        let rotation = self.transform.rotation();
        body_set_transform(
            self.space_handle,
            self.body_handle,
            position,
            rotation,
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
        self.shapes[idx].disabled
    }

    pub fn set_shape_as_one_way_collision(
        &mut self,
        p_idx: usize,
        p_one_way_collision: bool,
        p_margin: real,
    ) {
        if p_idx > self.shapes.len() {
            godot_error!("invalid index");
            return;
        }

        let shape = &mut self.shapes[p_idx];
        shape.one_way_collision = p_one_way_collision;
        shape.one_way_collision_margin = p_margin;
    }

    pub fn is_shape_set_as_one_way_collision(&self, p_idx: usize) -> bool {
        if p_idx > self.shapes.len() {
            godot_error!("invalid index");
            return false;
        }

        self.shapes[p_idx].one_way_collision
    }

    pub fn get_shape_one_way_collision_margin(&self, p_idx: usize) -> real {
        if p_idx >= self.shapes.len() {
            godot_error!("invalid index");
            return 0.0;
        }

        self.shapes[p_idx].one_way_collision_margin
    }

    pub fn set_collision_mask(&mut self, p_mask: u32) {
        self.collision_mask = p_mask;
    }

    pub fn get_collision_mask(&self) -> u32 {
        self.collision_mask
    }

    pub fn set_collision_layer(&mut self, p_layer: u32) {
        self.collision_layer = p_layer;
    }

    pub fn get_collision_layer(&self) -> u32 {
        self.collision_layer
    }

    pub fn set_collision_priority(&mut self, p_priority: real) {
        if p_priority < 0.0 {
            return;
        }
        self.collision_priority = p_priority;
    }

    pub fn get_collision_priority(&self) -> real {
        self.collision_priority
    }

    pub fn get_mode(&self) -> BodyMode {
        self.mode
    }

    pub fn set_pickable(&mut self, p_pickable: bool) {
        self.pickable = p_pickable;
    }

    pub fn interacts_with(&self, p_other: &RapierCollisionObject) -> bool {
        self.collision_layer & p_other.collision_mask != 0
            || p_other.collision_layer & self.collision_mask != 0
    }
}
