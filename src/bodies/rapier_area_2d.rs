use crate::bodies::rapier_collision_object_2d::{CollisionObjectShape, CollisionObjectType};
use crate::bodies::rapier_collision_object_2d::{
    IRapierCollisionObject2D, RapierCollisionObject2D,
};
use crate::rapier2d::collider::{default_material, Material};
use crate::rapier2d::handle::{invalid_handle, Handle};
use crate::servers::rapier_physics_singleton_2d::{shapes_singleton, spaces_singleton};
use crate::spaces::rapier_space_2d::RapierSpace2D;
use godot::builtin::{real, Transform2D};
use godot::meta::ToGodot;
use godot::obj::EngineEnum;
use godot::{
    builtin::{Callable, Rid, Variant, Vector2},
    engine::physics_server_2d::{AreaParameter, AreaSpaceOverrideMode},
};
use std::collections::HashMap;

use super::rapier_body_2d::RapierBody2D;

struct MonitorInfo {
    rid: Rid,
    instance_id: u64,
    object_shape_index: u32,
    area_shape_index: u32,
    collision_object_type: CollisionObjectType,
    state: i32,
}

struct BodyRefCount {
    count: u32,
}

pub struct RapierArea2D {
    gravity_override_mode: AreaSpaceOverrideMode,
    linear_damping_override_mode: AreaSpaceOverrideMode,
    angular_damping_override_mode: AreaSpaceOverrideMode,
    gravity: real,
    gravity_vector: Vector2,
    gravity_is_point: bool,
    gravity_point_unit_distance: real,
    linear_damp: real,
    angular_damp: real,
    priority: i32,
    monitorable: bool,
    monitor_callback: Callable,
    area_monitor_callback: Callable,
    monitored_objects: HashMap<u64, MonitorInfo>,
    detected_bodies: HashMap<Rid, BodyRefCount>,
    monitor_query_list: Vec<Rid>,
    area_override_update_list: Vec<Rid>,
    base: RapierCollisionObject2D,
}

impl RapierArea2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            gravity_override_mode: AreaSpaceOverrideMode::DISABLED,
            linear_damping_override_mode: AreaSpaceOverrideMode::DISABLED,
            angular_damping_override_mode: AreaSpaceOverrideMode::DISABLED,
            gravity: 0.0,
            gravity_vector: Vector2::new(0.0, 0.0),
            gravity_is_point: false,
            gravity_point_unit_distance: 0.0,
            linear_damp: 0.0,
            angular_damp: 0.0,
            priority: 0,
            monitorable: false,
            monitor_callback: Callable::invalid(),
            area_monitor_callback: Callable::invalid(),
            monitored_objects: HashMap::new(),
            detected_bodies: HashMap::new(),
            monitor_query_list: Vec::new(),
            area_override_update_list: Vec::new(),
            base: RapierCollisionObject2D::new(rid, CollisionObjectType::Area),
        }
    }

    pub fn _set_space_override_mode(
        &mut self,
        r_mode: AreaSpaceOverrideMode,
        p_value: AreaSpaceOverrideMode,
    ) {
        // Implementation needed
    }
    pub fn _enable_space_override() {
        // Implementation needed
    }
    pub fn _disable_space_override() {
        // Implementation needed
    }
    pub fn _reset_space_override(&self) {
        // Implementation needed
    }

    pub fn on_body_enter(
        &mut self,
        collider_handle: Handle,
        body: &Option<&mut Box<dyn IRapierCollisionObject2D>>,
        body_shape: usize,
        body_rid: Rid,
        body_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
    ) {
        // Implementation needed
    }

    pub fn on_body_exit(
        &mut self,
        collider_handle: Handle,
        body: &Option<&mut Box<dyn IRapierCollisionObject2D>>,
        body_shape: usize,
        body_rid: Rid,
        body_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
        update_detection: bool,
    ) {
        // if body is null, update_detection should be false
        // Implementation needed
    }

    pub fn on_area_enter(
        &mut self,
        collider_handle: Handle,
        other_area: &Option<&mut Box<dyn IRapierCollisionObject2D>>,
        other_area_shape: usize,
        other_area_rid: Rid,
        other_area_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
    ) {
        // Implementation needed
    }

    pub fn on_area_exit(
        &mut self,
        collider_handle: Handle,
        other_area: &Option<&mut Box<dyn IRapierCollisionObject2D>>,
        other_area_shape: usize,
        other_area_rid: Rid,
        other_area_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
    ) {
        // Implementation needed
    }

    pub fn update_area_override(&mut self) {
        // Implementation needed
    }

    pub fn has_any_space_override(&self) -> bool {
        self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED
            || self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED
            || self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED
    }

    pub fn set_monitor_callback(&mut self, callback: Callable) {
        self.monitor_callback = callback;
    }

    pub fn has_monitor_callback(&self) -> bool {
        self.monitor_callback.is_valid()
    }

    pub fn set_area_monitor_callback(&mut self, callback: Callable) {
        self.area_monitor_callback = callback;
    }

    pub fn has_area_monitor_callback(&self) -> bool {
        self.area_monitor_callback.is_valid()
    }

    pub fn set_param(&mut self, p_param: AreaParameter, p_value: Variant) {
        match p_param {
            AreaParameter::GRAVITY_OVERRIDE_MODE => {
                self._set_space_override_mode(
                    self.gravity_override_mode,
                    AreaSpaceOverrideMode::from_ord(p_value.to()),
                );
            }

            AreaParameter::GRAVITY => {
                let new_gravity = p_value.to();
                if new_gravity != self.gravity {
                    self.gravity = new_gravity;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        let mut lock = spaces_singleton().lock().unwrap();
                        if let Some(space) = lock.spaces.get_mut(&self.base.get_space()) {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_VECTOR => {
                let new_gravity_vector = p_value.to();
                if self.gravity_vector != new_gravity_vector {
                    self.gravity_vector = new_gravity_vector;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        let mut lock = spaces_singleton().lock().unwrap();
                        if let Some(space) = lock.spaces.get_mut(&self.base.get_space()) {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_IS_POINT => {
                let new_gravity_is_point = p_value.to();
                if self.gravity_is_point != new_gravity_is_point {
                    self.gravity_is_point = new_gravity_is_point;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        let mut lock = spaces_singleton().lock().unwrap();
                        if let Some(space) = lock.spaces.get_mut(&self.base.get_space()) {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_POINT_UNIT_DISTANCE => {
                let new_gravity_point_unit_distance = p_value.to();
                if self.gravity_point_unit_distance != new_gravity_point_unit_distance {
                    self.gravity_point_unit_distance = new_gravity_point_unit_distance;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        let mut lock = spaces_singleton().lock().unwrap();
                        if let Some(space) = lock.spaces.get_mut(&self.base.get_space()) {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::LINEAR_DAMP_OVERRIDE_MODE => {
                self._set_space_override_mode(
                    self.linear_damping_override_mode,
                    AreaSpaceOverrideMode::from_ord(p_value.to()),
                );
            }
            AreaParameter::LINEAR_DAMP => {
                let new_linear_damp = p_value.to();
                if self.linear_damp != new_linear_damp {
                    self.linear_damp = new_linear_damp;
                    if self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        let mut lock = spaces_singleton().lock().unwrap();
                        if let Some(space) = lock.spaces.get_mut(&self.base.get_space()) {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE => {
                self._set_space_override_mode(
                    self.angular_damping_override_mode,
                    AreaSpaceOverrideMode::from_ord(p_value.to()),
                );
            }
            AreaParameter::ANGULAR_DAMP => {
                let new_angular_damp = p_value.to();
                if self.angular_damp != new_angular_damp {
                    self.angular_damp = new_angular_damp;
                    if self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        let mut lock = spaces_singleton().lock().unwrap();
                        if let Some(space) = lock.spaces.get_mut(&self.base.get_space()) {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::PRIORITY => {
                let new_priority = p_value.to();
                if self.priority != new_priority {
                    self.priority = new_priority;
                    if self.has_any_space_override() {
                        // Need to re-process priority list for each body
                        self._reset_space_override();
                    }
                }
            }
            _ => {}
        }
    }

    pub fn get_param(&self, p_param: AreaParameter) -> Variant {
        match p_param {
            AreaParameter::GRAVITY_OVERRIDE_MODE => self.gravity_override_mode.to_variant(),
            AreaParameter::GRAVITY => self.gravity.to_variant(),
            AreaParameter::GRAVITY_VECTOR => self.gravity_vector.to_variant(),
            AreaParameter::GRAVITY_IS_POINT => self.gravity_is_point.to_variant(),
            AreaParameter::GRAVITY_POINT_UNIT_DISTANCE => {
                self.gravity_point_unit_distance.to_variant()
            }
            AreaParameter::LINEAR_DAMP_OVERRIDE_MODE => {
                self.linear_damping_override_mode.to_variant()
            }
            AreaParameter::LINEAR_DAMP => self.linear_damp.to_variant(),
            AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE => {
                self.angular_damping_override_mode.to_variant()
            }
            AreaParameter::ANGULAR_DAMP => self.angular_damp.to_variant(),
            AreaParameter::PRIORITY => self.priority.to_variant(),
            _ => Variant::nil(),
        }
    }

    pub fn set_gravity(&mut self, gravity: real) {
        self.gravity = gravity;
    }

    pub fn get_gravity(&self) -> real {
        self.gravity
    }

    pub fn set_gravity_vector(&mut self, gravity: Vector2) {
        self.gravity_vector = gravity;
    }

    pub fn get_gravity_vector(&self) -> Vector2 {
        self.gravity_vector
    }

    pub fn set_gravity_as_point(&mut self, enable: bool) {
        self.gravity_is_point = enable;
    }

    pub fn is_gravity_point(&self) -> bool {
        self.gravity_is_point
    }

    pub fn set_gravity_point_unit_distance(&mut self, scale: real) {
        self.gravity_point_unit_distance = scale;
    }

    pub fn get_gravity_point_unit_distance(&self) -> real {
        self.gravity_point_unit_distance
    }

    pub fn set_linear_damp(&mut self, linear_damp: real) {
        self.linear_damp = linear_damp;
    }

    pub fn get_linear_damp(&self) -> real {
        self.linear_damp
    }

    pub fn set_angular_damp(&mut self, angular_damp: real) {
        self.angular_damp = angular_damp;
    }

    pub fn get_angular_damp(&self) -> real {
        self.angular_damp
    }

    pub fn set_priority(&mut self, priority: i32) {
        self.priority = priority;
    }

    pub fn get_priority(&self) -> i32 {
        self.priority
    }

    pub fn set_monitorable(&mut self, monitorable: bool) {
        self.monitorable = monitorable;
    }

    pub fn is_monitorable(&self) -> bool {
        self.monitorable
    }

    pub fn call_queries(&self, space: &mut RapierSpace2D) {
        // Implementation needed
    }

    pub fn compute_gravity(&self, position: &Vector2) -> Vector2 {
        // Implementation needed
        Vector2::default()
    }
}

impl IRapierCollisionObject2D for RapierArea2D {
    fn get_base(&self) -> &RapierCollisionObject2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierCollisionObject2D {
        &mut self.base
    }
    fn set_space(&mut self, space: Rid) {
        // implementation needed
    }

    fn get_body(&self) -> Option<&RapierBody2D> {
        None
    }

    fn get_area(&self) -> Option<&RapierArea2D> {
        Some(self)
    }

    fn get_mut_body(&mut self) -> Option<&mut RapierBody2D> {
        None
    }

    fn get_mut_area(&mut self) -> Option<&mut RapierArea2D> {
        Some(self)
    }

    fn add_shape(
        &mut self,
        p_shape: godot::prelude::Rid,
        p_transform: godot::prelude::Transform2D,
        p_disabled: bool,
    ) {
        let mut shape = CollisionObjectShape {
            xform: p_transform,
            shape: p_shape,
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: invalid_handle(),
        };

        if !shape.disabled {
            shape.collider_handle = self.create_shape(shape, self.base.shapes.len());
            self.base.update_shape_transform(&shape);
        }

        self.base.shapes.push(shape);
        {
            let mut lock = shapes_singleton().lock().unwrap();
            if let Some(shape) = lock.shapes.get_mut(&p_shape) {
                shape.get_mut_base().add_owner(self.base.get_rid());
            }
        }

        if self.base.space_handle.is_valid() {
            self._shapes_changed();
        }
    }

    fn set_shape(&mut self, p_index: usize, p_shape: Rid) {
        assert!(p_index < self.base.shapes.len());

        self.base.shapes[p_index].collider_handle =
            self.base._destroy_shape(self.base.shapes[p_index], p_index);
        let shape = self.base.shapes[p_index];
        {
            let mut lock = shapes_singleton().lock().unwrap();
            if let Some(shape) = lock.shapes.get_mut(&shape.shape) {
                shape.get_mut_base().remove_owner(self.base.get_rid());
            }
        }
        self.base.shapes[p_index].shape = p_shape;
        {
            let mut lock = shapes_singleton().lock().unwrap();
            if let Some(shape) = lock.shapes.get_mut(&p_shape) {
                shape.get_mut_base().add_owner(self.base.get_rid());
            }
        }

        if !shape.disabled {
            self.base
                ._create_shape(shape, p_index, self._init_material());
            self.base.update_shape_transform(&shape);
        }

        if self.base.space_handle.is_valid() {
            self._shapes_changed();
        }
    }

    fn set_shape_transform(&mut self, p_index: usize, p_transform: Transform2D) {
        assert!(p_index < self.base.shapes.len());

        self.base.shapes[p_index].xform = p_transform;
        let shape = &self.base.shapes[p_index];

        self.base.update_shape_transform(shape);

        if self.base.space_handle.is_valid() {
            self._shapes_changed();
        }
    }

    fn set_shape_disabled(&mut self, p_index: usize, p_disabled: bool) {
        assert!(p_index < self.base.shapes.len());
        self.base.shapes[p_index].disabled = p_disabled;
        let shape = self.base.shapes[p_index];
        if shape.disabled == p_disabled {
            return;
        }
        if shape.disabled {
            self.base._destroy_shape(shape, p_index);
        }

        if !shape.disabled {
            self.base
                ._create_shape(shape, p_index, self._init_material());
            self.base.update_shape_transform(&shape);
        }

        if self.base.space_handle.is_valid() {
            self._shapes_changed();
        }
    }

    fn remove_shape_rid(&mut self, shape: Rid) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.base.shapes.len() {
            if self.base.shapes[i].shape == shape {
                self.remove_shape_idx(i);
            } else {
                i += 1;
            }
        }
    }

    fn remove_shape_idx(&mut self, p_index: usize) {
        // remove anything from shape to be erased to end, so subindices don't change
        assert!(p_index < self.base.shapes.len());

        let shape = &self.base.shapes[p_index];

        if !shape.disabled {
            self.base._destroy_shape(*shape, p_index);
        }
        let shape = &mut self.base.shapes[p_index];
        shape.collider_handle = invalid_handle();
        {
            let mut lock = shapes_singleton().lock().unwrap();
            if let Some(shape) = lock.shapes.get_mut(&shape.shape) {
                shape.get_mut_base().remove_owner(self.base.get_rid());
            }
        }

        self.base.shapes.remove(p_index);

        if self.base.space_handle.is_valid() {
            self._shapes_changed();
        }
    }

    fn create_shape(&mut self, shape: CollisionObjectShape, p_shape_index: usize) -> Handle {
        if !self.base.space_handle.is_valid() {
            return invalid_handle();
        }
        let mat = self._init_material();

        self.base._create_shape(shape, p_shape_index, mat)
    }

    fn _init_material(&self) -> Material {
        default_material()
    }

    fn _shape_changed(&mut self, p_shape: Rid) {
        if !self.base.space_handle.is_valid() {
            return;
        }
        for i in 0..self.base.shapes.len() {
            let shape = self.base.shapes[i];
            if shape.shape != p_shape || shape.disabled {
                continue;
            }

            self.base.shapes[i].collider_handle = self.base._destroy_shape(shape, i);

            self.base
                ._create_shape(self.base.shapes[i], i, self._init_material());
            self.base.update_shape_transform(&self.base.shapes[i]);
        }

        self._shapes_changed();
    }

    fn recreate_shapes(&mut self) {
        for i in 0..self.base.get_shape_count() as usize {
            if self.base.shapes[i].disabled {
                continue;
            }
            self.base.shapes[i].collider_handle = self.create_shape(self.base.shapes[i], i);
            self.base.update_shape_transform(&self.base.shapes[i]);
        }
    }

    fn _shapes_changed(&mut self) {}
}
