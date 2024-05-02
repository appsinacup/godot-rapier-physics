use crate::bodies::rapier_collision_object_2d::CollisionObjectType;
use crate::bodies::rapier_collision_object_2d::{
    IRapierCollisionObject2D, RapierCollisionObject2D,
};
use crate::rapier2d::handle::Handle;
use godot::builtin::real;
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

    pub fn _shapes_changed() {}

    pub fn _set_space_override_mode(
        r_mode: &AreaSpaceOverrideMode,
        p_value: AreaSpaceOverrideMode,
    ) {
    }
    pub fn _enable_space_override() {}
    pub fn _disable_space_override() {}
    pub fn _reset_space_override() {}

    pub fn on_body_enter(
        &mut self,
        collider_handle: Handle,
        body: Rid,
        body_shape: u32,
        area_collider_handle: Handle,
        area_shape: u32,
    ) {
        // Implementation needed
    }

    pub fn on_body_exit(
        &mut self,
        collider_handle: Handle,
        body: Rid,
        body_shape: u32,
        area_collider_handle: Handle,
        area_shape: u32,
        update_detection: bool,
    ) {
        // Implementation needed
    }

    pub fn on_area_enter(
        &mut self,
        collider_handle: Handle,
        other_area: Rid,
        other_area_shape: u32,
        area_collider_handle: Handle,
        area_shape: u32,
    ) {
        // Implementation needed
    }

    pub fn on_area_exit(
        &mut self,
        collider_handle: Handle,
        other_area: Rid,
        other_area_shape: u32,
        area_collider_handle: Handle,
        area_shape: u32,
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

    pub fn set_param(&mut self, param: AreaParameter, value: Variant) {
        // Implementation needed
    }

    pub fn get_param(&self, param: AreaParameter) -> Variant {
        // Implementation needed
        Variant::nil()
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

    pub fn call_queries(&self) {
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
        &mut &self.base
    }
    fn set_space(&mut self, space: Rid) {}
    
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
}
