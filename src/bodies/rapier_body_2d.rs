
use godot::engine::native::ObjectId;
// Import necessary libraries
use godot::prelude::*;
use godot::engine::physics_server_2d::{BodyDampMode, BodyMode, BodyState, CcdMode};
use std::collections::HashSet;

use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::bodies::rapier_collision_object_2d::RapierCollisionObject2D;

use super::rapier_collision_object_2d::CollisionObjectType;

// Define the RapierBody2D struct
pub struct RapierBody2D {
    linear_damping_mode: BodyDampMode,
    angular_damping_mode: BodyDampMode,
    linear_damping: f32,
    angular_damping: f32,
    total_linear_damping: f32,
    total_angular_damping: f32,
    total_gravity: Vector2,
    gravity_scale: f32,
    bounce: f32,
    friction: f32,
    mass: f32,
    inertia: f32,
    center_of_mass: Vector2,
    calculate_inertia: bool,
    calculate_center_of_mass: bool,
    using_area_gravity: bool,
    using_area_linear_damping: bool,
    using_area_angular_damping: bool,
    exceptions: HashSet<Rid>,
    ccd_mode: CcdMode,
    omit_force_integration: bool,
    active: bool,
    marked_active: bool,
    can_sleep: bool,
    constant_force: Vector2,
    linear_velocity: Vector2,
    impulse: Vector2,
    torque: f32,
    angular_velocity: f32,
    constant_torque: f32,
    to_add_angular_velocity: f32,
    to_add_linear_velocity: Vector2,
    sleep: bool,
    base: RapierCollisionObject2D
}

impl RapierBody2D {
    pub fn new() -> Self {
        Self {
            linear_damping_mode: BodyDampMode::COMBINE,
            angular_damping_mode: BodyDampMode::COMBINE,
            linear_damping: 0.0,
            angular_damping: 0.0,
            total_linear_damping: 0.0,
            total_angular_damping: 0.0,
            total_gravity: Vector2::ZERO,
            gravity_scale: 1.0,
            bounce: 0.0,
            friction: 1.0,
            mass: 0.0,
            inertia: 0.0,
            center_of_mass: Vector2::ZERO,
            calculate_inertia: false,
            calculate_center_of_mass: false,
            using_area_gravity: false,
            using_area_linear_damping: false,
            using_area_angular_damping: false,
            exceptions: HashSet::new(),
            ccd_mode: CcdMode::DISABLED,
            omit_force_integration: false,
            active: true,
            marked_active: true,
            can_sleep: true,
            constant_force: Vector2::ZERO,
            linear_velocity: Vector2::ZERO,
            impulse: Vector2::ZERO,
            torque: 0.0,
            angular_velocity: 0.0,
            constant_torque: 0.0,
            to_add_angular_velocity: 0.0,
            to_add_linear_velocity: Vector2::ZERO,
            sleep: false,
            base: RapierCollisionObject2D::new(CollisionObjectType::Body)
        }
    }
    
    fn add_contact(
        &mut self,
        local_pos: Vector2,
        local_normal: Vector2,
        depth: f32,
        local_shape: i32,
        local_velocity_at_pos: Vector2,
        collider_pos: Vector2,
        collider_shape: i32,
        collider_instance_id: ObjectId,
        collider_object: Option<Object>,
        collider: Rid,
        collider_velocity_at_pos: Vector2,
        impulse: Vector2,
    ) {
    }
}

// Define the Contact struct
struct Contact {
    local_pos: Vector2,
    local_normal: Vector2,
    depth: f32,
    local_shape: i32,
    collider_pos: Vector2,
    collider_shape: i32,
    collider_instance_id: ObjectId,
    collider_object: Option<Object>,
    collider: Rid,
    local_velocity_at_pos: Vector2,
    collider_velocity_at_pos: Vector2,
    impulse: Vector2,
}

impl IRapierCollisionObject2D for RapierBody2D {
    fn get_base(&self) -> &RapierCollisionObject2D {
        &self.base
    }
}