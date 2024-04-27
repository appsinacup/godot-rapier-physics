use std::collections::HashMap;

use godot::{builtin::{Rid, Vector2}, engine::native::ObjectId, obj::Gd};

use crate::{bodies::rapier_collision_object_2d::Type, rapier2d::handle::{invalid_handle, Handle}};

use super::rapier_direct_space_state_2d::RapierDirectSpaceState2D;

pub struct RapierSpace2D {
    // Add fields here
    direct_access: Option<Gd<RapierDirectSpaceState2D>>,
    rid: Rid,
    handle: Handle,
    removed_colliders: HashMap<u32, RemovedColliderInfo>,
    active_list: Vec<Rid>,
    mass_properties_update_list: Vec<Rid>,
    gravity_update_list: Vec<Rid>,
    state_query_list: Vec<Rid>,
    monitor_query_list: Vec<Rid>,
    area_update_list: Vec<Rid>,
    body_area_update_list: Vec<Rid>,
    solver_iterations: u32,
    contact_recycle_radius: f32,
    contact_max_separation: f32,
    contact_max_allowed_penetration: f32,
    contact_bias: f32,
    constraint_bias: f32,
    fluid_default_gravity_dir: Vector2,
    fluid_default_gravity_value: f32,
    default_gravity_dir: Vector2,
    default_gravity_value: f32,
    default_linear_damping: f32,
    default_angular_damping: f32,
    locked: bool,
    last_step: f32,
    island_count: i32,
    active_objects: i32,
    collision_pairs: i32,
    contact_debug: Vec<Vector2>,
    contact_debug_count: i32,
}

struct RemovedColliderInfo {
    rid: Rid,
    instance_id: ObjectId,
    shape_index: u32,
    type_: Type,
}

impl RapierSpace2D {
    pub fn new() -> Self {
        // Initialize fields here
        Self {
            direct_access: None,
            rid: Rid::Invalid,
            handle: invalid_handle(),
            removed_colliders: HashMap::new(),
            active_list: Vec::new(),
            mass_properties_update_list: Vec::new(),
            gravity_update_list: Vec::new(),
            state_query_list: Vec::new(),
            monitor_query_list: Vec::new(),
            area_update_list: Vec::new(),
            body_area_update_list: Vec::new(),
            solver_iterations: 0,
            contact_recycle_radius: 0.0,
            contact_max_separation: 0.0,
            contact_max_allowed_penetration: 0.0,
            contact_bias: 0.0,
            constraint_bias: 0.0,
            fluid_default_gravity_dir: Vector2::new(0.0, -1.0),
            fluid_default_gravity_value: -9.81,
            default_gravity_dir: Vector2::new(0.0, -1.0),
            default_gravity_value: -9.81,
            default_linear_damping: 0.0,
            default_angular_damping: 0.0,
            locked: false,
            last_step: 0.001,
            island_count: 0,
            active_objects: 0,
            collision_pairs: 0,
            contact_debug: Vec::new(),
            contact_debug_count: 0,
        }
    }
}
