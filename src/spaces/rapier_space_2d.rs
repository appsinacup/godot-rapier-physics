use std::{collections::HashMap, sync::{Arc, Mutex}};
use godot::{engine::native::ObjectId, prelude::*};
use crate::{bodies::{rapier_area_2d::RapierArea2D, rapier_body_2d::RapierBody2D, rapier_collision_object_2d::{CollisionObjectType, RapierCollisionObject2D}}, rapier2d::{handle::Handle, physics_hooks::{CollisionFilterInfo, OneWayDirection}, physics_world::{ActiveBodyInfo, CollisionEventInfo, ContactForceEventInfo, ContactPointInfo}, query::QueryExcludedInfo, user_data::UserData}};

use super::rapier_direct_space_state_2d::RapierDirectSpaceState2D;

pub struct RapierSpace2D {
    direct_access: Option<Gd<RapierDirectSpaceState2D>>,
    rid: Rid,
    handle: Handle,
    removed_colliders: HashMap<u32, RemovedColliderInfo>,
    active_list: Vec<RapierBody2D>,
    mass_properties_update_list: Vec<RapierBody2D>,
    gravity_update_list: Vec<RapierBody2D>,
    state_query_list: Vec<RapierBody2D>,
    monitor_query_list: Vec<RapierArea2D>,
    area_update_list: Vec<RapierArea2D>,
    body_area_update_list: Vec<RapierBody2D>,
    solver_iterations: i32,
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
    contact_debug: PackedVector2Array,
    contact_debug_count: i32,
}

impl RapierSpace2D {
    fn get_handle(&self) -> Handle {
        self.handle
    }

    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

    fn get_rid(&self) -> Rid {
        self.rid
    }
}

// Define helper structs
struct RemovedColliderInfo {
    rid: Rid,
    instance_id: ObjectId,
    shape_index: u32,
    collision_object_type: CollisionObjectType,
}

struct CollidersInfo {
    shape1: u32,
    object1: Option<RapierCollisionObject2D>,
    shape2: u32,
    object2: Option<RapierCollisionObject2D>,
}

// Implement callbacks
impl RapierSpace2D {
    fn active_body_callback(world_handle: Handle, active_body_info: &ActiveBodyInfo) {
        // Implement callback logic
    }

    fn collision_filter_common_callback(
        world_handle: Handle,
        filter_info: &CollisionFilterInfo,
        r_colliders_info: &mut CollidersInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

    fn collision_filter_body_callback(
        world_handle: Handle,
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

    fn collision_filter_sensor_callback(
        world_handle: Handle,
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

    fn collision_modify_contacts_callback(
        world_handle: Handle,
        filter_info: &CollisionFilterInfo,
    ) -> OneWayDirection {
        // Implement callback logic
        OneWayDirection{
            body1: false,
            body2: false,
            pixel_body1_margin: 0.0,
            pixel_body2_margin: 0.0,
            last_timestep: 0.0,
        }
    }

    fn collision_event_callback(world_handle: Handle, event_info: &CollisionEventInfo) {
        // Implement callback logic
    }

    fn contact_force_event_callback(
        world_handle: Handle,
        event_info: &ContactForceEventInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

    fn contact_point_callback(
        world_handle: Handle,
        contact_info: &ContactPointInfo,
        event_info: &ContactForceEventInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

    fn _is_handle_excluded_callback(
        world_handle: Handle,
        collider_handle: Handle,
        collider: &UserData,
        handle_excluded_info: &QueryExcludedInfo,
    ) -> bool {
        // Implement callback logic
        false
    }

    fn _get_object_instance_hack(p_object_id: u64) -> Option<Object> {
        // Implement hack logic
        None
    }
}
