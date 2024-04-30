use std::{collections::HashMap, sync::{Arc, Mutex}};
use godot::{engine::{native::ObjectId, physics_server_2d, PhysicsDirectSpaceState2D, ProjectSettings}, prelude::*};
use crate::{bodies::{rapier_area_2d::RapierArea2D, rapier_body_2d::RapierBody2D, rapier_collision_object_2d::{CollisionObjectType, IRapierCollisionObject2D, RapierCollisionObject2D}}, rapier2d::{handle::{invalid_handle, Handle}, physics_hooks::{CollisionFilterInfo, OneWayDirection}, physics_world::{world_create, world_set_active_body_callback, world_set_body_collision_filter_callback, world_set_collision_event_callback, world_set_contact_force_event_callback, world_set_contact_point_callback, world_set_modify_contacts_callback, world_set_sensor_collision_filter_callback, ActiveBodyInfo, CollisionEventInfo, ContactForceEventInfo, ContactPointInfo}, query::QueryExcludedInfo, settings::default_world_settings, user_data::{is_user_data_valid, UserData}}, servers::{rapier_physics_singleton_2d::physics_singleton, rapier_project_settings::RapierProjectSettings}};

use super::rapier_direct_space_state_2d::RapierDirectSpaceState2D;

const TEST_MOTION_MARGIN: real = 0.0001;

struct RemovedColliderInfo {
    rid: Rid,
    instance_id: InstanceId,
    shape_index: u32,
    collision_object_type: CollisionObjectType,
}

struct CollidersInfo {
    shape1: u32,
    object1: Option<RapierCollisionObject2D>,
    shape2: u32,
    object2: Option<RapierCollisionObject2D>,
}

pub struct RapierSpace2D {
    direct_access: Option<Gd<PhysicsDirectSpaceState2D>>,
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
    contact_recycle_radius: real,
    contact_max_separation: real,
    contact_max_allowed_penetration: real,
    contact_bias: real,
    constraint_bias: real,
    fluid_default_gravity_dir: Vector2,
    fluid_default_gravity_value: real,
    default_gravity_dir: Vector2,
    default_gravity_value: real,
    default_linear_damping: real,
    default_angular_damping: real,
    locked: bool,
    last_step: real,
    island_count: i32,
    active_objects: i32,
    collision_pairs: i32,
    contact_debug: PackedVector2Array,
    contact_debug_count: i32,
}

impl RapierSpace2D {
    pub fn new(rid: Rid) -> Self {
        let project_settings = ProjectSettings::singleton();

        let physics_fps = project_settings.get_setting_with_override("physics/common/physics_ticks_per_second".into());
        let mut last_step = 0.001;
        if !physics_fps.is_nil() {
            last_step = 1.0 / (physics_fps.to::<i32>() as f32);
        }

        let solver_iterations = project_settings.get_setting_with_override("physics/2d/solver/solver_iterations".into()).to();
        let contact_recycle_radius = project_settings.get_setting_with_override("physics/2d/solver/contact_recycle_radius".into()).to();
        let contact_max_separation = project_settings.get_setting_with_override("physics/2d/solver/contact_max_separation".into()).to();
        let contact_max_allowed_penetration = project_settings.get_setting_with_override("physics/2d/solver/contact_max_allowed_penetration".into()).to();
        let contact_bias = project_settings.get_setting_with_override("physics/2d/solver/default_contact_bias".into()).to();
        let constraint_bias = project_settings.get_setting_with_override("physics/2d/solver/default_constraint_bias".into()).to();

        let direct_access = RapierDirectSpaceState2D::new_alloc();
        //direct_access.set_space(rid);

        let mut world_settings = default_world_settings();
        world_settings.particle_radius = RapierProjectSettings::get_fluid_particle_radius() as real;
        world_settings.smoothing_factor = RapierProjectSettings::get_fluid_smoothing_factor() as real;
        let handle = world_create(&world_settings);
        assert!(handle.is_valid());

        world_set_active_body_callback(handle, Some(RapierSpace2D::active_body_callback));
        world_set_body_collision_filter_callback(handle, Some(RapierSpace2D::collision_filter_body_callback));
        world_set_sensor_collision_filter_callback(handle, Some(RapierSpace2D::collision_filter_sensor_callback));
        world_set_modify_contacts_callback(handle, Some(RapierSpace2D::collision_modify_contacts_callback));
        world_set_collision_event_callback(handle, Some(RapierSpace2D::collision_event_callback));
        world_set_contact_force_event_callback(handle, Some(RapierSpace2D::contact_force_event_callback));
        world_set_contact_point_callback(handle, Some(RapierSpace2D::contact_point_callback));
        Self {
            direct_access: None,
            rid,
            handle,
            removed_colliders: HashMap::new(),
            active_list: Vec::new(),
            mass_properties_update_list: Vec::new(),
            gravity_update_list: Vec::new(),
            state_query_list: Vec::new(),
            monitor_query_list: Vec::new(),
            area_update_list: Vec::new(),
            body_area_update_list: Vec::new(),
            solver_iterations,
            contact_recycle_radius,
            contact_max_separation,
            contact_max_allowed_penetration,
            contact_bias,
            constraint_bias,
            fluid_default_gravity_dir: Vector2::ZERO,
            fluid_default_gravity_value: 0.0,
            default_gravity_dir: Vector2::ZERO,
            default_gravity_value: 0.0,
            default_linear_damping: 0.0,
            default_angular_damping: 0.0,
            locked: false,
            last_step,
            island_count: 0,
            active_objects: 0,
            collision_pairs: 0,
            contact_debug: PackedVector2Array::new(),
            contact_debug_count: 0,
        }
    }

	fn active_body_callback(world_handle: Handle, active_body_info: &ActiveBodyInfo) {
        let lock = physics_singleton().lock().unwrap();
        let space = lock.active_spaces.get(&world_handle);
        if let Some(space) = space {
            let space = lock.spaces.get(space);
            if let Some(space) = space {
                let (rid, _) = RapierCollisionObject2D::get_collider_user_data(&active_body_info.body_user_data);
                let collision_object = lock.collision_objects.get(&rid);
                if let Some(collision_object) = collision_object {
                    collision_object.on_marked_active();
                }
            }
        }
    }

    fn collision_filter_common_callback(
        world_handle: Handle,
        filter_info: &CollisionFilterInfo,
        r_colliders_info: &mut CollidersInfo,
    ) -> bool {
        let lock = physics_singleton().lock().unwrap();
        let space = lock.active_spaces.get(&world_handle);
        if let Some(space) = space {
            let space = lock.spaces.get(space);
            if let Some(space) = space {

                if (rapier2d::is_user_data_valid(filter_info.user_data1)) {
                    r_colliders_info.object1 = RapierCollisionObject2D::get_collider_user_data(filter_info->user_data1, r_colliders_info.shape1);
                }
            
                if (rapier2d::is_user_data_valid(filter_info->user_data2)) {
                    r_colliders_info.object2 = RapierCollisionObject2D::get_collider_user_data(filter_info->user_data2, r_colliders_info.shape2);
                }
            
                ERR_FAIL_COND_V(!r_colliders_info.object1, false);
                ERR_FAIL_COND_V(!r_colliders_info.object2, false);
            
                if (!r_colliders_info.object1->interacts_with(r_colliders_info.object2)) {
                    return false;
                }
            
                return true;
            }
        }
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

	static Object *_get_object_instance_hack(uint64_t p_object_id) {
		return reinterpret_cast<Object *>((GodotObject *)(internal::gdextension_interface_object_get_instance_from_id(p_object_id)));
	}

	pub fn get_handle(&self) -> Handle { return handle; }

	pub fn set_rid(&mut self, p_rid: Rid) { self.rid = p_rid; }

    fn get_rid(&self) -> Rid {
        self.rid
    }

	pub fn body_add_to_mass_properties_update_list(&mut self, body: Rid) {
    }
	pub fn body_add_to_gravity_update_list(&mut self, body: Rid) {
    }

	pub fn body_add_to_active_list(&mut self, body: Rid) {
    }
    pub fn body_add_to_active_list(&mut self, body: Rid) {
        self.active_list.append(body);
    }
	pub fn body_add_to_state_query_list(body: Rid){

    }

	pub fn area_add_to_monitor_query_list(&mut self, area: Rid) {
    }
	pub fn area_add_to_area_update_list(&mut self, area: Rid) {
    }
	pub fn body_add_to_area_update_list(&mut self, body: Rid) {
    }

	pub fn add_removed_collider(&mut self, handle: Handle, p_object: Rid, p_shape_index: usize){
    }
	pub fn get_removed_collider_info(&mut self, handle: Handle, r_rid: &Rid, r_instance_id: &ObjectId, r_shape_index: &usize, r_type: &CollisionObjectType) -> bool {
    }

	pub fn get_solver_iterations(&self) -> i32 { return self.solver_iterations; }
	pub fn  get_contact_recycle_radius(&self) -> real { return self.contact_recycle_radius; }
	pub fn  get_contact_max_separation(&self)  -> real { return self.contact_max_separation; }
	pub fn  get_contact_max_allowed_penetration(&self)  -> real { return self.contact_max_allowed_penetration; }
	pub fn  get_contact_bias(&self)  -> real { return self.contact_bias; }
	pub fn  get_constraint_bias(&self)  -> real { return self.constraint_bias; }

	pub fn step(&self, step: real){
    }

	pub fn call_queries(&self){
    }

	pub fn is_locked(&self) -> bool {

    }
	pub fn lock(&self) {

    }
	pub fn unlock(&self){

    }

	pub fn get_last_step(&self) -> real { return last_step; }
	pub fn set_last_step(&mut self, step: real) { last_step = p_step; }


    pub fn set_param(&mut self, param: physics_server_2d::SpaceParameter, value: real) {
        match param {
            physics_server_2d::SpaceParameter::SOLVER_ITERATIONS => {
                self.solver_iterations = value as i32;
            }
            _ => {}
        }
    }
    
    pub fn get_param(&self, param: physics_server_2d::SpaceParameter) -> real {
        match param {
            physics_server_2d::SpaceParameter::SOLVER_ITERATIONS => {
                self.solver_iterations as real
            }
            _ => 0.0
        }
    }

	pub fn set_default_area_param(&mut self,param: AreaParameter, value: Variant) {
    }
	pub fn get_default_area_param(&self,param: AreaParameter) -> Variant {
    }

	pub fn get_island_count(&self,) -> i32 { return island_count; }

	pub fn get_active_objects(&self,) -> i32 { return active_objects; }

	pub fn get_collision_pairs(&self,) -> i32 { return collision_pairs; }

    pub fn set_debug_contacts(&mut self, max_contacts: i32) {
        self.contact_debug.resize(p_amount);
    }
	pub fn is_debugging_contacts() -> bool { return !contact_debug.is_empty(); }
	pub fn add_debug_contact(&mut self, contact: Vector2) {
		if (self.contact_debug_count < self.contact_debug.size()) {
			self.contact_debug[self.contact_debug_count] = p_contact;
            self.contact_debug_count+=1;
		}
	}
    pub fn get_debug_contacts(&self) -> PackedVector2Array {
        return self.contact_debug
    }
    pub fn get_debug_contact_count(&self) -> i32 {
        return self.contact_debug_count
    }

    pub fn get_direct_state(&mut self) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        if self.direct_access.is_none() {
            let direct_space_state = RapierDirectSpaceState2D::new_alloc();
            self.direct_access = Some(direct_space_state.upcast());
        }
        self.direct_access.clone()
    }

	pub fn test_body_motion(&self, body: Rid, from: Transform2D, motion: Vector2, margin: double, collide_separation_ray: bool, recovery_as_collision: bool, result: &PhysicsServer2DExtensionMotionResult) -> bool {

    }

	pub fn rapier_intersect_aabb(&self, aabb: Rect2, collision_mask: u32, collide_with_bodies: bool, collide_with_areas: bool, results: &PointHitInfo, max_results: i32, result_count: &i32, exclude_body: Rid) -> i32 {

    }

}
