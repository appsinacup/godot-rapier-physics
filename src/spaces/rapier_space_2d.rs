use crate::bodies;
use crate::bodies::rapier_body_2d::RapierBody2D;
use crate::rapier2d::physics_world::{world_get_active_objects_count, world_step};
use crate::rapier2d::query::{default_query_excluded_info, intersect_aabb, shapes_contact, ContactResult};
use crate::rapier2d::settings::SimulationSettings;
use crate::rapier2d::shape::shape_info_from_body_shape;
use crate::rapier2d::user_data::is_user_data_valid;
use crate::rapier2d::vector::Vector;
use crate::servers::rapier_physics_singleton_2d::{bodies_singleton, spaces_singleton};
use crate::servers::rapier_project_settings::RapierProjectSettings;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::spaces::rapier_direct_space_state_2d::RapierDirectSpaceState2D;
use crate::{
    bodies::rapier_collision_object_2d::{
            CollisionObjectType, RapierCollisionObject2D,
        },
    rapier2d::{
        handle::Handle,
        physics_hooks::{CollisionFilterInfo, OneWayDirection},
        physics_world::{
            world_create, world_set_active_body_callback, world_set_body_collision_filter_callback,
            world_set_collision_event_callback, world_set_contact_force_event_callback,
            world_set_contact_point_callback, world_set_modify_contacts_callback,
            world_set_sensor_collision_filter_callback, ActiveBodyInfo, CollisionEventInfo,
            ContactForceEventInfo, ContactPointInfo,
        },
        query::{PointHitInfo, QueryExcludedInfo},
        settings::default_world_settings,
        user_data::UserData,
    },
    servers::{
    },
};
use godot::engine::physics_server_2d::{AreaParameter, BodyMode};
use godot::{
    engine::{
        native::{ObjectId, PhysicsServer2DExtensionMotionResult},
        physics_server_2d, PhysicsDirectSpaceState2D, ProjectSettings,
    },
    prelude::*,
};
use std::cmp::max;
use std::collections::HashMap;
use std::f32::EPSILON;

const TEST_MOTION_MARGIN: f64 = 1e-4;

const TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR: f32 = 0.05;
const BODY_MOTION_RECOVER_ATTEMPTS: i32 = 4;
const BODY_MOTION_RECOVER_RATIO: f32 = 0.4;

pub struct RemovedColliderInfo {
    pub rid: Rid,
    pub instance_id: InstanceId,
    pub shape_index: u32,
    pub collision_object_type: CollisionObjectType,
}

impl RemovedColliderInfo {
    pub fn new(rid: Rid, instance_id: InstanceId, shape_index: u32, collision_object_type: CollisionObjectType) -> Self{
        Self {
            rid,
            instance_id,
            shape_index,
            collision_object_type,
        }
    }
}

pub struct CollidersInfo {
    pub shape1: usize,
    pub object1: Rid,
    pub shape2: usize,
    pub object2: Rid,
}

impl Default for CollidersInfo {
    fn default() -> Self {
        Self { shape1: 0, object1: Rid::Invalid, shape2: 0, object2: Rid::Invalid }
    }
}

pub struct RapierSpace2D {
    direct_access: Option<Gd<PhysicsDirectSpaceState2D>>,
    rid: Rid,
    pub handle: Handle,
    removed_colliders: HashMap<u32, RemovedColliderInfo>,
    active_list: Vec<Rid>,
    mass_properties_update_list: Vec<Rid>,
    gravity_update_list: Vec<Rid>,
    state_query_list: Vec<Rid>,
    monitor_query_list: Vec<Rid>,
    area_update_list: Vec<Rid>,
    body_area_update_list: Vec<Rid>,
    solver_iterations: i32,
    fluid_default_gravity_dir: Vector2,
    fluid_default_gravity_value: real,
    default_gravity_dir: Vector2,
    default_gravity_value: real,
    default_linear_damping: real,
    default_angular_damping: real,
    pub locked: bool,
    last_step: real,
    island_count: i32,
    active_objects: i32,
    collision_pairs: i32,
    contact_debug: PackedVector2Array,
    contact_debug_count: usize,
}

impl RapierSpace2D {
    pub fn new(rid: Rid) -> Self {
        let project_settings = ProjectSettings::singleton();

        let physics_fps = project_settings
            .get_setting_with_override("physics/common/physics_ticks_per_second".into());
        let mut last_step = 1e-3;
        if !physics_fps.is_nil() {
            last_step = 1.0 / (physics_fps.to::<i32>() as f32);
        }

        let solver_iterations = project_settings
            .get_setting_with_override("physics/2d/solver/solver_iterations".into())
            .to();

        let mut direct_access = RapierDirectSpaceState2D::new_alloc();
        direct_access.set_space(rid);

        let mut world_settings = default_world_settings();
        world_settings.particle_radius = RapierProjectSettings::get_fluid_particle_radius() as real;
        world_settings.smoothing_factor =
            RapierProjectSettings::get_fluid_smoothing_factor() as real;
        let handle = world_create(&world_settings,
            RapierSpace2D::active_body_callback,
            RapierSpace2D::collision_filter_body_callback,
            RapierSpace2D::collision_filter_sensor_callback,
            RapierSpace2D::collision_modify_contacts_callback,
            RapierSpace2D::collision_event_callback,
            RapierSpace2D::contact_force_event_callback,
            RapierSpace2D::contact_point_callback,
        );
        assert!(handle.is_valid());

        Self {
            direct_access: Some(direct_access.upcast()),
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

    fn active_body_callback(active_body_info: &ActiveBodyInfo) {
        let (rid, _) = RapierCollisionObject2D::get_collider_user_data(
            &active_body_info.body_user_data,
        );
        let lock = bodies_singleton().lock().unwrap();
        let collision_object = lock.collision_objects.get(&rid);
        if let Some(collision_object) = collision_object {
            collision_object.on_marked_active();
        }
    }

    fn collision_filter_common_callback(
        filter_info: &CollisionFilterInfo,
        r_colliders_info: &mut CollidersInfo,
    ) -> bool {
        let lock = bodies_singleton().lock().unwrap();
        (r_colliders_info.object1, r_colliders_info.shape1) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data1);
        (r_colliders_info.object2, r_colliders_info.shape2) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data2);
        if let Some(body1) = lock.collision_objects.get(&r_colliders_info.object1) {
            if let Some(body2) = lock.collision_objects.get(&r_colliders_info.object2) {
                return body1.get_base().interacts_with(body2.get_base())
            }
        }
        return false;
    }

    fn collision_filter_body_callback(
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        let colliders_info = CollidersInfo::default();
        if (!collision_filter_common_callback(filter_info, colliders_info)) {
            return false;
        }
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body1) = lock.collision_objects.get(&colliders_info.object1) {
            if let Some(body2) = lock.collision_objects.get(&colliders_info.object2) {
                if (body1.get_base().has_exception(body2.get_rid()) || body2.has_exception(body1.get_rid())) {
                    return false;
                }
            }
        }
    
        return true;
    }

    fn collision_filter_sensor_callback(
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        let colliders_info = CollidersInfo::default();
        collision_filter_common_callback(world_handle, filter_info, colliders_info)
    }

    fn collision_modify_contacts_callback(
        world_handle: Handle,
        filter_info: &CollisionFilterInfo,
    ) -> OneWayDirection {
        // Implement callback logic
        OneWayDirection {
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

    pub fn _is_handle_excluded_callback(
        world_handle: Handle,
        collider_handle: Handle,
        collider: &UserData,
        handle_excluded_info: &QueryExcludedInfo,
    ) -> bool {
    }

    pub fn _get_object_instance_hack(instance_id: u64) -> *mut Gd<Object> {
        let mut object: Gd<Object> = Gd::from_instance_id(InstanceId::from_i64(instance_id as i64));
        let raw_ptr: *mut Gd<Object> = &mut object as *mut _;
        raw_ptr
    }

    pub fn get_handle(&self) -> Handle {
        return self.handle;
    }

    pub fn set_rid(&mut self, p_rid: Rid) {
        self.rid = p_rid;
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn body_add_to_mass_properties_update_list(&mut self, body: Rid) {
        self.mass_properties_update_list.push(body);
    }
    pub fn body_add_to_gravity_update_list(&mut self, body: Rid) {
        self.gravity_update_list.push(body);
    }

    pub fn body_add_to_active_list(&mut self, body: Rid) {
        self.active_list.push(body);
    }
    pub fn body_add_to_state_query_list(&mut self, body: Rid) {
        self.state_query_list.push(body);
    }

    pub fn area_add_to_monitor_query_list(&mut self, area: Rid) {
        self.monitor_query_list.push(area);
    }
    pub fn area_add_to_area_update_list(&mut self, area: Rid) {
        self.area_update_list.push(area);
    }
    pub fn body_add_to_area_update_list(&mut self, body: Rid) {
        self.body_area_update_list.push(body);
    }

    pub fn add_removed_collider(&mut self, handle: Handle, object: Rid, instance_id: InstanceId, shape_index: usize, collision_object_type: CollisionObjectType) {
        self.removed_colliders.insert(handle, RemovedColliderInfo::new(object, instance_id, shape_index, collision_object_type))
    }
    pub fn get_removed_collider_info(
        &mut self,
        handle: Handle,
    ) -> Option<RemovedColliderInfo> {
        self.removed_colliders.get(handle)
    }

    pub fn get_solver_iterations(&self) -> i32 {
        return self.solver_iterations;
    }

    pub fn step(&mut self, step: real) {
        self.last_step = step;
        for body in self.active_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&body) {
                if let Some(body) = body.get_body() {
                    body.reset_contact_count();
                }
            }
        }
        self.contact_debug_count = 0;
        
        let project_settings = ProjectSettings::singleton();
    
        let default_gravity_dir = project_settings.get_setting_with_override("physics/2d/default_gravity_vector".into());
        let default_gravity_value = project_settings.get_setting_with_override("physics/2d/default_gravity".into());
    
        let fluid_default_gravity_dir = RapierProjectSettings::get_fluid_gravity_dir();
        let fluid_default_gravity_value = RapierProjectSettings::get_fluid_gravity_value();
    
        let default_linear_damping = project_settings.get_setting_with_override("physics/2d/default_linear_damp".into());
        let default_angular_damping = project_settings.get_setting_with_override("physics/2d/default_angular_damp".into());
        
        for body in self.mass_properties_update_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&body) {
                if let Some(body) = body.get_body() {
                    body.update_mass_properties(false);
                }
            }
        }
        for area in self.area_update_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(area) = lock.collision_objects.get(&area) {
                if let Some(area) = area.get_area() {
                    area.update_area_override();
                }
            }
        }
        for body in self.body_area_update_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&body) {
                if let Some(body) = body.get_body() {
                    body.update_area_override();
                }
            }
        }
        for body in self.gravity_update_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&body) {
                if let Some(body) = body.get_body() {
                    body.update_gravity(step);
                }
            }
        }
    
        let settings = SimulationSettings{
            pixel_liquid_gravity: Vector::new(fluid_default_gravity_dir.x * fluid_default_gravity_value as real,fluid_default_gravity_dir.y * fluid_default_gravity_value as real),
            dt: step,
            pixel_gravity: Vector::new(default_gravity_dir.x * default_gravity_value.to(),default_gravity_dir.y * default_gravity_value.to()),
            max_ccd_substeps: RapierProjectSettings::get_solver_max_ccd_substeps() as usize,
            num_additional_friction_iterations: RapierProjectSettings::get_solver_num_additional_friction_iterations() as usize,
            num_internal_pgs_iterations: RapierProjectSettings::get_solver_num_internal_pgs_iterations() as usize,
            num_solver_iterations: RapierProjectSettings::get_solver_num_solver_iterations() as usize,
        };
    
        world_step(self.handle, &settings);
    
        // Needed only for one physics step to retrieve lost info
        self.removed_colliders.clear();
    
        for body in self.gravity_update_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(&body) {
                if let Some(body) = body.get_body() {
                    body.on_update_active();
                }
            }
        }
        self.active_objects = world_get_active_objects_count(self.handle);
    }

    pub fn call_queries(&mut self) {
        for body_rid in self.state_query_list {
            let lock = bodies_singleton().lock().unwrap();
            let body = lock.collision_objects.get(&body_rid);
            if let Some(body) = body {
                if let Some(body) = body.get_body() {
                    body.call_queries();
                }
            }
        }
        for area_rid in self.monitor_query_list {
            let lock = bodies_singleton().lock().unwrap();
            let area = lock.collision_objects.get(&area_rid);
            if let Some(area) = area {
                if let Some(area) = area.get_area() {
                    area.call_queries();
                }
            }
        }
    }

    pub fn is_locked(&self) -> bool {
        self.locked
    }
    pub fn lock(&mut self) {
        self.locked = true;
    }
    pub fn unlock(&mut self) {
        self.locked = false;
    }

    pub fn get_last_step(&self) -> real {
        return self.last_step;
    }

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
            physics_server_2d::SpaceParameter::SOLVER_ITERATIONS => self.solver_iterations as real,
            _ => 0.0,
        }
    }

    pub fn set_default_area_param(&mut self, param: AreaParameter, value: Variant) {
        match param {
            AreaParameter::GRAVITY => self.default_gravity_value = value.to(),
            AreaParameter::GRAVITY_VECTOR => self.default_gravity_dir = value.to(),
            AreaParameter::LINEAR_DAMP => self.default_linear_damping = value.to(),
            AreaParameter::ANGULAR_DAMP => self.default_angular_damping = value.to(),
            _ => {}
        }
    }
    pub fn get_default_area_param(&self, param: AreaParameter) -> Variant {
        match param {
            AreaParameter::GRAVITY => return self.default_gravity_value.to_variant(),
            AreaParameter::GRAVITY_VECTOR => return self.default_gravity_dir.to_variant(),
            AreaParameter::LINEAR_DAMP => return self.default_linear_damping.to_variant(),
            AreaParameter::ANGULAR_DAMP => return self.default_angular_damping.to_variant(),
            _ => (0.0).to_variant()
        }
    }

    pub fn get_island_count(&self) -> i32 {
        return self.island_count;
    }

    pub fn get_active_objects(&self) -> i32 {
        return self.active_objects;
    }

    pub fn get_collision_pairs(&self) -> i32 {
        return self.collision_pairs;
    }

    pub fn set_debug_contacts(&mut self, max_contacts: i32) {
        self.contact_debug.resize(max_contacts as usize);
    }
    pub fn is_debugging_contacts(&self) -> bool {
        return !self.contact_debug.is_empty();
    }
    pub fn add_debug_contact(&mut self, contact: Vector2) {
        if (self.contact_debug_count < self.contact_debug.len()) {
            self.contact_debug[self.contact_debug_count] = contact;
            self.contact_debug_count += 1;
        }
    }
    pub fn get_debug_contacts(&self) -> PackedVector2Array {
        return self.contact_debug.clone();
    }
    pub fn get_debug_contact_count(&self) -> i32 {
        self.contact_debug_count
    }

    pub fn get_direct_state(&self) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        self.direct_access
    }

    pub fn test_body_motion(
        &self,
        body: Rid,
        from: Transform2D,
        motion: Vector2,
        margin: f64,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: &PhysicsServer2DExtensionMotionResult,
    ) -> bool {
        result.travel = Vector2::default();
	    let body_transform = from.clone(); // Because body_transform needs to be modified during recovery
        // Step 1: recover motion.
        // Expand the body colliders by the margin (grow) and check if now it collides with a collider,
        // if yes, "recover" / "push" out of this collider
	    let recover_motion;
        let margin = max(margin, TEST_MOTION_MARGIN);

	    let recovered = false;//RapierBodyUtils2D::body_motion_recover(*this, *p_body, body_transform, p_motion, margin, recover_motion);
        // Step 2: Cast motion.
        // Try to to find what is the possible motion (how far it can move, it's a shapecast, when you try to find the safe point (max you can move without collision ))
        let best_safe = 1.0;
        let best_unsafe = 1.0;
        let best_body_shape = -1;
	    //RapierBodyUtils2D::cast_motion(*this, *p_body, body_transform, p_motion, p_collide_separation_ray, contact_max_allowed_penetration, margin, best_safe, best_unsafe, best_body_shape);

        // Step 3: Rest Info
        // Apply the motion and fill the collision information
        let mut collided = false;
        if ((recovery_as_collision && recovered) || (best_safe < 1.0)) {
            if (best_safe >= 1.0) {
                best_body_shape = -1; //no best shape with cast, reset to -1
            }

            // Get the rest info in unsafe advance
            let unsafe_motion = motion * best_unsafe;
            body_transform.columns[2] += unsafe_motion;

            collided = false;// RapierBodyUtils2D::body_motion_collide(*this, *p_body, body_transform, p_motion, best_body_shape, margin, r_result);
        }

		if (collided) {
			result.travel += recover_motion + motion * best_safe;
			result.remainder = motion - motion * best_safe;
			result.collision_safe_fraction = best_safe;
			result.collision_unsafe_fraction = best_unsafe;
		} else {
			result.travel += recover_motion + motion;
			result.remainder = Vector2::default();
			result.collision_depth = 0.0;
			result.collision_safe_fraction = 1.0;
			result.collision_unsafe_fraction = 1.0;
		}

        collided
    }

    pub fn rapier_intersect_aabb(
        &self,
        aabb: Rect2,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: &PointHitInfo,
        max_results: i32,
        exclude_body: Rid,
    ) -> i32 {
        let max_results = max_results as usize;
        if max_results < 1 {
            return 0;
        }

        let rect_begin = Vector::new( aabb.position.x, aabb.position.y );
        let rect_end = Vector::new( aabb.end().x, aabb.end().y );
        let handle_excluded_info = default_query_excluded_info();
        let mut query_exclude: Vec<Handle> = Vec::with_capacity(max_results);
        handle_excluded_info.query_exclude = query_exclude.as_mut_ptr();
        handle_excluded_info.query_collision_layer_mask = collision_mask;
        handle_excluded_info.query_exclude_size = 0;
        handle_excluded_info.query_exclude_body = exclude_body.to_u64() as i64;
    
        return intersect_aabb(self.handle, &rect_begin, &rect_end, collide_with_bodies, collide_with_areas, results, max_results, RapierSpace2D::_is_handle_excluded_callback, &handle_excluded_info) as i32;
    
    }


fn body_motion_recover(
    &mut self,
    p_body: &mut RapierBody2D,
    p_transform: &mut Transform2D,
    p_motion: Vector2,
    p_margin: f32,
    p_recover_motion: &mut Vector2,
) -> bool {
    let shape_count = p_body.get_shape_count();
    if shape_count < 1 {
        return false;
    }
    let min_contact_depth = p_margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;

    let mut recovered = false;
    let mut recover_attempts = BODY_MOTION_RECOVER_ATTEMPTS;
    loop {
        let mut results = [PointHitInfo::default(); 32];

        let body_aabb = p_body.get_aabb();
        // Undo the currently transform the physics server is aware of and apply the provided one
        let margin_aabb = p_transform.xform(body_aabb);
        let margin_aabb = margin_aabb.grow(p_margin);
        
        let result_count = self.rapier_intersect_aabb(
            margin_aabb,
            p_body.get_collision_mask(),
            true,
            false,
            &mut results,
            32,
            p_body.get_rid(),
        );
        // Optimization
        if result_count == 0 {
            break;
        }

        let mut recover_step = Vector2::default();

        for body_shape_idx in 0..p_body.get_shape_count() {
            if p_body.is_shape_disabled(body_shape_idx) {
                continue;
            }

            let body_shape = p_body.get_shape(body_shape_idx);
            let body_shape_transform = *p_transform * p_body.get_shape_transform(body_shape_idx);
            let body_shape_info =
                shape_info_from_body_shape(body_shape.get_rapier_shape(), body_shape_transform);

            for result_idx in 0..result_count {
                let result = &mut results[result_idx];
                if !result.user_data.is_valid() {
                    continue;
                }
                let (shape_index, shape_col_object) =
                    RapierCollisionObject2D::get_collider_user_data(result.user_data);
                if shape_col_object.is_none() {
                    continue;
                }
                let shape_col_object = shape_col_object.unwrap();
                if shape_col_object.get_type() != RapierCollisionObject2D::TYPE_BODY {
                    continue;
                }
                let collision_body = shape_col_object.downcast_ref::<RapierBody2D>().unwrap();

                let col_shape = collision_body.get_shape(shape_index).unwrap();

                let col_shape_transform =
                    collision_body.get_transform() * collision_body.get_shape_transform(shape_index);
                let col_shape_info =
                    shape_info_from_body_shape(col_shape.get_rapier_shape(), col_shape_transform);

                let contact = shapes_contact(body_shape_info, col_shape_info, p_margin);

                if !contact.collided {
                    continue;
                }
                if should_skip_collision_one_dir(
                    contact,
                    body_shape,
                    collision_body,
                    shape_index,
                    &col_shape_transform,
                    p_margin,
                    self.get_last_step(),
                    p_motion,
                ) {
                    continue;
                }
                let a = Vector2::new(contact.pixel_point1.x, contact.pixel_point1.y);
                let b = Vector2::new(contact.pixel_point2.x, contact.pixel_point2.y);

                recovered = true;

                // Compute plane on b towards a.
                let n = Vector2::new(contact.normal1.x, contact.normal1.y);
                // Move it outside as to fit the margin
                let d = n.dot(b);

                // Compute depth on recovered motion.
                let depth = n.dot(a + recover_step) - d;
                if depth > min_contact_depth + EPSILON {
                    // Only recover if there is penetration.
                    recover_step -= n * (depth - min_contact_depth) * BODY_MOTION_RECOVER_RATIO;
                }
            }
        }
        if recover_step == Vector2::default() {
            recovered = false;
            break;
        }
        if recovered {
            *p_recover_motion += recover_step;
            p_transform.columns[2] += recover_step;
        }
        recover_attempts -= 1;
        if recover_attempts == 0 {
            break;
        }
    }

    recovered
}

fn cast_motion(
    &mut self,
    p_body: &mut RapierBody2D,
    p_transform: &Transform2D,
    p_motion: Vector2,
    p_collide_separation_ray: bool,
    contact_max_allowed_penetration: f32,
    p_margin: f32,
    p_closest_safe: &mut f32,
    p_closest_unsafe: &mut f32,
    p_best_body_shape: &mut i32,
) {
    let body_aabb = p_body.get_aabb();
    let margin_aabb = p_transform.xform(body_aabb);

    let margin_aabb = margin_aabb.grow(p_margin);
    let mut motion_aabb = margin_aabb;
    motion_aabb.position += p_motion;
    motion_aabb = motion_aabb.merge(margin_aabb);

    let mut results = [PointHitInfo::default(); 32];
    let result_count = self.rapier_intersect_aabb(
        motion_aabb,
        p_body.get_base().get_collision_mask(),
        true,
        false,
        &mut results,
        32,
        p_body.get_base().get_rid(),
    );

    if result_count == 0 {
        return;
    }

    for body_shape_idx in 0..p_body.get_shape_count() {
        if p_body.is_shape_disabled(body_shape_idx) {
            continue;
        }

        let body_shape = p_body.get_shape(body_shape_idx);
        let body_shape_transform = *p_transform * p_body.get_shape_transform(body_shape_idx);
        let body_shape_info =
            shape_info_from_body_shape(body_shape.get_rapier_shape(), body_shape_transform);

        // Colliding separation rays allows to properly snap to the ground,
        // otherwise it's not needed in regular motion.
        //if !p_collide_separation_ray
        //    && body_shape.get_type() == PhysicsServer2D::SHAPE_SEPARATION_RAY
        //{
            // When slide on slope is on, separation ray shape acts like a
            // regular shape.
            //if !body_shape.downcast_ref::<RapierSeparationRayShape2D>().unwrap().get_slide_on_slope()
            //{
            //    continue;
            //}
        //}

        let mut stuck = false;
        let mut best_safe = 1.0;
        let mut best_unsafe = 1.0;

        for result_idx in 0..result_count {
            let result = &mut results[result_idx];

            if !result.user_data.is_valid() {
                continue;
            }
            let (shape_index, shape_col_object) =
                RapierCollisionObject2D::get_collider_user_data(result.user_data);
            if shape_col_object.is_none() {
                continue;
            }
            let shape_col_object = shape_col_object.unwrap();
            if shape_col_object.get_type() != RapierCollisionObject2D::TYPE_BODY {
                continue;
            }
            let collision_body = shape_col_object.downcast_ref::<RapierBody2D>().unwrap();

            let col_shape = collision_body.get_shape(shape_index).unwrap();

            let col_shape_transform =
                collision_body.get_transform() * collision_body.get_shape_transform(shape_index);
            let col_shape_info =
                shape_info_from_body_shape(col_shape.get_rapier_shape(), col_shape_transform);

            let contact = shapes_contact(body_shape_info, col_shape_info, p_margin);

            if !contact.collided {
                continue;
            }
            if should_skip_collision_one_dir(
                contact,
                body_shape,
                collision_body,
                shape_index,
                &col_shape_transform,
                p_margin,
                self.get_last_step(),
                p_motion,
            ) {
                continue;
            }
            let a = Vector2::new(contact.pixel_point1.x, contact.pixel_point1.y);
            let b = Vector2::new(contact.pixel_point2.x, contact.pixel_point2.y);

            stuck = true;

            // Compute plane on b towards a.
            let n = Vector2::new(contact.normal1.x, contact.normal1.y);
            // Move it outside as to fit the margin
            let d = n.dot(b);

            // Compute depth on recovered motion.
            let depth = n.dot(a + p_motion) - d;
            if depth > EPSILON {
                // Only recover if there is penetration.
                best_safe = best_safe.min(0.0);
                best_unsafe = best_unsafe.min(1.0);
            }
        }
        if best_safe == 1.0 {
            continue;
        }
        if best_safe < *p_closest_safe {
            *p_closest_safe = best_safe;
            *p_closest_unsafe = best_unsafe;
            *p_best_body_shape = body_shape_idx as i32;
        }
    }
}

fn body_motion_collide(
    &mut self,
    p_body: &mut RapierBody2D,
    p_transform: &Transform2D,
    p_motion: Vector2,
    p_best_body_shape: i32,
    p_margin: f32,
    p_result: Option<&mut PhysicsServer2DExtensionMotionResult>,
) -> bool {
    let shape_count = p_body.get_shape_count();
    if shape_count < 1 {
        return false;
    }
    let body_aabb = p_body.get_aabb();
    let margin_aabb = p_transform.xform(body_aabb);
    let margin_aabb = margin_aabb.grow(p_margin);

    // also check things at motion
    let mut motion_aabb = margin_aabb;
    motion_aabb.position += p_motion;
    motion_aabb = motion_aabb.merge(margin_aabb);

    let mut results = [PointHitInfo::default(); 32];
    let result_count = self.rapier_intersect_aabb(
        motion_aabb,
        p_body.get_collision_mask(),
        true,
        false,
        &mut results,
        32,
        p_body.get_rid(),
    );
    // Optimization
    if result_count == 0 {
        return false;
    }

    let mut min_distance = f32::INFINITY;
    let mut best_collision_body = None;
    let mut best_collision_shape_index = -1;
    let mut best_body_shape_index = -1;
    let mut best_contact = ContactResult::default();

    let from_shape = if p_best_body_shape != -1 {
        p_best_body_shape
    } else {
        0
    };
    let to_shape = if p_best_body_shape != -1 {
        p_best_body_shape + 1
    } else {
        p_body.get_shape_count()
    };
    for body_shape_idx in from_shape..to_shape {
        if p_body.is_shape_disabled(body_shape_idx) {
            continue;
        }

        let body_shape = p_body.get_shape(body_shape_idx);
        let body_shape_transform = *p_transform * p_body.get_shape_transform(body_shape_idx);
        let body_shape_info =
            shape_info_from_body_shape(body_shape.get_rapier_shape(), body_shape_transform);

        for result_idx in 0..result_count {
            let result = &mut results[result_idx];

            if !is_user_data_valid(result.user_data) {
                continue;
            }
            let (shape_index, shape_col_object) =
                RapierCollisionObject2D::get_collider_user_data(result.user_data);
            if shape_col_object.is_none() {
                continue;
            }
            let shape_col_object = shape_col_object.unwrap();
            if shape_col_object.get_type() != RapierCollisionObject2D::TYPE_BODY {
                continue;
            }
            let collision_body = shape_col_object.downcast_ref::<RapierBody2D>().unwrap();

            let col_shape = collision_body.get_shape(shape_index).unwrap();

            let col_shape_transform =
                collision_body.get_transform() * collision_body.get_shape_transform(shape_index);
            let col_shape_info =
                shape_info_from_body_shape(col_shape.get_rapier_shape(), col_shape_transform);

            let contact = shapes_contact(body_shape_info, col_shape_info, p_margin);
            if !contact.collided {
                continue;
            }

            let a = Vector2::new(contact.pixel_point1.x, contact.pixel_point1.y);
            let b = Vector2::new(contact.pixel_point2.x, contact.pixel_point2.y);

            if should_skip_collision_one_dir(
                contact,
                body_shape,
                collision_body,
                shape_index,
                &col_shape_transform,
                p_margin,
                self.get_last_step(),
                p_motion,
            ) {
                continue;
            }
            if contact.pixel_distance < min_distance {
                min_distance = contact.pixel_distance;
                best_collision_body = Some(collision_body);
                best_collision_shape_index = shape_index;
                best_body_shape_index = body_shape_idx;
                best_contact = contact;
            }
        }
    }
    if let Some(best_collision_body) = best_collision_body {
        // conveyer belt
        if best_collision_body.get_static_linear_velocity() != Vector2::default() {
            p_result.travel += best_collision_body.get_static_linear_velocity() * self.get_last_step();
        }
        if let Some(p_result) = p_result {
            p_result.collider = best_collision_body.get_rid();
            p_result.collider_id = best_collision_body.get_instance_id();
            p_result.collider_shape = best_collision_shape_index;
            p_result.collision_local_shape = best_body_shape_index;
            // World position from the moving body to get the contact point
            p_result.collision_point = Vector2::new(best_contact.pixel_point1.x, best_contact.pixel_point1.y);
            // Normal from the collided object to get the contact normal
            p_result.collision_normal = Vector2::new(best_contact.normal2.x, best_contact.normal2.y);
            // compute distance without sign
            p_result.collision_depth = p_margin - best_contact.pixel_distance;

            let local_position = p_result.collision_point - best_collision_body.get_transform().get_origin();
            p_result.collider_velocity = best_collision_body.get_velocity_at_local_point(local_position);
        }

        return true;
    }

    false
}

}


fn should_skip_collision_one_dir(
    contact: ContactResult,
    body_shape: &IRapierShape2D,
    collision_body: &RapierBody2D,
    shape_index: usize,
    col_shape_transform: &Transform2D,
    p_margin: f32,
    last_step: f32,
    p_motion: Vector2,
) -> bool {
    let dist = contact.pixel_distance;
    if !contact.within_margin
        && body_shape.allows_one_way_collision()
        && collision_body.is_shape_set_as_one_way_collision(shape_index)
    {
        let mut valid_depth = 10e20;
        let valid_dir = col_shape_transform.y.normalize();

        let owc_margin = collision_body.get_shape_one_way_collision_margin(shape_index);
        valid_depth = valid_depth.max(owc_margin).max(p_margin);

        if collision_body.get_type() == RapierCollisionObject2D::TYPE_BODY {
            let b = collision_body;
            if b.get_mode() >= BodyMode::KINEMATIC {
                // fix for moving platforms (kinematic and dynamic), margin is increased by how much it moved in the
                // given direction
                let lv = b.get_linear_velocity();
                // compute displacement from linear velocity
                let motion = lv * last_step;
                let motion_len = motion.length();
                let motion = motion.normalize();
                valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
            }
        }
        let motion = p_motion;
        let motion_len = motion.length();
        let motion = motion.normalize();
        valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
        if dist < -valid_depth || p_motion.normalize().dot(valid_dir) < EPSILON {
            return true;
        }
    }
    false
}
