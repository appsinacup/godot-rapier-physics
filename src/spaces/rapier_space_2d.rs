use crate::bodies;
use crate::bodies::rapier_body_2d::RapierBody2D;
use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::rapier2d::physics_world::{world_get_active_objects_count, world_step};
use crate::rapier2d::query::{default_query_excluded_info, intersect_aabb, shapes_contact, ContactResult};
use crate::rapier2d::settings::SimulationSettings;
use crate::rapier2d::shape::shape_info_from_body_shape;
use crate::rapier2d::user_data::is_user_data_valid;
use crate::rapier2d::vector::Vector;
use crate::servers::rapier_physics_singleton_2d::{active_spaces_singleton, bodies_singleton, shapes_singleton, spaces_singleton};
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
            world_create, ActiveBodyInfo, CollisionEventInfo,
            ContactForceEventInfo, ContactPointInfo,
        },
        query::{PointHitInfo, QueryExcludedInfo},
        settings::default_world_settings,
        user_data::UserData,
    },
    servers::{
    },
};
use godot::engine::{collision_object_2d, IPhysicsDirectBodyState2DExtension, PhysicsDirectBodyState2DExtension};
use godot::engine::physics_server_2d::{AreaParameter, BodyMode};
use godot::{
    engine::{
        native::{ObjectId, PhysicsServer2DExtensionMotionResult},
        physics_server_2d, PhysicsDirectSpaceState2D, ProjectSettings,
    },
    prelude::*,
};
use std::cmp::{self, max};
use std::collections::HashMap;
use std::f32::EPSILON;
use std::mem::swap;

const TEST_MOTION_MARGIN: f64 = 1e-4;

const TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR: f32 = 0.05;
const BODY_MOTION_RECOVER_ATTEMPTS: i32 = 4;
const BODY_MOTION_RECOVER_RATIO: f32 = 0.4;

pub struct RemovedColliderInfo {
    pub rid: Rid,
    pub instance_id: u64,
    pub shape_index: usize,
    pub collision_object_type: CollisionObjectType,
}

impl RemovedColliderInfo {
    pub fn new(rid: Rid, instance_id: u64, shape_index: usize, collision_object_type: CollisionObjectType) -> Self{
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
    removed_colliders: HashMap<Handle, RemovedColliderInfo>,
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
        if let Some(body) = lock.collision_objects.get(&rid) {
            if let Some(body) = body.get_body() {
                body.on_marked_active();
            }
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
        let mut colliders_info = CollidersInfo::default();
        if !Self::collision_filter_common_callback(filter_info, &mut colliders_info) {
            return false;
        }
        let lock = bodies_singleton().lock().unwrap();
        if let Some(body1) = lock.collision_objects.get(&colliders_info.object1) {
            if let Some(body1) = body1.get_body() {
                if let Some(body2) = lock.collision_objects.get(&colliders_info.object2) {
                    if let Some(body2) = body2.get_body() {
                        if body1.has_exception(body2.get_base().get_rid()) || body2.has_exception(body1.get_base().get_rid()) {
                            return false;
                        }
                    }
            }
        }
        }
    
        return true;
    }

    fn collision_filter_sensor_callback(
        filter_info: &CollisionFilterInfo,
    ) -> bool {
        let mut colliders_info = CollidersInfo::default();
        Self::collision_filter_common_callback(filter_info, &mut colliders_info)
    }

    fn collision_modify_contacts_callback(
        filter_info: &CollisionFilterInfo,
    ) -> OneWayDirection {
        let mut result = OneWayDirection::default();
        
        let mut lock = bodies_singleton().lock().unwrap();
        let (object1, shape1) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data1);
        let (object2, shape2) =
            RapierCollisionObject2D::get_collider_user_data(&filter_info.user_data2);
        if !lock.collision_objects.contains_key(&object1) || !lock.collision_objects.contains_key(&object2) {
            return result
        }
        let [collision_object_1, collision_object_2] = lock.collision_objects.get_many_mut([&object1, &object2]).unwrap();
        if collision_object_1.get_base().interacts_with(collision_object_2.get_base()) {
            if !collision_object_1.get_base().is_shape_disabled(shape1)
                && !collision_object_2.get_base().is_shape_disabled(shape2)
            {
                result.body1 = collision_object_1.get_base().is_shape_set_as_one_way_collision(shape1);
                result.pixel_body1_margin =
                    collision_object_1.get_base().get_shape_one_way_collision_margin(shape1);
                result.body2 = collision_object_2.get_base().is_shape_set_as_one_way_collision(shape2);
                result.pixel_body2_margin =
                    collision_object_2.get_base().get_shape_one_way_collision_margin(shape2);
                if let Some(body1) = collision_object_1.get_mut_body() {
                    if let Some(body2) = collision_object_2.get_mut_body() {
                        let static_linear_velocity1 = body1.get_static_linear_velocity();
                        let static_linear_velocity2 = body2.get_static_linear_velocity();
                        if static_linear_velocity1 != Vector2::default() {
                            body2.to_add_static_constant_linear_velocity(static_linear_velocity1);
                        }
                        if static_linear_velocity2 != Vector2::default() {
                            body1.to_add_static_constant_linear_velocity(static_linear_velocity2);
                        }
                        let static_angular_velocity1 = body1.get_static_angular_velocity();
                        let static_angular_velocity2 = body2.get_static_angular_velocity();
                        if static_angular_velocity1 != 0.0 {
                            body2.to_add_static_constant_angular_velocity(static_angular_velocity1);
                        }
                        if static_angular_velocity2 != 0.0 {
                            body1.to_add_static_constant_angular_velocity(static_angular_velocity2);
                        }
                    }
                }
            }
        }

        result
    }

    fn collision_event_callback(world_handle: Handle, event_info: &CollisionEventInfo) {
        let mut spaces_lock = spaces_singleton().lock().unwrap();
        let active_spaces_lock = active_spaces_singleton().lock().unwrap();
        if let Some(space) = active_spaces_lock.active_spaces.get(&world_handle) {
            if let Some(space) = spaces_lock.spaces.get_mut(space) {
                let (mut pObject1, mut shape1) = RapierCollisionObject2D::get_collider_user_data(
                    &event_info.user_data1,
                );
                let (mut pObject2, mut shape2) = RapierCollisionObject2D::get_collider_user_data(
                    &event_info.user_data2,
                );
    
                let mut collider_handle1 = event_info.collider1;
                let mut collider_handle2 = event_info.collider2;

                let (mut rid1, mut rid2) = (Rid::Invalid, Rid::Invalid);
                let (mut instance_id1, mut instance_id2) = (0, 0);
                let (mut type1, mut type2) = (CollisionObjectType::Area, CollisionObjectType::Area);
                

                if event_info.is_removed {
                    if pObject1.is_invalid() {
                        if let Some(removed_collider_info_1) = space.get_removed_collider_info(
                            &collider_handle1,
                        ) {
                            rid1 = removed_collider_info_1.rid;
                            instance_id1 = removed_collider_info_1.instance_id;
                            type1 = removed_collider_info_1.collision_object_type;
                            shape1 = removed_collider_info_1.shape_index;
                        }
                    } else {
                        let body_lock = bodies_singleton().lock().unwrap();
                        if let Some(body) = body_lock.collision_objects.get(&pObject1) {
                            rid1 = body.get_base().get_rid();
                            instance_id1 = body.get_base().get_instance_id();
                            type1 = body.get_base().get_type();
                        }
                    }
                    if pObject2.is_invalid() {
                        if let Some(removed_collider_info_2) = space.get_removed_collider_info(
                            &collider_handle2,
                        ) {
                            rid2 = removed_collider_info_2.rid;
                            instance_id2 = removed_collider_info_2.instance_id;
                            type2 = removed_collider_info_2.collision_object_type;
                            shape2 = removed_collider_info_2.shape_index;
                        }
                    } else {
                        let body_lock = bodies_singleton().lock().unwrap();
                        if let Some(body) = body_lock.collision_objects.get(&pObject2) {
                            rid2 = body.get_base().get_rid();
                            instance_id2 = body.get_base().get_instance_id();
                            type2 = body.get_base().get_type();
                        }
                    }
                } else {
                    let body_lock = bodies_singleton().lock().unwrap();
                    if let Some(body) = body_lock.collision_objects.get(&pObject1) {
                        rid1 = body.get_base().get_rid();
                        instance_id1 = body.get_base().get_instance_id();
                        type1 = body.get_base().get_type();
                    }
                    if let Some(body) = body_lock.collision_objects.get(&pObject2) {
                        rid2 = body.get_base().get_rid();
                        instance_id2 = body.get_base().get_instance_id();
                        type2 = body.get_base().get_type();
                    }
                }

        if event_info.is_sensor {
            if instance_id1 == 0 {
                godot_error!("Should be able to get info about a removed object if the other one is still valid.");
                return;
            }
            if instance_id2 == 0 {
                godot_error!( "Should be able to get info about a removed object if the other one is still valid.");
                return;
            }
    
            if type1 != CollisionObjectType::Area {
                if type2 != CollisionObjectType::Area {
                    godot_error!("Expected Area.");
                    return;
                }
                swap(&mut pObject1, &mut pObject2);
                swap(&mut type1, &mut type2);
                swap(&mut shape1, &mut shape2);
                swap(&mut collider_handle1, &mut collider_handle2);
                swap(&mut rid1, &mut rid2);
                swap(&mut instance_id1, &mut instance_id2);
            }
            
            let mut body_lock = bodies_singleton().lock().unwrap();
            let mut p_area = body_lock.collision_objects.get_mut(&pObject1).unwrap().get_mut_area();
            if type2 == CollisionObjectType::Area {
                let p_area2 = body_lock.collision_objects.get_mut(&pObject2).unwrap().get_mut_area();
                if event_info.is_started {
                    let mut p_area = p_area.unwrap();
                    p_area.on_area_enter(
                        collider_handle2,
                        p_area2,
                        shape2,
                        rid2,
                        instance_id2,
                        collider_handle1,
                        shape1,
                    );
                    p_area2.unwrap().on_area_enter(
                        collider_handle1,
                        Some(p_area),
                        shape1,
                        rid1,
                        instance_id1,
                        collider_handle2,
                        shape2,
                    );
                } else if event_info.is_stopped {
                    if let Some(pArea) = p_area {
                        pArea.on_area_exit(
                            collider_handle2,
                            p_area2,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                        );
                    } else {
                        // Try to retrieve area if not destroyed yet
                        let p_area = body_lock.collision_objects.get(&rid1).unwrap().get_mut_area();
                        if let Some(p_area) = p_area {
                            // Use invalid area case to keep counters consistent for already removed collider
                            p_area.on_area_exit(
                                collider_handle2,
                                None,
                                shape2,
                                rid2,
                                instance_id2,
                                collider_handle1,
                                shape1,
                            );
                        }
                    }
                    if let Some(p_area_2) = p_area2 {
                        p_area_2.on_area_exit(
                            collider_handle1,
                            p_area,
                            shape1,
                            rid1,
                            instance_id1,
                            collider_handle2,
                            shape2,
                        );
                    } else {
                        // Try to retrieve area if not destroyed yet
                        let p_area2 = body_lock.collision_objects.get_mut(&rid2).unwrap().get_mut_area();
                        if let Some(p_area_2) = p_area2 {
                            // Use invalid area case to keep counters consistent for already removed collider
                            p_area_2.on_area_exit(
                                collider_handle1,
                                None,
                                shape1,
                                rid1,
                                instance_id1,
                                collider_handle2,
                                shape2,
                            );
                        }
                    }
                }
            } else {
                let p_body = body_lock.collision_objects.get_mut(&pObject2).unwrap().get_mut_body();
                if event_info.is_started {
                    if !p_area.is_some() {
                        godot_error!("Should be able to get info about a removed object if the other one is still valid.");
                        return;
                    }
                    p_area.unwrap().on_body_enter(
                        collider_handle2,
                        p_body,
                        shape2,
                        rid2,
                        instance_id2,
                        collider_handle1,
                        shape1,
                    );
                } else if let Some(p_area) = p_area {
                    if event_info.is_stopped {
                        p_area.on_body_exit(
                            collider_handle2,
                            p_body,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                            true,
                        );
                    }
                } else if event_info.is_stopped {
                    // Try to retrieve area if not destroyed yet
                    let p_area = body_lock.collision_objects.get_mut(&rid1).unwrap().get_mut_area();
                    if let Some(p_area) = p_area {
                        // Use invalid body case to keep counters consistent for already removed collider
                        p_area.on_body_exit(
                            collider_handle2,
                            None,
                            shape2,
                            rid2,
                            instance_id2,
                            collider_handle1,
                            shape1,
                            false,
                        );
                    }
                }
            }
        } else {
            // Body contacts use contact_force_event_callback instead
            godot_error!("Shouldn't receive rigidbody collision events.");
        }
            }
        }
    }

    fn contact_force_event_callback(
        world_handle: Handle,
        event_info: &ContactForceEventInfo,
    ) -> bool {
        let spaces_lock = spaces_singleton().lock().unwrap();
        let active_spaces_lock = active_spaces_singleton().lock().unwrap();
        let mut send_contacts = false;
        if let Some(space) = active_spaces_lock.active_spaces.get(&world_handle) {
            if let Some(space) = spaces_lock.spaces.get(space) {
                send_contacts = space.is_debugging_contacts();
            }

        let (pObject1,_)  = 
        RapierCollisionObject2D::get_collider_user_data(&event_info.user_data1);

        let (pObject2,_)  = 
        RapierCollisionObject2D::get_collider_user_data(&event_info.user_data2);
        let bodies_lock = bodies_singleton().lock().unwrap();
        if let Some(body1) = bodies_lock.collision_objects.get(&pObject1) {
            if let Some(body1) = body1.get_body() {
                if body1.can_report_contacts() {
                    send_contacts = true;
                }
            }
        }
        if let Some(body2) = bodies_lock.collision_objects.get(&pObject2) {
            if let Some(body2) = body2.get_body() {
                if body2.can_report_contacts() {
                    send_contacts = true;
                }
            }
        }

        return send_contacts;
    }
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
        for exclude_index in 0..handle_excluded_info.query_exclude_size {
            if handle_excluded_info.query_exclude[exclude_index] == collider_handle {
                return true;
            }
        }
    
        let (collision_object_2d, shape_index) = RapierCollisionObject2D::get_collider_user_data(user_data);
        let body_lock = bodies_singleton().lock().unwrap();
        let collision_object_2d = body_lock.collision_objects.get(&collision_object_2d).unwrap();
        if handle_excluded_info.query_canvas_instance_id != collision_object_2d.get_base().get_canvas_instance_id() {
            return true;
        }
    
        if collision_object_2d.get_base().get_collision_layer() & handle_excluded_info.query_collision_layer_mask == 0 {
            return true;
        }
    
        if handle_excluded_info.query_exclude_body == collision_object_2d.get_base().get_rid().to_u64() as i64 {
            return true;
        }
        let spaces_lock = spaces_singleton().lock().unwrap();
        let active_spaces_lock = active_spaces_singleton().lock().unwrap();
        let space = active_spaces_lock.active_spaces.get(&world_handle).unwrap();
        let space = spaces_lock.spaces.get(space).unwrap();
        let direct_state = space.get_rapier_direct_state().unwrap();
        return direct_state.base().is_body_excluded_from_query(collision_object_2d.get_base().get_rid());
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
    pub fn body_remove_from_mass_properties_update_list(&mut self, body: Rid) {
        self.mass_properties_update_list.retain(|&x| x != body);
    }
    pub fn body_add_to_gravity_update_list(&mut self, body: Rid) {
        self.gravity_update_list.push(body);
    }
    pub fn body_remove_from_gravity_update_list(&mut self, body: Rid) {
        self.gravity_update_list.retain(|&x| x != body);
    }

    pub fn body_add_to_active_list(&mut self, body: Rid) {
        self.active_list.push(body);
    }
    pub fn body_remove_from_active_list(&mut self, body: Rid) {
        self.state_query_list.retain(|&x| x != body);
    }
    pub fn body_add_to_state_query_list(&mut self, body: Rid) {
        self.state_query_list.push(body);
    }
    pub fn body_remove_from_state_query_list(&mut self, body: Rid) {
        self.state_query_list.retain(|&x| x != body);
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
    pub fn body_remove_from_area_update_list(&mut self, body: Rid) {
        self.body_area_update_list.retain(|&x| x != body);
    }

    pub fn add_removed_collider(&mut self, handle: Handle, object: Rid, instance_id: u64, shape_index: usize, collision_object_type: CollisionObjectType) {
        self.removed_colliders.insert(handle, RemovedColliderInfo::new(object, instance_id, shape_index, collision_object_type));
    }
    pub fn get_removed_collider_info(
        &mut self,
        handle: &Handle,
    ) -> Option<&RemovedColliderInfo> {
        self.removed_colliders.get(&handle)
    }

    pub fn get_solver_iterations(&self) -> i32 {
        return self.solver_iterations;
    }

    pub fn step(&mut self, step: real) {
        for body in &self.active_list {
            let mut lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get_mut(&body) {
                if let Some(body) = body.get_mut_body() {
                    body.reset_contact_count();
                }
            }
        }
        self.contact_debug_count = 0;
        
        let project_settings = ProjectSettings::singleton();
    
        let default_gravity_dir: Vector2 = project_settings.get_setting_with_override("physics/2d/default_gravity_vector".into()).to();
        let default_gravity_value: real = project_settings.get_setting_with_override("physics/2d/default_gravity".into()).to();
    
        let fluid_default_gravity_dir = RapierProjectSettings::get_fluid_gravity_dir();
        let fluid_default_gravity_value = RapierProjectSettings::get_fluid_gravity_value();
    
        let default_linear_damping: real = project_settings.get_setting_with_override("physics/2d/default_linear_damp".into()).to();
        let default_angular_damping: real = project_settings.get_setting_with_override("physics/2d/default_angular_damp".into()).to();
        
        for body in &self.mass_properties_update_list {
            let mut lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get_mut(&body) {
                if let Some(body) = body.get_mut_body() {
                    body.update_mass_properties(false);
                }
            }
        }
        for area in &self.area_update_list {
            let mut lock = bodies_singleton().lock().unwrap();
            if let Some(area) = lock.collision_objects.get_mut(&area) {
                if let Some(area) = area.get_mut_area() {
                    area.update_area_override();
                }
            }
        }
        for body in &self.body_area_update_list {
            let mut lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get_mut(&body) {
                if let Some(body) = body.get_mut_body() {
                    body.update_area_override();
                }
            }
        }
        for body in &self.gravity_update_list {
            let mut lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get_mut(&body) {
                if let Some(body) = body.get_mut_body() {
                    body.update_gravity(step);
                }
            }
        }
    
        let settings = SimulationSettings{
            pixel_liquid_gravity: Vector::new(fluid_default_gravity_dir.x * fluid_default_gravity_value as real,fluid_default_gravity_dir.y * fluid_default_gravity_value as real),
            dt: step,
            pixel_gravity: Vector::new(default_gravity_dir.x * default_gravity_value,default_gravity_dir.y * default_gravity_value),
            max_ccd_substeps: RapierProjectSettings::get_solver_max_ccd_substeps() as usize,
            num_additional_friction_iterations: RapierProjectSettings::get_solver_num_additional_friction_iterations() as usize,
            num_internal_pgs_iterations: RapierProjectSettings::get_solver_num_internal_pgs_iterations() as usize,
            num_solver_iterations: RapierProjectSettings::get_solver_num_solver_iterations() as usize,
        };
    
        world_step(self.handle, &settings);
    
        // Needed only for one physics step to retrieve lost info
        self.removed_colliders.clear();
    
        for body in &self.gravity_update_list {
            let mut lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get_mut(&body) {
                if let Some(body) = body.get_mut_body() {
                    body.on_update_active();
                }
            }
        }
        self.active_objects = world_get_active_objects_count(self.handle) as i32;
    }

    pub fn call_queries(&mut self) {
        for body_rid in &self.state_query_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(body) = lock.collision_objects.get(body_rid) {
                if let Some(body) = body.get_body() {
                    body.call_queries();
                }
            }
        }
        for area_rid in &self.monitor_query_list {
            let lock = bodies_singleton().lock().unwrap();
            if let Some(area) = lock.collision_objects.get(area_rid) {
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

    pub fn get_last_step() -> real {
        let project_settings = ProjectSettings::singleton();
        let physics_fps = project_settings
            .get_setting_with_override("physics/common/physics_ticks_per_second".into());
        let mut last_step = 1e-3;
        if !physics_fps.is_nil() {
            last_step = 1.0 / (physics_fps.to::<i32>() as f32);
        }
        return last_step;
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
        if self.contact_debug_count < self.contact_debug.len() {
            self.contact_debug.set(self.contact_debug_count, contact);
            self.contact_debug_count += 1;
        }
    }
    pub fn get_debug_contacts(&self) -> PackedVector2Array {
        return self.contact_debug.clone();
    }
    pub fn get_debug_contact_count(&self) -> i32 {
        self.contact_debug_count as i32
    }

    pub fn get_direct_state(&self) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        self.direct_access
    }

    pub fn get_rapier_direct_state(&self) -> Option<Gd<RapierDirectSpaceState2D>> {
        if let Some(direct_access) = self.direct_access {
            return Some(direct_access.cast())
        }
        None
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
        results: &mut [PointHitInfo],
        max_results: usize,
        exclude_body: Rid,
    ) -> i32 {
        let max_results = max_results as usize;
        if max_results < 1 {
            return 0;
        }

        let rect_begin = Vector::new( aabb.position.x, aabb.position.y );
        let rect_end = Vector::new( aabb.end().x, aabb.end().y );
        let mut handle_excluded_info = default_query_excluded_info();
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
    let shape_count = p_body.get_base().get_shape_count();
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
        let margin_aabb = p_transform.basis_xform(body_aabb);
        let margin_aabb = margin_aabb.grow(p_margin);
        
        let result_count = self.rapier_intersect_aabb(
            margin_aabb,
            p_body.get_base().get_collision_mask(),
            true,
            false,
            &mut results,
            32,
            p_body.get_base().get_rid(),
        );
        // Optimization
        if result_count == 0 {
            break;
        }

        let mut recover_step = Vector2::default();

        for body_shape_idx in 0..p_body.get_base().get_shape_count() {
            if p_body.get_base().is_shape_disabled(body_shape_idx) {
                continue;
            }

            let body_shape = p_body.get_base().get_shape(body_shape_idx);
            let body_shape_transform = *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
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
    let margin_aabb = p_transform.basis_xform(body_aabb);

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

    for body_shape_idx in 0..p_body.get_base().get_shape_count() {
        if p_body.get_base().is_shape_disabled(body_shape_idx) {
            continue;
        }

        let body_shape = p_body.get_base().get_shape(body_shape_idx);
        let body_shape_transform = *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
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
    let shape_count = p_body.get_base().get_shape_count();
    if shape_count < 1 {
        return false;
    }
    let body_aabb = p_body.get_aabb();
    let margin_aabb = p_transform.basis_xform(body_aabb);
    let margin_aabb = margin_aabb.grow(p_margin);

    // also check things at motion
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
    // Optimization
    if result_count == 0 {
        return false;
    }

    let mut min_distance = f32::INFINITY;
    let mut best_collision_body = None;
    let mut best_collision_shape_index: i32 = -1;
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
        p_body.get_base().get_shape_count()
    };
    for body_shape_idx in from_shape..to_shape {
        if p_body.get_base().is_shape_disabled(body_shape_idx as usize) {
            continue;
        }
        let mut shapes_lock = shapes_singleton().lock().unwrap();
        let body_shape = p_body.get_base().get_shape(body_shape_idx as usize);
        let body_shape_transform = *p_transform * p_body.get_base().get_shape_transform(body_shape_idx as usize);
        let mut body_shape_obj = shapes_lock.shapes.get(&body_shape).unwrap();
        let body_shape_info =
            shape_info_from_body_shape(body_shape_obj.get_rapier_shape(), body_shape_transform);

        for result_idx in 0..result_count {
            let result = &mut results[result_idx as usize];

            if !is_user_data_valid(result.user_data) {
                continue;
            }
            let (shape_col_object, shape_index) =
                RapierCollisionObject2D::get_collider_user_data(&result.user_data);
            if shape_col_object.is_invalid() {
                continue;
            }
            let bodies_lock = bodies_singleton().lock().unwrap();
            let shape_col_object = bodies_lock.collision_objects.get(&shape_col_object).unwrap();
            if shape_col_object.get_base().get_type() != CollisionObjectType::Body {
                continue;
            }
            let collision_body = shape_col_object.get_body().unwrap();

            let col_shape_rid = collision_body.get_base().get_shape(shape_index);
            let col_shape = shapes_lock.shapes.get_mut(&col_shape_rid).unwrap();

            let col_shape_transform =
                collision_body.get_base().get_transform() * collision_body.get_base().get_shape_transform(shape_index);
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
                body_shape_obj,
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
                best_collision_shape_index = shape_index as i32;
                best_body_shape_index = body_shape_idx;
                best_contact = contact;
            }
        }
    }
    if let Some(best_collision_body) = best_collision_body {
        if let Some(p_result) = p_result {
            // conveyer belt
            if best_collision_body.get_static_linear_velocity() != Vector2::default() {
                p_result.travel += best_collision_body.get_static_linear_velocity() * self.get_last_step();
            }
            p_result.collider = best_collision_body.get_base().get_rid();
            p_result.collider_id = ObjectId{id : best_collision_body.get_base().get_instance_id()};
            p_result.collider_shape = best_collision_shape_index as i32;
            p_result.collision_local_shape = best_body_shape_index;
            // World position from the moving body to get the contact point
            p_result.collision_point = Vector2::new(best_contact.pixel_point1.x, best_contact.pixel_point1.y);
            // Normal from the collided object to get the contact normal
            p_result.collision_normal = Vector2::new(best_contact.normal2.x, best_contact.normal2.y);
            // compute distance without sign
            p_result.collision_depth = p_margin - best_contact.pixel_distance;

            let local_position = p_result.collision_point - best_collision_body.get_base().get_transform().origin;
            p_result.collider_velocity = best_collision_body.get_velocity_at_local_point(local_position);
        }

        return true;
    }

    false
}

}


fn should_skip_collision_one_dir(
    contact: ContactResult,
    body_shape: &Box<dyn IRapierShape2D>,
    collision_body: &dyn IRapierCollisionObject2D,
    shape_index: usize,
    col_shape_transform: &Transform2D,
    p_margin: f32,
    last_step: f32,
    p_motion: Vector2,
) -> bool {
    let dist = contact.pixel_distance;
    if !contact.within_margin
        && body_shape.allows_one_way_collision()
        && collision_body.get_base().is_shape_set_as_one_way_collision(shape_index)
    {   
        let valid_dir = col_shape_transform.origin.normalized();

        let owc_margin = collision_body.get_base().get_shape_one_way_collision_margin(shape_index);
        let mut valid_depth = owc_margin.max(p_margin);

        if collision_body.get_base().get_type() == CollisionObjectType::Body {
            let b = collision_body.get_body().unwrap();
            if b.get_mode().ord() >= BodyMode::KINEMATIC.ord() {
                // fix for moving platforms (kinematic and dynamic), margin is increased by how much it moved in the
                // given direction
                let lv = b.get_linear_velocity();
                // compute displacement from linear velocity
                let motion = lv * last_step;
                let motion_len = motion.length();
                let motion = motion.normalized();
                valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
            }
        }
        let motion = p_motion;
        let motion_len = motion.length();
        let motion = motion.normalized();
        valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
        if dist < -valid_depth || p_motion.normalized().dot(valid_dir) < EPSILON {
            return true;
        }
    }
    false
}
