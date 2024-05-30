use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::rapier2d::physics_world::{
    world_export_binary, world_export_json, world_get_active_objects_count,
};
use crate::servers::rapier_physics_singleton_2d::bodies_singleton;
use crate::servers::rapier_project_settings::RapierProjectSettings;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::spaces::rapier_direct_space_state_2d::RapierDirectSpaceState2D;
use crate::{
    bodies::rapier_collision_object_2d::CollisionObjectType,
    rapier2d::{
        handle::Handle, physics_world::world_create,
        settings::default_world_settings,
    },
};
use godot::engine::physics_server_2d::{AreaParameter};
use godot::{
    engine::{physics_server_2d, PhysicsDirectSpaceState2D, ProjectSettings},
    prelude::*,
};
use std::collections::{HashMap, HashSet};

pub struct RemovedColliderInfo {
    pub rid: Rid,
    pub instance_id: u64,
    pub shape_index: usize,
    pub collision_object_type: CollisionObjectType,
}

impl RemovedColliderInfo {
    pub fn new(
        rid: Rid,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) -> Self {
        Self {
            rid,
            instance_id,
            shape_index,
            collision_object_type,
        }
    }
}

pub struct RapierSpace2D {
    direct_access: Option<Gd<PhysicsDirectSpaceState2D>>,
    rid: Rid,
    pub handle: Handle,
    removed_colliders: HashMap<Handle, RemovedColliderInfo>,
    active_list: HashSet<Rid>,
    mass_properties_update_list: HashSet<Rid>,
    gravity_update_list: HashSet<Rid>,
    state_query_list: HashSet<Rid>,
    monitor_query_list: HashSet<Rid>,
    area_update_list: HashSet<Rid>,
    body_area_update_list: HashSet<Rid>,
    solver_iterations: i32,
    fluid_default_gravity_dir: Vector2,
    fluid_default_gravity_value: real,
    default_gravity_dir: Vector2,
    default_gravity_value: real,
    default_linear_damping: real,
    default_angular_damping: real,
    pub locked: bool,
    island_count: i32,
    active_objects: i32,
    collision_pairs: i32,
    contact_debug: PackedVector2Array,
    contact_debug_count: usize,
}

impl RapierSpace2D {
    pub fn new(rid: Rid) -> Self {
        let project_settings = ProjectSettings::singleton();
        let solver_iterations = project_settings
            .get_setting_with_override("physics/2d/solver/solver_iterations".into())
            .to();
        let mut direct_access = RapierDirectSpaceState2D::new_alloc();
        direct_access.bind_mut().set_space(rid);

        let mut world_settings = default_world_settings();
        world_settings.particle_radius = RapierProjectSettings::get_fluid_particle_radius() as real;
        world_settings.smoothing_factor =
            RapierProjectSettings::get_fluid_smoothing_factor() as real;
        let handle = world_create(&world_settings);
        assert!(handle.is_valid());

        Self {
            direct_access: Some(direct_access.upcast()),
            rid,
            handle,
            removed_colliders: HashMap::new(),
            active_list: HashSet::new(),
            mass_properties_update_list: HashSet::new(),
            gravity_update_list: HashSet::new(),
            state_query_list: HashSet::new(),
            monitor_query_list: HashSet::new(),
            area_update_list: HashSet::new(),
            body_area_update_list: HashSet::new(),
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

    pub fn _get_object_instance_hack(instance_id: u64) -> *mut Gd<Object> {
        let mut object: Gd<Object> = Gd::from_instance_id(InstanceId::from_i64(instance_id as i64));
        let raw_ptr: *mut Gd<Object> = &mut object as *mut _;
        raw_ptr
    }

    pub fn get_handle(&self) -> Handle {
        self.handle
    }

    pub fn set_rid(&mut self, p_rid: Rid) {
        self.rid = p_rid;
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn body_add_to_mass_properties_update_list(&mut self, body: Rid) {
        self.mass_properties_update_list.insert(body);
    }
    pub fn body_remove_from_mass_properties_update_list(&mut self, body: Rid) {
        self.mass_properties_update_list.remove(&body);
    }
    pub fn body_add_to_gravity_update_list(&mut self, body: Rid) {
        self.gravity_update_list.insert(body);
    }
    pub fn body_remove_from_gravity_update_list(&mut self, body: Rid) {
        self.gravity_update_list.remove(&body);
    }

    pub fn body_add_to_active_list(&mut self, body: Rid) {
        self.active_list.insert(body);
    }
    pub fn body_remove_from_active_list(&mut self, body: Rid) {
        self.active_list.remove(&body);
    }
    pub fn body_add_to_state_query_list(&mut self, body: Rid) {
        self.state_query_list.insert(body);
    }
    pub fn body_remove_from_state_query_list(&mut self, body: Rid) {
        self.state_query_list.remove(&body);
    }

    pub fn area_add_to_monitor_query_list(&mut self, area: Rid) {
        self.monitor_query_list.insert(area);
    }
    pub fn area_add_to_area_update_list(&mut self, area: Rid) {
        self.area_update_list.insert(area);
    }
    pub fn area_remove_from_area_update_list(&mut self, area: Rid) {
        self.area_update_list.remove(&area);
    }
    pub fn body_add_to_area_update_list(&mut self, body: Rid) {
        self.body_area_update_list.insert(body);
    }
    pub fn body_remove_from_area_update_list(&mut self, body: Rid) {
        self.body_area_update_list.remove(&body);
    }

    pub fn add_removed_collider(
        &mut self,
        handle: Handle,
        rid: Rid,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) {
        self.removed_colliders.insert(
            handle,
            RemovedColliderInfo::new(rid, instance_id, shape_index, collision_object_type),
        );
    }
    pub fn get_removed_collider_info(&mut self, handle: &Handle) -> Option<&RemovedColliderInfo> {
        self.removed_colliders.get(handle)
    }

    pub fn get_solver_iterations(&self) -> i32 {
        self.solver_iterations
    }

    pub fn call_queries(&mut self) {
        for body_rid in self.state_query_list.clone() {
            let mut direct_state = None;
            let mut fi_callback_data = None;
            let mut state_sync_callback = Callable::invalid();
            {
                let lock = bodies_singleton();
                if let Some(body) = lock.collision_objects.get_mut(&body_rid) {
                    if let Some(body) = body.get_mut_body() {
                        if let Some(direct_state_gd) = body.get_direct_state() {
                            direct_state = Some(direct_state_gd);
                            fi_callback_data = body.get_force_integration_callback();
                            state_sync_callback = body.get_state_sync_callback();
                        }

                        if !body.is_active() {
                            self.body_remove_from_state_query_list(body.get_base().get_rid());
                        }
                    }
                }
            }
            if let Some(fi_callback_data) = fi_callback_data {
                if fi_callback_data.callable.is_valid() {
                    if let Some(direct_state) = direct_state.clone() {
                        let mut arg_array = Array::new();

                        arg_array.push(direct_state.to_variant());
                        arg_array.push(fi_callback_data.udata.clone());

                        fi_callback_data.callable.callv(arg_array);
                    }
                }
            }
            //  Sync body server with Godot by sending body direct state
            if state_sync_callback.is_valid() {
                if let Some(direct_state) = direct_state.clone() {
                    let mut arg_array = Array::new();
                    arg_array.push(direct_state.to_variant());
                    state_sync_callback.callv(arg_array);
                }
            }
        }
        for area_rid in self.monitor_query_list.clone() {
            let lock = bodies_singleton();
            if let Some(area) = lock.collision_objects.get(&area_rid) {
                if let Some(area) = area.get_area() {
                    area.call_queries(self);
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
        last_step
    }

    pub fn set_param(&mut self, param: physics_server_2d::SpaceParameter, value: real) {
        if param == physics_server_2d::SpaceParameter::SOLVER_ITERATIONS {
            self.solver_iterations = value as i32;
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
            AreaParameter::GRAVITY => self.default_gravity_value.to_variant(),
            AreaParameter::GRAVITY_VECTOR => self.default_gravity_dir.to_variant(),
            AreaParameter::LINEAR_DAMP => self.default_linear_damping.to_variant(),
            AreaParameter::ANGULAR_DAMP => self.default_angular_damping.to_variant(),
            _ => (0.0).to_variant(),
        }
    }

    pub fn get_island_count(&self) -> i32 {
        self.island_count
    }

    pub fn get_active_objects(&self) -> i32 {
        self.active_objects
    }

    pub fn get_collision_pairs(&self) -> i32 {
        self.collision_pairs
    }

    pub fn set_debug_contacts(&mut self, max_contacts: i32) {
        self.contact_debug.resize(max_contacts as usize);
    }
    pub fn is_debugging_contacts(&self) -> bool {
        !self.contact_debug.is_empty()
    }
    pub fn add_debug_contact(&mut self, contact: Vector2) {
        if self.contact_debug_count < self.contact_debug.len() {
            self.contact_debug[self.contact_debug_count] = contact;
            self.contact_debug_count += 1;
        }
    }
    pub fn get_debug_contacts(&self) -> PackedVector2Array {
        self.contact_debug.clone()
    }
    pub fn get_debug_contact_count(&self) -> i32 {
        self.contact_debug_count as i32
    }
    pub fn before_step(&mut self) {
        self.contact_debug_count = 0
    }
    pub fn after_step(&mut self) {
        // Needed only for one physics step to retrieve lost info
        self.removed_colliders.clear();
        self.active_objects = world_get_active_objects_count(self.handle) as i32;

        for body in self.active_list.clone() {
            let lock = bodies_singleton();
            if let Some(body) = lock.collision_objects.get_mut(&body) {
                if let Some(body) = body.get_mut_body() {
                    body.on_update_active(self);
                }
            }
        }
    }

    pub fn get_direct_state(&self) -> Option<Gd<PhysicsDirectSpaceState2D>> {
        self.direct_access.clone()
    }

    pub fn get_rapier_direct_state(&self) -> Option<Gd<RapierDirectSpaceState2D>> {
        if let Some(direct_access) = &self.direct_access {
            return Some(direct_access.clone().cast());
        }
        None
    }

    pub fn get_active_list(&self) -> &HashSet<Rid> {
        &self.active_list
    }
    pub fn get_mass_properties_update_list(&self) -> &HashSet<Rid> {
        &self.mass_properties_update_list
    }
    pub fn get_area_update_list(&self) -> &HashSet<Rid> {
        &self.area_update_list
    }
    pub fn get_body_area_update_list(&self) -> &HashSet<Rid> {
        &self.body_area_update_list
    }
    pub fn get_gravity_update_list(&self) -> &HashSet<Rid> {
        &self.gravity_update_list
    }

    pub fn export_json(&self) -> String {
        world_export_json(self.handle)
    }
    pub fn export_binary(&self) -> PackedByteArray {
        let mut buf = PackedByteArray::new();
        let binary_data = world_export_binary(self.handle);
        buf.resize(binary_data.len());
        for i in 0..binary_data.len() {
            buf[i] = binary_data[i];
        }
        buf
    }
}
