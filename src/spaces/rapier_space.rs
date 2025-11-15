use std::sync::mpsc;

use bodies::rapier_area::RapierArea;
use bodies::rapier_body::RapierBody;
use godot::classes::ProjectSettings;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::HashSet;
use rapier::dynamics::IntegrationParameters;
use rapier::geometry::BroadPhasePairEvent;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;
use servers::rapier_physics_singleton::PhysicsData;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::PhysicsSpaces;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;
use spaces::rapier_space_state::RapierSpaceState;

use super::PhysicsDirectSpaceState;
use super::RapierDirectSpaceState;
use crate::bodies::exportable_object::ExportableObject;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::RapierPhysicsServer;
use crate::servers::rapier_project_settings::*;
use crate::types::*;
use crate::*;
#[cfg(feature = "dim2")]
const DEFAULT_GRAVITY_VECTOR: &str = "physics/2d/default_gravity_vector";
#[cfg(feature = "dim3")]
const DEFAULT_GRAVITY_VECTOR: &str = "physics/3d/default_gravity_vector";
#[cfg(feature = "dim2")]
const DEFAULT_GRAVITY: &str = "physics/2d/default_gravity";
#[cfg(feature = "dim3")]
const DEFAULT_GRAVITY: &str = "physics/3d/default_gravity";
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize, Clone))]
pub struct SpaceExport<'a> {
    space: &'a RapierSpaceState,
    world: &'a PhysicsObjects,
}
#[cfg(feature = "serde-serialize")]
impl ExportableObject for RapierSpace {
    type ExportState<'a> = SpaceExport<'a>;

    fn get_export_state<'a>(&'a self, physics_engine: &'a mut PhysicsEngine) -> Option<Self::ExportState<'a>> {
        if let Some(inner) = physics_engine.world_export(self.state.get_id()) {
            Some(SpaceExport {
                space: &self.state,
                world: inner, 
            })
        } else {
            return None
        }   
    }
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize))]
pub struct SpaceImport {
    space: RapierSpaceState,
    world: PhysicsObjects,
}
pub struct RapierSpace {
    direct_access: Option<Gd<PhysicsDirectSpaceState>>,
    contact_max_allowed_penetration: real,
    default_gravity_dir: Vector,
    default_gravity_value: real,
    default_linear_damping: real,
    default_angular_damping: real,
    contact_debug: PackedVectorArray,
    contact_debug_count: usize,
    ghost_collision_distance: real,
    state: RapierSpaceState,
}
impl RapierSpace {
    pub fn create(
        id: RapierId,
        rid: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        let mut direct_access = RapierDirectSpaceState::new_alloc();
        direct_access.bind_mut().set_space(rid);
        let project_settings = ProjectSettings::singleton();
        let default_gravity_dir: Vector = project_settings
            .get_setting_with_override(DEFAULT_GRAVITY_VECTOR)
            .try_to()
            .unwrap_or_default();
        let default_gravity_value =
            variant_to_float(&project_settings.get_setting_with_override(DEFAULT_GRAVITY));
        let space = Self {
            direct_access: Some(direct_access.upcast()),
            contact_max_allowed_penetration: 0.0,
            default_gravity_dir,
            default_gravity_value,
            default_linear_damping: 0.0,
            default_angular_damping: 0.0,
            contact_debug: PackedVectorArray::new(),
            contact_debug_count: 0,
            ghost_collision_distance: RapierProjectSettings::get_ghost_collision_distance(),
            state: RapierSpaceState::new(id, physics_engine, &Self::get_world_settings()),
        };
        physics_spaces.insert(rid, space);
    }

    pub fn get_state(&self) -> &RapierSpaceState {
        &self.state
    }

    pub fn get_mut_state(&mut self) -> &mut RapierSpaceState {
        &mut self.state
    }

    pub fn call_queries(
        state_query_list: &HashSet<RapierId>,
        force_integrate_query_list: &HashSet<RapierId>,
        monitor_query_list: &HashSet<RapierId>,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        for body_id in state_query_list.union(force_integrate_query_list) {
            let mut direct_state_array = None;
            let mut state_sync_callback = None;
            let mut fi_callback = None;
            let mut fi_array = None;
            if let Some(body) =
                physics_collision_objects.get_mut(&get_id_rid(*body_id, physics_ids))
                && let Some(body) = body.get_mut_body()
            {
                body.create_direct_state();
                state_sync_callback = body.get_state_sync_callback();
                fi_callback = body.get_force_integration_callable();
                direct_state_array = Some(body.get_direct_state_array());
                fi_array = Some(body.get_force_integration_array());
            }
            if let Some(state_sync_callback) = state_sync_callback
                && let Some(direct_state_array) = direct_state_array
            {
                state_sync_callback.callv(direct_state_array);
            }
            if let Some(fi_callback) = fi_callback
                && let Some(fi_array) = fi_array
            {
                fi_callback.callv(fi_array);
            }
        }
        for area_handle in monitor_query_list {
            let mut unhandled_event_queue = None;
            let mut monitor_callback = None;
            let mut area_monitor_callback = None;
            if let Some(area) =
                physics_collision_objects.get(&get_id_rid(*area_handle, physics_ids))
                && let Some(area) = area.get_area()
            {
                unhandled_event_queue = Some(area.state.unhandled_event_queue.clone());
                monitor_callback = area.monitor_callback.clone();
                area_monitor_callback = area.area_monitor_callback.clone();
            }
            if let Some(unhandled_event_queue) = unhandled_event_queue {
                
                let mon_obj_len = unhandled_event_queue.len();
                godot_print!("monitored_objects length passed into call_queries is {}", mon_obj_len);
                
                RapierArea::call_queries(
                    &unhandled_event_queue,
                    monitor_callback,
                    area_monitor_callback,
                    physics_ids,
                );
            }
        }
    }

    pub fn update_after_queries(
        &mut self,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        for area_handle in self.state.get_monitor_query_list().clone() {
            if let Some(area) =
                physics_collision_objects.get_mut(&get_id_rid(area_handle, physics_ids))
                && let Some(area) = area.get_mut_area()
            {
                area.clear_event_queue();
            }
        }
        self.state.reset_monitor_query_list();
    }

    pub fn step(
        step: real,
        space_rid: &Rid,
        physics_data: &mut PhysicsData,
        settings: SimulationSettings,
    ) {
        let mut area_update_list = HashSet::default();
        if let Some(space) = physics_data.spaces.get_mut(space_rid) {
            space
                .state
                .set_time_stepped(space.state.get_time_stepped() + step);
            area_update_list = space.get_state().get_area_update_list().clone();
        }
        for area in area_update_list {
            RapierArea::update_area_override(
                &mut physics_data.collision_objects,
                &mut physics_data.spaces,
                &mut physics_data.physics_engine,
                &area,
                &physics_data.ids,
            );
        }
        let Some(space) = physics_data.spaces.get_mut(space_rid) else {
            return;
        };
        let body_area_update_list = space.get_state().get_body_area_update_list().clone();
        let gravity_update_list = space.get_state().get_gravity_update_list().clone();
        let default_gravity_value: real =
            variant_to_float(&space.get_default_area_param(AreaParameter::GRAVITY));
        let default_gravity_dir = space
            .get_default_area_param(AreaParameter::GRAVITY_VECTOR)
            .try_to()
            .unwrap_or_default();
        let space_handle = space.get_state().get_id();
        space.before_step();
        for body in space.get_state().get_active_list() {
            if let Some(body) = physics_data
                .collision_objects
                .get_mut(&get_id_rid(*body, &physics_data.ids))
                && let Some(body) = body.get_mut_body()
            {
                body.reset_contact_count();
            }
        }
        for body in space.get_state().get_mass_properties_update_list() {
            if let Some(body) = physics_data
                .collision_objects
                .get_mut(&get_id_rid(*body, &physics_data.ids))
                && let Some(body) = body.get_mut_body()
            {
                body.update_mass_properties(false, &mut physics_data.physics_engine);
            }
        }
        space.get_mut_state().reset_mass_properties_update_list();
        for body in &body_area_update_list {
            RapierBody::apply_area_override_to_body(
                body,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
        }
        for body in gravity_update_list {
            // first update the area override
            RapierBody::apply_area_override_to_body(
                &body,
                &mut physics_data.physics_engine,
                &mut physics_data.spaces,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
            if let Some(body) = physics_data
                .collision_objects
                .get_mut(&get_id_rid(body, &physics_data.ids))
                && let Some(body) = body.get_mut_body()
            {
                body.update_gravity(step, &mut physics_data.physics_engine);
            }
        }
        let mut settings = settings;
        settings.pixel_liquid_gravity =
            vector_to_rapier(default_gravity_dir) * default_gravity_value;
        settings.pixel_gravity = vector_to_rapier(default_gravity_dir) * default_gravity_value;
        if let Some(space) = physics_data.spaces.get_mut(space_rid) {
            // this calls into rapier
            physics_data.physics_engine.world_step(
                space_handle,
                &settings,
                RapierSpace::collision_filter_body_callback,
                RapierSpace::collision_modify_contacts_callback,
                space,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
            space.after_step(
                &mut physics_data.physics_engine,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
        }
    }

    pub fn get_last_step() -> real {
        let project_settings = ProjectSettings::singleton();
        let physics_fps =
            project_settings.get_setting_with_override("physics/common/physics_ticks_per_second");
        let mut last_step = 1e-3;
        if !physics_fps.is_nil() {
            last_step = 1.0 / (physics_fps.to::<i32>() as f32);
        }
        last_step
    }

    pub fn set_default_area_param(&mut self, param: AreaParameter, value: Variant) {
        match param {
            AreaParameter::GRAVITY => self.default_gravity_value = variant_to_float(&value),
            AreaParameter::GRAVITY_VECTOR => {
                self.default_gravity_dir = value.try_to().unwrap_or_default()
            }
            AreaParameter::LINEAR_DAMP => self.default_linear_damping = variant_to_float(&value),
            AreaParameter::ANGULAR_DAMP => self.default_angular_damping = variant_to_float(&value),
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

    pub fn set_debug_contacts(&mut self, max_contacts: i32) {
        self.contact_debug.resize(max_contacts as usize);
    }

    pub fn is_debugging_contacts(&self) -> bool {
        !self.contact_debug.is_empty()
    }

    pub fn add_debug_contact(&mut self, contact: Vector) {
        if self.contact_debug_count < self.contact_debug.len() {
            self.contact_debug[self.contact_debug_count] = contact;
            self.contact_debug_count += 1;
        }
    }

    pub fn get_debug_contacts(&self) -> &PackedVectorArray {
        &self.contact_debug
    }

    pub fn get_debug_contact_count(&self) -> i32 {
        self.contact_debug_count as i32
    }

    pub fn before_step(&mut self) {
        self.contact_debug_count = 0
    }

    pub fn after_step(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        // Needed only for one physics step to retrieve lost info
        self.state.reset_removed_colliders();
        self.state.set_active_objects(
            physics_engine.world_get_active_objects_count(self.state.get_id()) as i32,
        );
        for body in self.state.get_active_list().clone() {
            if let Some(body) = physics_collision_objects.get_mut(&get_id_rid(body, physics_ids))
                && let Some(body) = body.get_mut_body()
            {
                if body.is_sleeping(physics_engine) {
                    body.set_active(false, self);
                    continue;
                }
                body.on_update_active(self, physics_engine);
            }
        }
    }

    pub fn get_direct_state(&self) -> &Option<Gd<PhysicsDirectSpaceState>> {
        &self.direct_access
    }

    pub fn get_contact_max_allowed_penetration(&self) -> real {
        self.contact_max_allowed_penetration
    }

    pub fn get_world_settings() -> WorldSettings {
        WorldSettings {
            particle_radius: RapierProjectSettings::get_fluid_particle_radius() as real,
            smoothing_factor: RapierProjectSettings::get_fluid_smoothing_factor() as real,
            counters_enabled: false,
            boundary_coef: RapierProjectSettings::get_fluid_boundary_coef() as real,
            #[cfg(feature = "parallel")]
            thread_count: RapierProjectSettings::get_num_threads(),
        }
    }

    #[cfg(feature = "serde-serialize")]
    pub fn export_json(&self, physics_engine: &mut PhysicsEngine) -> String {
        godot_warn!("WARNING: export_json() of a space is broken due to errors while serializing Rapier's Broadphase.");
        if let Some(inner) = physics_engine.world_export(self.state.get_id()) {
            let export = SpaceExport {
                space: &self.state,
                world: inner,
            };

            match serde_json::to_string(&export) {
                Ok(s) => return s,
                Err(e) => {
                    godot_error!("Failed to serialize space to json: {}", e);
                    match serde_json::to_string(&self.state) {
                        Ok(_) => {}
                        Err(e) => {
                            godot_error!("Failed to serialize space state to json: {}", e);
                        }
                    }
                    match serde_json::to_string(&inner) {
                        Ok(_) => {}
                        Err(e) => {
                            godot_error!("Failed to serialize space world to json: {}", e);                            
                        }
                    }
                }
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    pub fn export_binary(&self, physics_engine: &mut PhysicsEngine) -> Vec<u8> {
        if let Some(inner) = physics_engine.world_export(self.state.get_id()) {
            let export = SpaceExport {
                space: &self.state,
                world: inner,
            };
            match bincode::serialize(&export) {
                Ok(binary_data) => {
                    return binary_data
                }
                Err(e) => {
                    godot_error!("Failed to serialize space to binary: {}", e);
                }
            }
        }
        Vec::new()
    }

    fn _import(&mut self, physics_engine: &mut PhysicsEngine, import: SpaceImport) {
        use rapier::geometry::ColliderPair;        
        use crate::servers::rapier_physics_singleton::physics_data;

        let physics_data = physics_data();                
        let imported_physics_objects = import.world;
        
        // Here, we compare our narrowphase to the imported narrowphase. Any collisions present in our pre-load state
        // that don't exist in the imported state will be manually cleaned up.
        let mut stale_collider_pairs: Vec<ColliderPair> = Vec::new();
        if let Some(current_world) = physics_engine.get_mut_world(self.get_state().get_id())
        {
            let imported_narrowphase = imported_physics_objects.narrow_phase.clone();
            let current_narrowphase = current_world.physics_objects.narrow_phase.clone();

            // Do I need to check the reversed pair too? eg handle2, handle1?
            for (handle1, handle2, _intersecting) in current_narrowphase.intersection_pairs()
            {
                match imported_narrowphase.intersection_pair(handle1, handle2) {
                    Some(true) => continue,
                    Some(false) | None => stale_collider_pairs.push(ColliderPair::new(handle1, handle2)),
                }
            }

            // let b = physics_data.collision_objects.iter_mut();
            // current_world.physics_objects.rigid_body_set
            //current_world.physics_objects.collider_set.iter_mut()
                                
            // for (a,b) in current_world.physics_objects.collider_set.iter_mut()
            // {
            //     b.is_sensor()
            //     //let rb1 = co1.parent.map(|co_parent1| &bodies[co_parent1.handle]);
            //     if let Some(parent) = b.parent() {

            //     }

            // }
            // for (a,b) in current_world.physics_objects.rigid_body_set.iter_mut()
            // {
            //     let rb1 = co1.parent.map(|co_parent1| &bodies[co_parent1.handle]);
            // }

            // Is there a better way to iterate through the areas of this specific space?
            for (_, collision_object) in physics_data.collision_objects.iter_mut()
            {
                if collision_object.get_base().get_space_id() == self.get_state().get_id()                     
                    && let Some(area) = collision_object.get_mut_area()
                {
                    area.close_stale_contacts(self, &stale_collider_pairs);
                }
            }
        }


        
        self.flush(physics_data);

        self.state = import.space;
                        
        let world_settings = WorldSettings {
            particle_radius: RapierProjectSettings::get_fluid_particle_radius() as real,
            smoothing_factor: RapierProjectSettings::get_fluid_smoothing_factor() as real,
            counters_enabled: false,
            boundary_coef: RapierProjectSettings::get_fluid_boundary_coef() as real,
            #[cfg(feature = "parallel")]
            thread_count: RapierProjectSettings::get_num_threads(),
        };

        let physics_objects = imported_physics_objects;             
        physics_engine.world_import(
            self.get_state().get_id(),
            &world_settings,
            physics_objects,
        );

        self.zero_tick(physics_data);
        self.flush(physics_data);

    }

    #[cfg(feature = "serde-serialize")]
    pub fn import_json(&mut self, physics_engine: &mut PhysicsEngine, data: String) {
        match serde_json::from_str::<SpaceImport>(&data) {
            Ok(import) => {
                self._import(physics_engine, import);
            }
            Err(e) => {
                godot_error!("Failed to deserialize space from JSON: {}", e);
            }
        }
    }

    #[cfg(feature = "serde-serialize")]
    pub fn import_binary(&mut self, physics_engine: &mut PhysicsEngine, data: PackedByteArray) {     
        match bincode::deserialize::<SpaceImport>(data.as_slice()) {
            Ok(import) => {
                self._import(physics_engine, import);
            }
            Err(e) => {
                godot_error!("Failed to deserialize space from binary: {}", e);
            }
        }
    }

    fn zero_tick(
        &mut self,
        physics_data: &mut PhysicsData,
    )
    {
        // Fetch project settings.
        let settings = SimulationSettings {
            dt: 0.0,
            length_unit: RapierProjectSettings::get_length_unit(),
            max_ccd_substeps: RapierProjectSettings::get_solver_max_ccd_substeps() as usize,
            num_internal_pgs_iterations:
                RapierProjectSettings::get_solver_num_internal_pgs_iterations() as usize,
            num_solver_iterations: RapierProjectSettings::get_solver_num_solver_iterations()
                as usize,
            normalized_allowed_linear_error:
                RapierProjectSettings::get_normalized_allowed_linear_error(),
            normalized_max_corrective_velocity:
                RapierProjectSettings::get_normalized_max_corrective_velocity(),
            normalized_prediction_distance:
                RapierProjectSettings::get_normalized_prediction_distance(),
            predictive_contact_allowance_threshold:
                RapierProjectSettings::get_predictive_contact_allowance_threshold(),
            num_internal_stabilization_iterations:
                RapierProjectSettings::get_num_internal_stabilization_iterations() as usize,
            contact_damping_ratio: RapierProjectSettings::get_contact_damping_ratio(),
            contact_natural_frequency: RapierProjectSettings::get_contact_natural_frequency(),
            pixel_gravity: vector_to_rapier(Vector::ZERO),
            pixel_liquid_gravity: vector_to_rapier(Vector::ZERO),
        };

        let space_rid = physics_data.spaces
            .iter()
            .find(|(_, space)| std::ptr::eq(*space, self))
            .map(|(rid, _)| *rid);

        if let Some(rid) = space_rid {                    
            // Use our space's RID to tick for a zero timestep.
            RapierSpace::step( 0.0, &rid, physics_data, settings);            
        }
    }

    fn flush(
        &mut self,
        physics_data: &mut PhysicsData
    ){
        let state_query_list = Some(self.get_state().get_state_query_list());
        let force_integrate_query_list = Some(self.get_state().get_force_integrate_query_list());
        let monitor_query_list = Some(self.get_state().get_monitor_query_list());

        if let Some(state_query_list) = state_query_list
            && let Some(force_integrate_query_list) = force_integrate_query_list
            && let Some(monitor_query_list) = monitor_query_list
        {
            RapierSpace::call_queries(
                state_query_list,
                force_integrate_query_list,
                monitor_query_list,
                &mut physics_data.collision_objects,
                &physics_data.ids,
            );
        }

        self.update_after_queries(&mut physics_data.collision_objects, &physics_data.ids);
    }

    pub fn get_ghost_collision_distance(&self) -> real {
        self.ghost_collision_distance
    }
}
