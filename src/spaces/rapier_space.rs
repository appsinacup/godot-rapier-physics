use bodies::rapier_area::RapierArea;
use bodies::rapier_body::RapierBody;
use godot::classes::ProjectSettings;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::HashSet;
use rapier::geometry::ColliderPair;
use rapier::geometry::NarrowPhase;
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
use crate::bodies::exportable_object::ObjectImportState;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::physics_data;
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

#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize, Clone))]
pub struct SpaceImport {
    space: RapierSpaceState,
    pub(crate) world: PhysicsObjects,
}

impl<'a> SpaceExport<'a> {
    pub fn to_import(self) -> SpaceImport {
        SpaceImport {
            space: self.space.clone(),
            world: self.world.clone(),
        }
    }
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

    fn import_state(&mut self, physics_engine: &mut PhysicsEngine, data: ObjectImportState) {
        match data {
            bodies::exportable_object::ObjectImportState::RapierSpace(space_import) => {
                self.import(physics_engine, space_import);
            },
            _ => {
                godot_error!("Attempted to import invalid state data.");
            }
        }        
    }
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

    pub fn get_intersection_deltas(
        &self,
        physics_engine: &mut PhysicsEngine,
        new_narrowphase: &NarrowPhase,
    ) -> Option<(Vec<ColliderPair>, Vec<ColliderPair>)>{
        let mut stale_collider_pairs: Vec<ColliderPair> = Vec::new();
        let mut new_collider_pairs: Vec<ColliderPair> = Vec::new();
        if let Some(current_world) = physics_engine.get_mut_world(self.get_state().get_id())
        {
            // Convert the narrowphases into hashsets so we can idiomatically get their differences and intersections.
            let imp_set: HashSet<_> = new_narrowphase.intersection_pairs().collect();
            let cur_set: HashSet<_> = current_world.physics_objects.narrow_phase.intersection_pairs().collect();

            let only_in_imported: Vec<_> = imp_set.difference(&cur_set).cloned().collect();
            let only_in_current: Vec<_> = cur_set.difference(&imp_set).cloned().collect();
            //let common_to_both: Vec<_> = imp_set.intersection(&cur_set).cloned().collect();

            for (handle1, handle2, _intersecting) in only_in_current {
                stale_collider_pairs.push(ColliderPair::new(handle1, handle2));
            }

            for (handle1, handle2, _intersecting) in only_in_imported {
                new_collider_pairs.push(ColliderPair::new(handle1, handle2));
            }

            return Some((stale_collider_pairs, new_collider_pairs))
        }

        return None
    }

    fn import(&mut self, physics_engine: &mut PhysicsEngine, import: SpaceImport) {             
        // NOTE: Areas in this space MUST be made to clean up their stale intersections before import is called here.
        let imported_physics_objects = import.world; 

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
    }

    pub fn flush(
        &mut self
    ){
        let physics_data = physics_data();
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
