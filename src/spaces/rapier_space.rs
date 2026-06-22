use std::collections::BTreeSet;

use bodies::rapier_area::RapierArea;
use bodies::rapier_body::RapierBody;
use bodies::rapier_collision_object_base::CollisionObjectType;
use godot::classes::ProjectSettings;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
#[cfg(feature = "serde-serialize")]
use hashbrown::HashSet;
#[cfg(feature = "serde-serialize")]
use rapier::geometry::ColliderPair;
#[cfg(feature = "serde-serialize")]
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
#[cfg(feature = "serde-serialize")]
use crate::bodies::exportable_object::ExportToImport;
#[cfg(feature = "serde-serialize")]
use crate::bodies::exportable_object::ExportableObject;
#[cfg(feature = "serde-serialize")]
use crate::bodies::exportable_object::ImportToExport;
#[cfg(feature = "serde-serialize")]
use crate::bodies::exportable_object::ObjectImportState;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::physics_data;
use crate::servers::rapier_project_settings::*;
use crate::types::*;
use crate::*;

enum PendingQueryCallback {
    Callv {
        callable: Callable,
        args: VarArray,
    },
    Call {
        callable: Callable,
        args: Vec<Variant>,
    },
}

impl PendingQueryCallback {
    fn call(self) {
        match self {
            Self::Callv { callable, args } => {
                callable.callv(&args);
            }
            Self::Call { callable, args } => {
                callable.call(args.as_slice());
            }
        }
    }
}

#[cfg(feature = "dim2")]
const DEFAULT_GRAVITY_VECTOR: &str = "physics/2d/default_gravity_vector";
#[cfg(feature = "dim3")]
const DEFAULT_GRAVITY_VECTOR: &str = "physics/3d/default_gravity_vector";
#[cfg(feature = "dim2")]
const DEFAULT_GRAVITY: &str = "physics/2d/default_gravity";
#[cfg(feature = "dim3")]
const DEFAULT_GRAVITY: &str = "physics/3d/default_gravity";
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize, Clone))]
#[cfg(feature = "serde-serialize")]
pub struct SpaceExport<'a> {
    space: &'a RapierSpaceState,
    world: &'a PhysicsObjects,
}
#[cfg(feature = "serde-serialize")]
impl<'a> ExportToImport for SpaceExport<'a> {
    type Import = Box<SpaceImport>;

    fn into_import(self) -> Self::Import {
        Box::new(SpaceImport {
            space: self.space.clone(),
            world: self.world.clone(),
        })
    }
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize, Clone))]
#[cfg(feature = "serde-serialize")]
pub struct SpaceImport {
    space: RapierSpaceState,
    pub(crate) world: PhysicsObjects,
}
#[cfg(feature = "serde-serialize")]
impl ImportToExport for SpaceImport {
    type Export<'a> = SpaceExport<'a>;

    fn as_export<'a>(&'a self) -> Self::Export<'a> {
        SpaceExport {
            space: &self.space,
            world: &self.world,
        }
    }
}
#[cfg(feature = "serde-serialize")]
impl ExportableObject for RapierSpace {
    type ExportState<'a> = SpaceExport<'a>;

    fn get_export_state<'a>(
        &'a self,
        physics_engine: &'a mut PhysicsEngine,
    ) -> Option<Self::ExportState<'a>> {
        physics_engine
            .world_export(self.state.get_id())
            .map(|inner| SpaceExport {
                space: &self.state,
                world: inner,
            })
    }

    fn import_state(&mut self, physics_engine: &mut PhysicsEngine, data: ObjectImportState) {
        match data {
            bodies::exportable_object::ObjectImportState::Space(space_import) => {
                self.import(physics_engine, *space_import);
            }
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
    #[cfg(feature = "dim2")]
    constraint_default_bias: real,
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
            #[cfg(feature = "dim2")]
            constraint_default_bias: 0.2,
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

    #[inline]
    fn collect_body_state_query(
        body_id: RapierId,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> Option<PendingQueryCallback> {
        let mut direct_state_array = None;
        let mut state_sync_callback = None;
        if let Some(body) = physics_collision_objects.get_mut(&get_id_rid(body_id, physics_ids))
            && let Some(body) = body.get_mut_body()
        {
            body.create_direct_state();
            state_sync_callback = body.get_state_sync_callback().cloned();
            direct_state_array = Some(body.get_direct_state_array().clone());
        }
        if let Some(state_sync_callback) = state_sync_callback
            && let Some(direct_state_array) = direct_state_array
        {
            return Some(PendingQueryCallback::Callv {
                callable: state_sync_callback,
                args: direct_state_array,
            });
        }
        None
    }

    #[inline]
    fn collect_body_force_query(
        body_id: RapierId,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> Option<PendingQueryCallback> {
        let mut fi_callback = None;
        let mut fi_array = None;
        if let Some(body) = physics_collision_objects.get_mut(&get_id_rid(body_id, physics_ids))
            && let Some(body) = body.get_mut_body()
        {
            body.create_direct_state();
            fi_callback = body.get_force_integration_callable().cloned();
            fi_array = Some(body.get_force_integration_array().clone());
        }
        if let Some(fi_callback) = fi_callback
            && let Some(fi_array) = fi_array
        {
            return Some(PendingQueryCallback::Callv {
                callable: fi_callback,
                args: fi_array,
            });
        }
        None
    }

    #[inline]
    fn collect_body_state_and_force_queries(
        body_id: RapierId,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> Vec<PendingQueryCallback> {
        let mut callbacks = Vec::new();
        let mut direct_state_array = None;
        let mut state_sync_callback = None;
        let mut fi_callback = None;
        let mut fi_array = None;
        if let Some(body) = physics_collision_objects.get_mut(&get_id_rid(body_id, physics_ids))
            && let Some(body) = body.get_mut_body()
        {
            body.create_direct_state();
            state_sync_callback = body.get_state_sync_callback().cloned();
            fi_callback = body.get_force_integration_callable().cloned();
            direct_state_array = Some(body.get_direct_state_array().clone());
            fi_array = Some(body.get_force_integration_array().clone());
        }
        if let Some(state_sync_callback) = state_sync_callback
            && let Some(direct_state_array) = direct_state_array
        {
            callbacks.push(PendingQueryCallback::Callv {
                callable: state_sync_callback,
                args: direct_state_array,
            });
        }
        if let Some(fi_callback) = fi_callback
            && let Some(fi_array) = fi_array
        {
            callbacks.push(PendingQueryCallback::Callv {
                callable: fi_callback,
                args: fi_array,
            });
        }
        callbacks
    }

    #[inline]
    fn for_each_intersection_body<F>(
        first: &BTreeSet<RapierId>,
        second: &BTreeSet<RapierId>,
        mut callback: F,
    ) where
        F: FnMut(RapierId),
    {
        let (iterate_list, filter_list) = if first.len() <= second.len() {
            (first, second)
        } else {
            (second, first)
        };
        for body_id in iterate_list {
            if filter_list.contains(body_id) {
                callback(*body_id);
            }
        }
    }

    fn collect_query_callbacks(
        active_list: &BTreeSet<RapierId>,
        deactivated_state_sync_list: &BTreeSet<RapierId>,
        state_query_list: &BTreeSet<RapierId>,
        force_integrate_query_list: &BTreeSet<RapierId>,
        monitor_query_list: &BTreeSet<RapierId>,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> Vec<PendingQueryCallback> {
        let mut callbacks = Vec::new();
        if !active_list.is_empty() && force_integrate_query_list.is_empty() {
            Self::for_each_intersection_body(active_list, state_query_list, |body_id| {
                if let Some(callback) =
                    Self::collect_body_state_query(body_id, physics_collision_objects, physics_ids)
                {
                    callbacks.push(callback);
                }
            });
        } else if !active_list.is_empty() && state_query_list.is_empty() {
            Self::for_each_intersection_body(active_list, force_integrate_query_list, |body_id| {
                if let Some(callback) =
                    Self::collect_body_force_query(body_id, physics_collision_objects, physics_ids)
                {
                    callbacks.push(callback);
                }
            });
        } else if !active_list.is_empty() {
            for body_id in active_list {
                let state_query = state_query_list.contains(body_id);
                let force_query = force_integrate_query_list.contains(body_id);
                match (state_query, force_query) {
                    (true, true) => callbacks.extend(Self::collect_body_state_and_force_queries(
                        *body_id,
                        physics_collision_objects,
                        physics_ids,
                    )),
                    (true, false) => {
                        if let Some(callback) = Self::collect_body_state_query(
                            *body_id,
                            physics_collision_objects,
                            physics_ids,
                        ) {
                            callbacks.push(callback);
                        }
                    }
                    (false, true) => {
                        if let Some(callback) = Self::collect_body_force_query(
                            *body_id,
                            physics_collision_objects,
                            physics_ids,
                        ) {
                            callbacks.push(callback);
                        }
                    }
                    (false, false) => {}
                }
            }
        }
        Self::for_each_intersection_body(
            deactivated_state_sync_list,
            state_query_list,
            |body_id| {
                if !active_list.contains(&body_id)
                    && let Some(callback) = Self::collect_body_state_query(
                        body_id,
                        physics_collision_objects,
                        physics_ids,
                    )
                {
                    callbacks.push(callback);
                }
            },
        );
        for area_handle in monitor_query_list {
            let mut unhandled_event_queue = None;
            let mut monitor_callback = None;
            let mut area_monitor_callback = None;
            if let Some(area) =
                physics_collision_objects.get(&get_id_rid(*area_handle, physics_ids))
                && let Some(area) = area.get_area()
            {
                unhandled_event_queue = Some(area.state.unhandled_events.clone());
                monitor_callback = area.monitor_callback.clone();
                area_monitor_callback = area.area_monitor_callback.clone();
            }
            if let Some(unhandled_event_queue) = unhandled_event_queue {
                for monitor_report in unhandled_event_queue.values() {
                    if monitor_report.state == 0 {
                        godot_error!("Invalid monitor state");
                        continue;
                    }
                    let rid = get_id_rid(monitor_report.id, physics_ids);
                    let godot_instance_id: i64 = if let Some(obj_rid) =
                        physics_ids.get(&monitor_report.instance_id)
                        && let Some(obj) = physics_collision_objects.get(obj_rid)
                    {
                        obj.get_base().get_instance_id() as i64
                    } else {
                        0
                    };
                    let arg_array = if monitor_report.state > 0 {
                        vec![
                            AreaBodyStatus::ADDED.to_variant(),
                            rid.to_variant(),
                            godot_instance_id.to_variant(),
                            monitor_report.object_shape_index.to_variant(),
                            monitor_report.this_area_shape_index.to_variant(),
                        ]
                    } else {
                        vec![
                            AreaBodyStatus::REMOVED.to_variant(),
                            rid.to_variant(),
                            godot_instance_id.to_variant(),
                            monitor_report.object_shape_index.to_variant(),
                            monitor_report.this_area_shape_index.to_variant(),
                        ]
                    };
                    if monitor_report.collision_object_type == CollisionObjectType::Body {
                        if let Some(monitor_callback) = &monitor_callback {
                            callbacks.push(PendingQueryCallback::Call {
                                callable: monitor_callback.clone(),
                                args: arg_array,
                            });
                        }
                    } else if let Some(area_monitor_callback) = &area_monitor_callback {
                        callbacks.push(PendingQueryCallback::Call {
                            callable: area_monitor_callback.clone(),
                            args: arg_array,
                        });
                    }
                }
            }
        }
        callbacks
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
        let mut area_update_list = BTreeSet::default();
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
        self.contact_debug.resize(max_contacts.max(0) as usize);
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

    #[cfg(feature = "dim2")]
    pub fn set_param(&mut self, _param: SpaceParameter, _value: f32) {
        if _param == SpaceParameter::CONSTRAINT_DEFAULT_BIAS {
            self.constraint_default_bias = _value;
        }
    }

    #[cfg(feature = "dim2")]
    pub fn get_param(&self, _param: SpaceParameter) -> f32 {
        match _param {
            SpaceParameter::CONSTRAINT_DEFAULT_BIAS => self.constraint_default_bias,
            _ => 0.0,
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
        let active_bodies: Vec<_> = self.state.get_active_list().iter().copied().collect();
        for body in active_bodies {
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
    pub fn get_intersection_deltas(
        &self,
        physics_engine: &mut PhysicsEngine,
        new_narrowphase: &NarrowPhase,
    ) -> Option<(Vec<ColliderPair>, Vec<ColliderPair>)> {
        let mut stale_collider_pairs: Vec<ColliderPair> = Vec::new();
        let mut new_collider_pairs: Vec<ColliderPair> = Vec::new();
        if let Some(current_world) = physics_engine.get_mut_world(self.get_state().get_id()) {
            // Convert the narrowphases into hashsets so we can idiomatically get their differences and intersections.
            let imp_set: HashSet<_> = new_narrowphase.intersection_pairs().collect();
            let cur_set: HashSet<_> = current_world
                .physics_objects
                .narrow_phase
                .intersection_pairs()
                .collect();
            let only_in_imported: Vec<_> = imp_set.difference(&cur_set).cloned().collect();
            let only_in_current: Vec<_> = cur_set.difference(&imp_set).cloned().collect();
            for (handle1, handle2, _intersecting) in only_in_current {
                stale_collider_pairs.push(ColliderPair::new(handle1, handle2));
            }
            for (handle1, handle2, _intersecting) in only_in_imported {
                new_collider_pairs.push(ColliderPair::new(handle1, handle2));
            }
            return Some((stale_collider_pairs, new_collider_pairs));
        }
        None
    }

    #[cfg(feature = "serde-serialize")]
    fn import(&mut self, physics_engine: &mut PhysicsEngine, import: SpaceImport) {
        // NOTE: Areas in this space MUST be made to clean up their stale intersections before import is called here.
        self.state = import.space;
        let world_settings = WorldSettings {
            particle_radius: RapierProjectSettings::get_fluid_particle_radius() as real,
            smoothing_factor: RapierProjectSettings::get_fluid_smoothing_factor() as real,
            counters_enabled: false,
            boundary_coef: RapierProjectSettings::get_fluid_boundary_coef() as real,
            #[cfg(feature = "parallel")]
            thread_count: RapierProjectSettings::get_num_threads(),
        };
        let physics_objects = import.world;
        physics_engine.world_import(self.get_state().get_id(), &world_settings, physics_objects);
    }

    pub fn flush(&mut self) {
        let callbacks = {
            let physics_data = physics_data();
            let physics_ids = physics_data.ids.clone();
            let active_list = self.get_state().get_active_list().clone();
            let deactivated_state_sync_list =
                self.get_state().get_deactivated_state_sync_list().clone();
            let state_query_list = self.get_state().get_state_query_list().clone();
            let force_integrate_query_list =
                self.get_state().get_force_integrate_query_list().clone();
            let monitor_query_list = self.get_state().get_monitor_query_list().clone();
            RapierSpace::collect_query_callbacks(
                &active_list,
                &deactivated_state_sync_list,
                &state_query_list,
                &force_integrate_query_list,
                &monitor_query_list,
                &mut physics_data.collision_objects,
                &physics_ids,
            )
        };
        for callback in callbacks {
            callback.call();
        }
        let physics_data = physics_data();
        self.get_mut_state().reset_deactivated_state_sync_list();
        self.update_after_queries(&mut physics_data.collision_objects, &physics_data.ids);
    }

    pub fn get_ghost_collision_distance(&self) -> real {
        self.ghost_collision_distance
    }
}
impl Drop for RapierSpace {
    fn drop(&mut self) {
        if let Some(direct_access) = self.direct_access.take() {
            direct_access.free();
        }
    }
}
