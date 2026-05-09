use bodies::rapier_collision_object_base::CollisionObjectShape;
use bodies::rapier_collision_object_base::CollisionObjectType;
use bodies::rapier_collision_object_base::RapierCollisionObjectBase;
use bodies::rapier_collision_object_base::RapierCollisionObjectBaseState;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::global::godot_error;
use godot::meta::ToGodot;
use godot::obj::EngineEnum;
use godot::prelude::*;
use hashbrown::HashMap;
use rapier::geometry::ColliderHandle;
use rapier::geometry::ColliderPair;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::PhysicsShapes;
use servers::rapier_physics_singleton::PhysicsSpaces;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;

use super::exportable_object::ExportToImport;
use super::exportable_object::ExportableObject;
use super::exportable_object::ImportToExport;
use super::exportable_object::ObjectImportState;
use super::rapier_body::RapierBody;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::spaces::rapier_space::*;
use crate::types::*;
use crate::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
// Contains information necessary to track the full lifetime of a contact or intersection event.
// Includes a count of the number of contacts between this area and the other collider.
// Most of the time, the instance_id, shape indexes and object types are not required-- however,
// when we need to fabricate an exit event (eg in the case of state loading), we will need this information.
pub struct MonitorInfo {
    pub other_collider_id: RapierId,
    pub last_entry_report: EventReport,
    pub num_contacts: i32,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
// This struct contains the data that will get reported to Godot in case of an entry/exit event.
// It's designed to be temporary, and destroyed after the signal is sent.
pub struct EventReport {
    pub id: RapierId,
    pub collision_object_type: CollisionObjectType,
    pub state: i32,
    pub instance_id: u64,
    pub object_shape_index: u32,
    pub this_area_shape_index: u32,
}
impl EventReport {
    pub fn from_monitor_info(from_info: MonitorInfo, state_in: i32) -> Self {
        Self {
            state: state_in,
            id: from_info.last_entry_report.id,
            collision_object_type: from_info.last_entry_report.collision_object_type,
            instance_id: from_info.last_entry_report.instance_id,
            object_shape_index: from_info.last_entry_report.object_shape_index,
            this_area_shape_index: from_info.last_entry_report.this_area_shape_index,
        }
    }

    fn is_same_shape_pair(&self, other: &Self) -> bool {
        self.id == other.id
            && self.collision_object_type == other.collision_object_type
            && self.object_shape_index == other.object_shape_index
            && self.this_area_shape_index == other.this_area_shape_index
    }
}
struct ProcessedMonitorEvent {
    num_contacts: i32,
    should_report: bool,
}
#[derive(Debug, PartialEq, Eq)]
enum MonitorEventResult {
    Contacts(i32),
    StaleRemovedExit,
    MissingExit,
    InvalidState,
}
impl RapierAreaState {
    fn process_monitor_event(
        &mut self,
        this_collider_handle: ColliderHandle,
        other_collider_handle: ColliderHandle,
        event_report: &EventReport,
        event_removed: bool,
    ) -> MonitorEventResult {
        let monitor_key = (other_collider_handle, this_collider_handle);
        let current_monitor = self.monitored_objects.get_mut(&monitor_key);
        match event_report.state {
            -1 => {
                if let Some(current_monitor) = current_monitor {
                    current_monitor.num_contacts -= 1;
                    let num_contacts = current_monitor.num_contacts;
                    if num_contacts == 0 {
                        self.monitored_objects.remove(&monitor_key);
                    }
                    MonitorEventResult::Contacts(num_contacts)
                } else if event_removed {
                    MonitorEventResult::StaleRemovedExit
                } else {
                    MonitorEventResult::MissingExit
                }
            }
            1 => {
                if let Some(current_monitor) = current_monitor {
                    current_monitor.num_contacts += 1;
                    MonitorEventResult::Contacts(current_monitor.num_contacts)
                } else {
                    let new_monitor_info = MonitorInfo {
                        other_collider_id: event_report.id,
                        last_entry_report: *event_report,
                        num_contacts: 1,
                    };
                    self.monitored_objects.insert(monitor_key, new_monitor_info);
                    MonitorEventResult::Contacts(1)
                }
            }
            _ => MonitorEventResult::InvalidState,
        }
    }

    fn find_opposite_unhandled_event_for_shape(
        &self,
        other_collider_handle: ColliderHandle,
        new_event: &EventReport,
    ) -> Option<ColliderHandle> {
        self.unhandled_events
            .iter()
            .find_map(|(queued_collider_handle, queued_event)| {
                if *queued_collider_handle != other_collider_handle
                    && queued_event.state == -new_event.state
                    && queued_event.is_same_shape_pair(new_event)
                {
                    Some(*queued_collider_handle)
                } else {
                    None
                }
            })
    }
}
pub enum AreaUpdateMode {
    EnableSpaceOverride,
    DisableSpaceOverride,
    ResetSpaceOverride,
    None,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
#[derive(Debug)]
pub struct AreaExport<'a> {
    area_state: &'a RapierAreaState,
    base_state: &'a RapierCollisionObjectBaseState,
}
impl ExportToImport for AreaExport<'_> {
    type Import = AreaImport;

    fn into_import(self) -> Self::Import {
        AreaImport {
            area_state: self.area_state.clone(),
            base_state: self.base_state.clone(),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize, Clone))]
pub struct AreaImport {
    area_state: RapierAreaState,
    base_state: RapierCollisionObjectBaseState,
}
impl ImportToExport for AreaImport {
    type Export<'a> = AreaExport<'a>;

    fn as_export<'a>(&'a self) -> Self::Export<'a> {
        AreaExport {
            area_state: &self.area_state,
            base_state: &self.base_state,
        }
    }
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RapierAreaState {
    // New events go into this queue; once handled (eg once reported to Godot), they are removed. No need to serialize this, I think.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub unhandled_events: HashMap<ColliderHandle, EventReport>,
    // This is a persistent list of all current contacts. Entries only disappear when contact ceases, or when contacts are manually cleared.
    #[cfg_attr(
        feature = "serde-serialize",
        serde(
            serialize_with = "rapier::utils::serde::serialize_to_vec_tuple",
            deserialize_with = "rapier::utils::serde::deserialize_from_vec_tuple"
        )
    )]
    pub monitored_objects: HashMap<(ColliderHandle, ColliderHandle), MonitorInfo>,
}
#[derive(Debug)]
pub struct RapierArea {
    gravity_override_mode: AreaSpaceOverrideMode,
    linear_damping_override_mode: AreaSpaceOverrideMode,
    angular_damping_override_mode: AreaSpaceOverrideMode,
    gravity: real,
    gravity_vector: Vector,
    gravity_is_point: bool,
    gravity_point_unit_distance: real,
    linear_damp: real,
    angular_damp: real,
    priority: i32,
    monitorable: bool,
    pub monitor_callback: Option<Callable>,
    pub area_monitor_callback: Option<Callable>,
    pub state: RapierAreaState,
    base: RapierCollisionObjectBase,
}
impl RapierArea {
    pub fn new(id: RapierId, rid: Rid) -> Self {
        Self {
            gravity_override_mode: AreaSpaceOverrideMode::DISABLED,
            linear_damping_override_mode: AreaSpaceOverrideMode::DISABLED,
            angular_damping_override_mode: AreaSpaceOverrideMode::DISABLED,
            gravity: 0.0,
            gravity_vector: Vector::default(),
            gravity_is_point: false,
            gravity_point_unit_distance: 0.0,
            linear_damp: 0.0,
            angular_damp: 0.0,
            priority: 0,
            monitorable: false,
            monitor_callback: None,
            area_monitor_callback: None,
            state: RapierAreaState::default(),
            base: RapierCollisionObjectBase::new(id, rid, CollisionObjectType::Area),
        }
    }

    pub fn enable_space_override(
        area_id: &RapierId,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        let area_rid = get_id_rid(*area_id, physics_ids);
        let mut detected_bodies = HashMap::default();
        let mut space_rid = Rid::Invalid;
        if let Some(area_rid) = physics_collision_objects.get(&area_rid)
            && let Some(area) = area_rid.get_area()
        {
            detected_bodies = area.state.monitored_objects.clone();
            space_rid = area.get_base().get_space(physics_ids);
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            for (_, monitor_info) in detected_bodies.iter() {
                if let [Some(body), Some(area)] = physics_collision_objects.get_disjoint_mut([
                    &get_id_rid(monitor_info.other_collider_id, physics_ids),
                    &area_rid,
                ]) && let Some(body) = body.get_mut_body()
                    && let Some(area) = area.get_mut_area()
                {
                    body.add_area(area, space);
                }
            }
            // No need to update anymore if it was scheduled before
            space
                .get_mut_state()
                .area_remove_from_area_update_list(*area_id);
        }
    }

    pub fn disable_space_override(
        area_id: &RapierId,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        let area_rid = get_id_rid(*area_id, physics_ids);
        let mut detected_bodies = HashMap::default();
        let mut space_rid = Rid::Invalid;
        if let Some(area_rid) = physics_collision_objects.get(&area_rid)
            && let Some(area) = area_rid.get_area()
        {
            detected_bodies = area.state.monitored_objects.clone();
            space_rid = area.get_base().get_space(physics_ids);
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            for (_, monitor_info) in detected_bodies.iter() {
                if let Some(body) = physics_collision_objects
                    .get_mut(&get_id_rid(monitor_info.other_collider_id, physics_ids))
                    && let Some(body) = body.get_mut_body()
                {
                    body.remove_area(*area_id, space);
                }
            }
            // No need to update anymore if it was scheduled before
            space
                .get_mut_state()
                .area_remove_from_area_update_list(*area_id);
        }
    }

    pub fn reset_space_override(
        area_id: &RapierId,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        let area_rid = get_id_rid(*area_id, physics_ids);
        let mut detected_bodies = HashMap::default();
        let mut space_rid: Rid = Rid::Invalid;
        if let Some(area_rid) = physics_collision_objects.get(&area_rid)
            && let Some(area) = area_rid.get_area()
        {
            detected_bodies = area.state.monitored_objects.clone();
            space_rid = area.get_base().get_space(physics_ids);
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            for (_, monitor_info) in detected_bodies {
                if let [Some(body), Some(area)] = physics_collision_objects.get_disjoint_mut([
                    &get_id_rid(monitor_info.other_collider_id, physics_ids),
                    &area_rid,
                ]) && let Some(body) = body.get_mut_body()
                    && let Some(area) = area.get_mut_area()
                {
                    body.remove_area(*area_id, space);
                    body.add_area(area, space);
                }
            }
            space
                .get_mut_state()
                .area_remove_from_area_update_list(*area_id);
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn receive_event(
        &mut self,
        event_state: i32,
        event_removed: bool,
        other_collider_type: CollisionObjectType,
        other_collider_handle: ColliderHandle,
        other_collider: &mut Option<&mut RapierCollisionObject>,
        other_collider_shape: usize,
        other_collider_id: RapierId,
        other_collider_instance_id: u64,
        this_collider_handle: ColliderHandle,
        this_shape: usize,
        space: &mut RapierSpace,
    ) {
        // If the other object is an area:
        if let Some(other_collider) = other_collider
            && let Some(other_area) = other_collider.get_mut_area()
            && (self.area_monitor_callback.is_none() || !other_area.is_monitorable())
        {
            return;
        }
        let event_report = EventReport {
            id: other_collider_id,
            collision_object_type: other_collider_type,
            state: event_state,
            instance_id: other_collider_instance_id,
            object_shape_index: other_collider_shape as u32,
            this_area_shape_index: this_shape as u32,
        };
        // Process the event, creating or destroying a monitor as necessary.
        if let Some(processed_event) = self.process_new_event(
            space,
            this_collider_handle,
            other_collider_handle,
            &event_report,
            event_removed,
        ) {
            // If the other object is a body, we may have to register or unregister this area from the body:
            if let Some(other_collider) = other_collider
                && let Some(other_body) = other_collider.get_mut_body()
            {
                if processed_event.num_contacts == 1 && event_state == 1 {
                    // If we're adding this contact anew, register the area in the body:
                    other_body.add_area(self, space);
                } else if processed_event.num_contacts == 0 {
                    // Else, if our monitor has no more contacts:
                    other_body.remove_area(self.base.get_id(), space);
                }
            }
            if processed_event.should_report {
                self.enqueue_unhandled_event(space, other_collider_handle, event_report);
            }
        }
    }

    // Returns the number of contacts remaining in our monitor for the other collider.
    // If it returns zero, then the monitor has been removed and the collider is exiting.
    // If it returns None, then the event was invalid or stale enough to ignore completely.
    fn process_new_event(
        &mut self,
        space: &mut RapierSpace,
        this_collider_handle: ColliderHandle,
        other_collider_handle: ColliderHandle,
        entry_report: &EventReport,
        event_removed: bool,
    ) -> Option<ProcessedMonitorEvent> {
        let num_contacts = match self.state.process_monitor_event(
            this_collider_handle,
            other_collider_handle,
            entry_report,
            event_removed,
        ) {
            MonitorEventResult::Contacts(num_contacts) => num_contacts,
            MonitorEventResult::StaleRemovedExit => {
                self.filter_on_unhandled_events(space, other_collider_handle, entry_report);
                return None;
            }
            MonitorEventResult::MissingExit => {
                let should_report_exit =
                    self.filter_on_unhandled_events(space, other_collider_handle, entry_report);
                if should_report_exit {
                    godot_warn!(
                        "Area has received an Exit Event for a collider with no recorded Entry Event."
                    );
                }
                return None;
            }
            MonitorEventResult::InvalidState => {
                godot_error!("Event has an invalid state!");
                return None;
            }
        };
        // Edge case to handle the scenario where an object has entered and is now exiting this area in the same frame.
        // As we flush queries every frame, the only way our unhandled event queue will have an entry is if this event occured this frame.
        let should_report =
            self.filter_on_unhandled_events(space, other_collider_handle, entry_report);
        Some(ProcessedMonitorEvent {
            num_contacts,
            should_report,
        })
    }

    // Add an entry event to our queue.
    fn enqueue_unhandled_event(
        &mut self,
        space: &mut RapierSpace,
        other_collider_handle: ColliderHandle,
        new_event: EventReport,
    ) {
        self.state
            .unhandled_events
            .insert(other_collider_handle, new_event);
        // Finally, we tell the space that we have events in our queue.
        space
            .get_mut_state()
            .area_add_to_monitor_query_list(self.base.get_id());
    }

    // Returns true if the new event is required (eg it doesn't contradict or invalidate an existing unhandled event).
    fn filter_on_unhandled_events(
        &mut self,
        space: &mut RapierSpace,
        other_collider_handle: ColliderHandle,
        new_event: &EventReport,
    ) -> bool {
        if let Some(current_event_in_queue) =
            self.state.unhandled_events.get(&other_collider_handle)
        {
            // See if we currently have a pending unhandled event (eg an event from this frame) that is the opposite of this one.
            // If so, this event will cancel that one out.
            if current_event_in_queue.state == -new_event.state {
                self.state.unhandled_events.remove(&other_collider_handle);
                if self.state.unhandled_events.is_empty() {
                    space
                        .get_mut_state()
                        .area_remove_from_monitor_query_list(self.base.get_id());
                }
                return false;
            } else if current_event_in_queue.state == new_event.state {
                // In this scenario, we can just stop here-- the state is already going to report an entry event at the end of this tick.
                return false;
            }
        }
        if let Some(cancelled_collider_handle) = self
            .state
            .find_opposite_unhandled_event_for_shape(other_collider_handle, new_event)
        {
            self.state
                .unhandled_events
                .remove(&cancelled_collider_handle);
            if self.state.unhandled_events.is_empty() {
                space
                    .get_mut_state()
                    .area_remove_from_monitor_query_list(self.base.get_id());
            }
            return false;
        }
        true
    }

    pub fn update_area_override(
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_spaces: &mut PhysicsSpaces,
        physics_engine: &mut PhysicsEngine,
        area_id: &RapierId,
        physics_ids: &PhysicsIds,
    ) {
        let mut monitored_bodies = HashMap::default();
        let mut space_rid = Rid::Invalid;
        let area_rid = get_id_rid(*area_id, physics_ids);
        if let Some(area_rid) = physics_collision_objects.get(&area_rid)
            && let Some(area) = area_rid.get_area()
        {
            monitored_bodies = area.state.monitored_objects.clone();
            space_rid = area.get_base().get_space(physics_ids);
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            space
                .get_mut_state()
                .area_remove_from_area_update_list(*area_id);
        }
        for (_, monitor_info) in &monitored_bodies {
            RapierBody::apply_area_override_to_body(
                &monitor_info.other_collider_id,
                physics_engine,
                physics_spaces,
                physics_collision_objects,
                physics_ids,
            )
        }
    }

    pub fn has_any_space_override(&self) -> bool {
        self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED
            || self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED
            || self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED
    }

    fn queue_area_override_update(
        physics_spaces: &mut PhysicsSpaces,
        space_rid: &Rid,
        area_id: RapierId,
    ) {
        if let Some(space) = physics_spaces.get_mut(space_rid) {
            space.get_mut_state().area_add_to_area_update_list(area_id);
        }
    }

    pub fn set_monitor_callback(
        &mut self,
        callback: Callable,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if callback.is_valid() {
            self.monitor_callback = Some(callback);
        } else {
            self.monitor_callback = None;
        }
        self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
    }

    pub fn set_area_monitor_callback(
        &mut self,
        callback: Callable,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if callback.is_valid() {
            self.area_monitor_callback = Some(callback);
        } else {
            self.area_monitor_callback = None;
        }
        self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
    }

    pub fn set_param(
        &mut self,
        p_param: AreaParameter,
        p_value: Variant,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) -> AreaUpdateMode {
        let id = self.base.get_id();
        let space_rid = self.base.get_space(physics_ids);
        match p_param {
            AreaParameter::GRAVITY_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                let new_gravity_override_mode =
                    AreaSpaceOverrideMode::from_ord(p_value.try_to().unwrap_or_default());
                if self.gravity_override_mode != new_gravity_override_mode {
                    self.gravity_override_mode = new_gravity_override_mode;
                    let has_override = self.has_any_space_override();
                    if has_override != had_override {
                        if has_override {
                            return AreaUpdateMode::EnableSpaceOverride;
                        } else {
                            return AreaUpdateMode::DisableSpaceOverride;
                        }
                    }
                    if has_override {
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }

            AreaParameter::GRAVITY => {
                let new_gravity = variant_to_float(&p_value);
                if new_gravity != self.gravity {
                    self.gravity = new_gravity;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::GRAVITY_VECTOR => {
                let new_gravity_vector = p_value.try_to().unwrap_or_default();
                if self.gravity_vector != new_gravity_vector {
                    self.gravity_vector = new_gravity_vector;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::GRAVITY_IS_POINT => {
                let new_gravity_is_point = p_value.try_to().unwrap_or_default();
                if self.gravity_is_point != new_gravity_is_point {
                    self.gravity_is_point = new_gravity_is_point;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::GRAVITY_POINT_UNIT_DISTANCE => {
                let new_gravity_point_unit_distance = variant_to_float(&p_value);
                if self.gravity_point_unit_distance != new_gravity_point_unit_distance {
                    self.gravity_point_unit_distance = new_gravity_point_unit_distance;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::LINEAR_DAMP_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                let new_linear_damping_override_mode =
                    AreaSpaceOverrideMode::from_ord(p_value.try_to().unwrap_or_default());
                if self.linear_damping_override_mode != new_linear_damping_override_mode {
                    self.linear_damping_override_mode = new_linear_damping_override_mode;
                    let has_override = self.has_any_space_override();
                    if has_override != had_override {
                        if has_override {
                            return AreaUpdateMode::EnableSpaceOverride;
                        } else {
                            return AreaUpdateMode::DisableSpaceOverride;
                        }
                    }
                    if has_override {
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::LINEAR_DAMP => {
                let new_linear_damp = variant_to_float(&p_value);
                if self.linear_damp != new_linear_damp {
                    self.linear_damp = new_linear_damp;
                    if self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                let new_angular_damping_override_mode =
                    AreaSpaceOverrideMode::from_ord(p_value.try_to().unwrap_or_default());
                if self.angular_damping_override_mode != new_angular_damping_override_mode {
                    self.angular_damping_override_mode = new_angular_damping_override_mode;
                    let has_override = self.has_any_space_override();
                    if has_override != had_override {
                        if has_override {
                            return AreaUpdateMode::EnableSpaceOverride;
                        } else {
                            return AreaUpdateMode::DisableSpaceOverride;
                        }
                    }
                    if has_override {
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP => {
                let new_angular_damp = variant_to_float(&p_value);
                if self.angular_damp != new_angular_damp {
                    self.angular_damp = new_angular_damp;
                    if self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        Self::queue_area_override_update(physics_spaces, &space_rid, id);
                    }
                }
            }
            AreaParameter::PRIORITY => {
                let new_priority = p_value.try_to().unwrap_or_default();
                if self.priority != new_priority {
                    self.priority = new_priority;
                    if self.has_any_space_override() {
                        return AreaUpdateMode::ResetSpaceOverride;
                    }
                }
            }
            _ => {}
        }
        AreaUpdateMode::None
    }

    pub fn get_param(&self, p_param: AreaParameter) -> Variant {
        match p_param {
            AreaParameter::GRAVITY_OVERRIDE_MODE => self.gravity_override_mode.to_variant(),
            AreaParameter::GRAVITY => self.gravity.to_variant(),
            AreaParameter::GRAVITY_VECTOR => self.gravity_vector.to_variant(),
            AreaParameter::GRAVITY_IS_POINT => self.gravity_is_point.to_variant(),
            AreaParameter::GRAVITY_POINT_UNIT_DISTANCE => {
                self.gravity_point_unit_distance.to_variant()
            }
            AreaParameter::LINEAR_DAMP_OVERRIDE_MODE => {
                self.linear_damping_override_mode.to_variant()
            }
            AreaParameter::LINEAR_DAMP => self.linear_damp.to_variant(),
            AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE => {
                self.angular_damping_override_mode.to_variant()
            }
            AreaParameter::ANGULAR_DAMP => self.angular_damp.to_variant(),
            AreaParameter::PRIORITY => self.priority.to_variant(),
            _ => Variant::nil(),
        }
    }

    pub fn get_gravity_point_unit_distance(&self) -> real {
        self.gravity_point_unit_distance
    }

    pub fn get_linear_damp(&self) -> real {
        self.linear_damp
    }

    pub fn get_angular_damp(&self) -> real {
        self.angular_damp
    }

    pub fn set_monitorable(
        &mut self,
        monitorable: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        self.monitorable = monitorable;
        self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
    }

    pub fn is_monitorable(&self) -> bool {
        self.monitorable
    }

    pub fn get_priority(&self) -> i32 {
        self.priority
    }

    pub fn call_queries(
        unhandled_events: &HashMap<ColliderHandle, EventReport>,
        monitor_callback: Option<Callable>,
        area_monitor_callback: Option<Callable>,
        physics_ids: &PhysicsIds,
    ) {
        for (_, monitor_report) in unhandled_events {
            if monitor_report.state == 0 {
                godot_error!("Invalid monitor state");
                continue;
            }
            let rid = get_id_rid(monitor_report.id, physics_ids);
            let arg_array = if monitor_report.state > 0 {
                vec![
                    AreaBodyStatus::ADDED.to_variant(),
                    rid.to_variant(),
                    (monitor_report.instance_id as i64).to_variant(),
                    monitor_report.object_shape_index.to_variant(),
                    monitor_report.this_area_shape_index.to_variant(),
                ]
            } else {
                vec![
                    AreaBodyStatus::REMOVED.to_variant(),
                    rid.to_variant(),
                    (monitor_report.instance_id as i64).to_variant(),
                    monitor_report.object_shape_index.to_variant(),
                    monitor_report.this_area_shape_index.to_variant(),
                ]
            };
            if monitor_report.collision_object_type == CollisionObjectType::Body {
                if let Some(ref monitor_callback) = monitor_callback {
                    monitor_callback.call(arg_array.as_slice());
                }
            } else if let Some(ref area_monitor_callback) = area_monitor_callback {
                area_monitor_callback.call(arg_array.as_slice());
            }
        }
    }

    // For state loading, we sometimes need to load a state where a contact present in the pre-load state is no longer extant.
    // In this scenario, the area needs to be manually told to cease monitoring the contact, and Godot must be notified.
    // On the state's next query flush after calling this method, this signal will be emitted.
    pub fn close_stale_contacts(
        &mut self,
        space: &mut RapierSpace,
        stale_collider_pairs: &[ColliderPair],
    ) {
        let mut add_to_monitored_list = false;
        let mut monitors_to_remove: Vec<(ColliderHandle, ColliderHandle)> = Vec::new();
        for ((handle_1, handle_2), monitor_info) in &mut self.state.monitored_objects {
            // If our stale colliders contain a pair in either order:
            let ab = ColliderPair::new(*handle_1, *handle_2);
            let ba = ColliderPair::new(*handle_2, *handle_1);
            if stale_collider_pairs.contains(&ab) || stale_collider_pairs.contains(&ba) {
                add_to_monitored_list = true;
                let new_exit_event = EventReport::from_monitor_info(*monitor_info, -1);
                self.state
                    .unhandled_events
                    .insert(*handle_1, new_exit_event);
                monitors_to_remove.push((*handle_1, *handle_2));
            }
        }
        for removal in monitors_to_remove {
            self.state.monitored_objects.remove(&removal);
        }
        if add_to_monitored_list {
            space
                .get_mut_state()
                .area_add_to_monitor_query_list(self.base.get_id());
        }
    }

    pub fn open_new_contacts(
        &mut self,
        space: &mut RapierSpace,
        new_collider_pairs: &[ColliderPair],
    ) {
        let mut add_to_monitored_list = false;
        for ((handle_1, handle_2), monitor_info) in &mut self.state.monitored_objects {
            // If our stale colliders contain a pair in either order:
            let ab = ColliderPair::new(*handle_1, *handle_2);
            let ba = ColliderPair::new(*handle_2, *handle_1);
            if new_collider_pairs.contains(&ab) || new_collider_pairs.contains(&ba) {
                add_to_monitored_list = true;
                let new_exit_event = EventReport::from_monitor_info(*monitor_info, 1);
                self.state
                    .unhandled_events
                    .insert(*handle_1, new_exit_event);
            }
        }
        if add_to_monitored_list {
            space
                .get_mut_state()
                .area_add_to_monitor_query_list(self.base.get_id());
        }
    }

    pub fn clear_monitored_objects(&mut self) {
        self.state.monitored_objects.clear();
    }

    pub fn clear_event_queue(&mut self) {
        self.state.unhandled_events.clear();
    }

    pub fn compute_gravity(&self, position: Vector) -> Vector {
        if self.gravity_is_point {
            let gr_unit_dist = self.get_gravity_point_unit_distance();
            let v = self.get_base().get_transform() * self.gravity_vector - position;
            if gr_unit_dist > 0.0 {
                let v_length_sq = v.length_squared();
                if v_length_sq > 0.0 {
                    let gravity_strength = self.gravity * gr_unit_dist * gr_unit_dist / v_length_sq;
                    vector_normalized(v) * gravity_strength
                } else {
                    Vector::default()
                }
            } else {
                vector_normalized(v) * self.gravity
            }
        } else {
            self.gravity_vector * self.gravity
        }
    }
}
#[cfg(feature = "serde-serialize")]
impl ExportableObject for RapierArea {
    type ExportState<'a> = AreaExport<'a>;

    fn get_export_state(&self, _: &mut PhysicsEngine) -> Option<Self::ExportState<'_>> {
        Some(AreaExport {
            area_state: &self.state,
            base_state: &self.base.state,
        })
    }

    fn import_state(&mut self, _: &mut PhysicsEngine, data: ObjectImportState) {
        match data {
            bodies::exportable_object::ObjectImportState::Area(area_import) => {
                self.state = area_import.area_state;
                self.base.state = area_import.base_state;
            }
            _ => {
                godot_error!("Attempted to import invalid state data.");
            }
        }
    }
}
// We won't use the pointers between threads, so it should be safe.
unsafe impl Sync for RapierArea {}
impl IRapierCollisionObject for RapierArea {
    fn get_base(&self) -> &RapierCollisionObjectBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierCollisionObjectBase {
        &mut self.base
    }

    fn get_body(&self) -> Option<&RapierBody> {
        None
    }

    fn get_area(&self) -> Option<&RapierArea> {
        Some(self)
    }

    fn get_mut_body(&mut self) -> Option<&mut RapierBody> {
        None
    }

    fn get_mut_area(&mut self) -> Option<&mut RapierArea> {
        Some(self)
    }

    fn set_space(
        &mut self,
        p_space: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &mut PhysicsIds,
    ) {
        if p_space == self.base.get_space(physics_ids) {
            return;
        }
        self.base
            .set_space(p_space, physics_engine, physics_spaces, physics_ids);
        self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
    }

    fn add_shape(
        &mut self,
        p_shape_id: RapierId,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::add_shape(
            self,
            p_shape_id,
            p_transform,
            p_disabled,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_ids,
        );
    }

    fn set_shape(
        &mut self,
        p_index: usize,
        p_shape: ShapeHandle,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::set_shape(
            self,
            p_index,
            p_shape,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_ids,
        );
    }

    fn set_shape_transform(
        &mut self,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::set_shape_transform(
            self,
            p_index,
            p_transform,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
    }

    fn set_shape_disabled(
        &mut self,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::set_shape_disabled(
            self,
            p_index,
            p_disabled,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
    }

    fn remove_shape_rid(
        &mut self,
        shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.base.state.shapes.len() {
            if get_id_rid(self.base.state.shapes[i].id, physics_ids) == shape {
                self.remove_shape_idx(
                    i,
                    physics_engine,
                    physics_spaces,
                    physics_shapes,
                    physics_ids,
                );
            } else {
                i += 1;
            }
        }
    }

    fn remove_shape_idx(
        &mut self,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::remove_shape_idx(
            self,
            p_index,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_ids,
        );
    }

    fn create_shape(
        &mut self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_engine: &mut PhysicsEngine,
    ) -> ColliderHandle {
        if !self.base.is_valid() {
            return ColliderHandle::invalid();
        }
        let mat = self.init_material();
        self.base
            .create_shape(shape, p_shape_index, mat, physics_engine)
    }

    fn init_material(&self) -> Material {
        Material::new(
            self.base.get_collision_layer(),
            self.base.get_collision_mask(),
            self.base.get_dominance(),
        )
    }

    fn recreate_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::recreate_shapes(
            self,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
    }

    fn shape_changed(
        &mut self,
        p_shape_id: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::shape_changed(
            self,
            p_shape_id,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
    }

    fn shapes_changed(
        &mut self,
        _physics_engine: &mut PhysicsEngine,
        _physics_spaces: &mut PhysicsSpaces,
        _physics_ids: &PhysicsIds,
    ) {
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn collider_handle(index: u32) -> ColliderHandle {
        ColliderHandle::from_raw_parts(index, 0)
    }
    fn event_report(state: i32) -> EventReport {
        EventReport {
            id: 42,
            collision_object_type: CollisionObjectType::Area,
            state,
            instance_id: 7,
            object_shape_index: 0,
            this_area_shape_index: 0,
        }
    }
    #[test]
    fn stale_removed_exit_without_monitor_is_ignored() {
        let mut state = RapierAreaState::default();
        let result = state.process_monitor_event(
            collider_handle(1),
            collider_handle(2),
            &event_report(-1),
            true,
        );
        assert_eq!(result, MonitorEventResult::StaleRemovedExit);
        assert!(state.monitored_objects.is_empty());
    }
    #[test]
    fn non_removed_exit_without_monitor_is_reported_as_missing() {
        let mut state = RapierAreaState::default();
        let result = state.process_monitor_event(
            collider_handle(1),
            collider_handle(2),
            &event_report(-1),
            false,
        );
        assert_eq!(result, MonitorEventResult::MissingExit);
    }
    #[test]
    fn monitor_event_lifetime_counts_contacts() {
        let mut state = RapierAreaState::default();
        let this_collider = collider_handle(1);
        let other_collider = collider_handle(2);
        assert_eq!(
            state.process_monitor_event(this_collider, other_collider, &event_report(1), false),
            MonitorEventResult::Contacts(1)
        );
        assert_eq!(
            state.process_monitor_event(this_collider, other_collider, &event_report(1), false),
            MonitorEventResult::Contacts(2)
        );
        assert_eq!(
            state.process_monitor_event(this_collider, other_collider, &event_report(-1), false),
            MonitorEventResult::Contacts(1)
        );
        assert_eq!(
            state.process_monitor_event(this_collider, other_collider, &event_report(-1), false),
            MonitorEventResult::Contacts(0)
        );
        assert!(
            !state
                .monitored_objects
                .contains_key(&(other_collider, this_collider))
        );
    }
    #[test]
    fn unhandled_opposite_event_matches_recreated_collider_for_same_shape_pair() {
        let mut state = RapierAreaState::default();
        let removed_collider = collider_handle(2);
        let added_collider = collider_handle(3);
        state
            .unhandled_events
            .insert(removed_collider, event_report(-1));
        assert_eq!(
            state.find_opposite_unhandled_event_for_shape(added_collider, &event_report(1)),
            Some(removed_collider)
        );
    }
    #[test]
    fn unhandled_opposite_event_ignores_different_shape_pair() {
        let mut state = RapierAreaState::default();
        let removed_collider = collider_handle(2);
        let added_collider = collider_handle(3);
        state
            .unhandled_events
            .insert(removed_collider, event_report(-1));
        let mut different_shape_event = event_report(1);
        different_shape_event.object_shape_index = 1;
        assert_eq!(
            state.find_opposite_unhandled_event_for_shape(added_collider, &different_shape_event),
            None
        );
    }
    #[cfg(feature = "serde-serialize")]
    #[test]
    fn area_state_does_not_serialize_unhandled_events() {
        let mut state = RapierAreaState::default();
        state
            .unhandled_events
            .insert(collider_handle(2), event_report(1));
        let json = serde_json::to_string(&state).unwrap();
        assert!(!json.contains("unhandled_events"));
        let imported_state: RapierAreaState = serde_json::from_str(&json).unwrap();
        assert!(imported_state.unhandled_events.is_empty());
    }
}
