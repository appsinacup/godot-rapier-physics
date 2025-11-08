
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

use super::rapier_body::RapierBody;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::collider;
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
pub struct MonitorInfo {
    pub id: RapierId,
    pub instance_id: u64,
    pub object_shape_index: u32,
    pub area_shape_index: u32,
    pub collision_object_type: CollisionObjectType,
    pub num_contacts: i32,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
// This struct contains the data that will get reported to Godot in case of an entry/exit event.
// It's designed to be temporary, and destroyed after the signal is sent.
pub struct MonitorReport {
    pub id: RapierId,
    pub collision_object_type: CollisionObjectType,
    pub state: i32,
    pub instance_id: u64,
    pub object_shape_index: u32,
    pub area_shape_index: u32,
}

impl MonitorReport {
    pub fn from_monitor_info(from_info: MonitorInfo, state_in: i32) -> Self 
    {
        Self{
            state: state_in,
            id: from_info.id.clone(),
            collision_object_type: from_info.collision_object_type.clone(),
            instance_id: from_info.instance_id.clone(),
            object_shape_index: from_info.object_shape_index.clone(),
            area_shape_index: from_info.area_shape_index.clone(),
        }
    }
}

pub enum AreaUpdateMode {
    EnableSpaceOverride,
    DisableSpaceOverride,
    ResetSpaceOverride,
    None,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
pub struct AreaExport<'a> {
    area_state: &'a RapierAreaState,
    base_state: &'a RapierCollisionObjectBaseState,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize))]
pub struct AreaImport {
    area_state: RapierAreaState,
    base_state: RapierCollisionObjectBaseState,
}
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierAreaState {
    // New events go into this queue; once handled (eg once reported to Godot), they are removed.
    pub unhandled_event_queue: HashMap<ColliderHandle, MonitorReport>,
    // This is a persistent list of all current contacts. Entries only disappear when contact ceases, or when contacts are manually cleared.
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
            
            //EDITED
            for (_, monitor_info) in detected_bodies.iter() {
                if let Some([body, area]) = physics_collision_objects
                    .get_many_mut([&get_id_rid(monitor_info.id, physics_ids), &area_rid])
                    && let Some(body) = body.get_mut_body()
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
                if let Some(body) =
                    physics_collision_objects.get_mut(&get_id_rid(monitor_info.id, physics_ids))
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
                if let Some([body, area]) = physics_collision_objects
                    .get_many_mut([&get_id_rid(monitor_info.id, physics_ids), &area_rid])
                    && let Some(body) = body.get_mut_body()
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
    pub fn on_body_enter(
        &mut self,
        other_collider_handle: ColliderHandle,
        body: &mut Option<&mut RapierCollisionObject>,
        body_shape: usize,
        body_id: RapierId,
        body_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        let new_entry = self.process_entry(
            area_collider_handle, 
            area_shape,
            other_collider_handle,
            body_shape, 
            body_id, 
            body_instance_id,
            CollisionObjectType::Body,
        );

        if new_entry {            
            if let Some(body) = body {
                if let Some(body) = body.get_mut_body() {
                    body.add_area(self, space);
                }
            } else {
                godot_error!("Body not found when entering area");
            }

            // If we have no monitoring callback, then no worries-- we won't report this event.
            if self.monitor_callback.is_none() {
                return;
            }

            // Edge case here handles the scenario where an object has exited this area this frame, and is also entering it again--
            // in this scenario, we want to clear the "exit" event from the queue and, if there are no other unhandled events, remove this area from the space's update list.
            if !self.filter_on_unhandled_events(
                space,
                other_collider_handle, 
                1
            ){
                return;
            }

            // Add an entry event to our queue.            
            self.enqueue_unhandled_event(
                space, 
                other_collider_handle, 
                MonitorReport {
                    id: body_id,
                    collision_object_type: CollisionObjectType::Body,
                    state: 1,
                    instance_id: body_instance_id,
                    object_shape_index: body_shape as u32,
                    area_shape_index: area_shape as u32,
                }
            );
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_body_exit(
        &mut self,
        other_collider_handle: ColliderHandle,
        body: &mut Option<&mut RapierCollisionObject>,
        body_shape: usize,
        body_id: RapierId,
        body_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        // Ensure that this body is actually in our list of monitored objects.
        match self.state.monitored_objects.get_mut(&(other_collider_handle, area_collider_handle)) {
            None => {
                return;
            }
            Some(monitor_info) => {
                monitor_info.num_contacts -= 1;
                if monitor_info.num_contacts == 0 {
                    // If we're no longer contacting any shapes from this body:
                    self.state.monitored_objects.remove(&(other_collider_handle, area_collider_handle));

                    // Tell the body to remove this area.
                    if let Some(body) = body
                        && let Some(body) = body.get_mut_body()
                    {
                        body.remove_area(self.base.get_id(), space);
                    }

                    // If we're not monitoring, return now.
                    if self.monitor_callback.is_none() {
                        return;
                    }

                    // Edge case to handle the scenario where an object has entered and is now exiting this area in the same frame.
                    // As we flush queries every frame, the only way our unhandled event queue will have an entry is if this event occured this frame.
                    if !self.filter_on_unhandled_events(
                        space,
                        other_collider_handle, 
                        -1
                    ){
                        return;
                    }

                    // Add an exit event to our queue.            
                    self.enqueue_unhandled_event(
                        space, 
                        other_collider_handle, 
                        MonitorReport {
                            id: body_id,
                            collision_object_type: CollisionObjectType::Body,
                            state: -1,
                            instance_id: body_instance_id,
                            object_shape_index: body_shape as u32,
                            area_shape_index: area_shape as u32,
                        }
                    );
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_area_enter(
        &mut self,
        other_collider_handle: ColliderHandle,
        other_area: &mut Option<&mut RapierCollisionObject>,
        other_area_shape: usize,
        other_area_id: RapierId,
        other_area_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        if self.area_monitor_callback.is_none() {
            return;
        }
        
        if let Some(other_area) = other_area
            && let Some(other_area) = other_area.get_mut_area()
            && !other_area.is_monitorable()
        {
            return;
        }

        let new_entry = self.process_entry(
            area_collider_handle, 
            area_shape,
            other_collider_handle,
            other_area_shape, 
            other_area_id, 
            other_area_instance_id,
            CollisionObjectType::Area,
        );

        if new_entry {
            // Check the edge case for an area entering on this frame that also exited earlier this frame.
            if !self.filter_on_unhandled_events(
                space,
                other_collider_handle, 
                1
            ) {
                return;
            }

            // Add an entry event to our queue.      
            self.enqueue_unhandled_event(
                space, 
                other_collider_handle, 
                MonitorReport {
                    id: other_area_id,
                    collision_object_type: CollisionObjectType::Area,
                    state: 1,
                    instance_id: other_area_instance_id,
                    object_shape_index: other_area_shape as u32,
                    area_shape_index: area_shape as u32,
                }
            );
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_area_exit(
        &mut self,
        other_collider_handle: ColliderHandle,
        other_area: &mut Option<&mut RapierCollisionObject>,
        other_area_shape: usize,
        other_area_id: RapierId,
        other_area_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        if self.area_monitor_callback.is_none() {
            return;
        }

        if let Some(other_area) = other_area
            && let Some(other_area) = other_area.get_mut_area()
            && !other_area.is_monitorable()
        {
            return;
        }

        match self.state.monitored_objects.get_mut(&(other_collider_handle, area_collider_handle)) {
            None => {
                return;
            }
            Some(monitor_info) => {
                monitor_info.num_contacts -= 1;
                if monitor_info.num_contacts == 0 {
                    // If we're no longer contacting any shapes from this area:
                    self.state.monitored_objects.remove(&(other_collider_handle, area_collider_handle));

                    // Edge case to handle the scenario where an object has entered and is now exiting this area in the same frame.
                    // As we flush queries every frame, the only way our unhandled event queue will have an entry is if this event occured this frame.
                    if !self.filter_on_unhandled_events(
                        space,
                        other_collider_handle, 
                        -1
                    ){
                        return;
                    }

                    // Add an exit event to our queue.            
                    self.enqueue_unhandled_event(
                        space, 
                        other_collider_handle, 
                        MonitorReport {
                            id: other_area_id,
                            collision_object_type: CollisionObjectType::Area,
                            state: -1,
                            instance_id: other_area_instance_id,
                            object_shape_index: other_area_shape as u32,
                            area_shape_index: area_shape as u32,
                        }
                    );
                }
            }
        }
    }

    // Considers a new 
    // Returns true if the entry is new, false if we're already monitoring the body.
    fn process_entry(
        &mut self,
        this_collider_handle: ColliderHandle,
        this_collider_shape: usize,
        other_collider_handle: ColliderHandle,
        other_collider_shape: usize,
        other_collider_id: RapierId,
        other_collider_instance_id: u64,
        other_collider_type: CollisionObjectType,     
    ) -> bool {
        if let Some(monitor_info) = self
        .state
        .monitored_objects
        .get_mut(&(other_collider_handle, this_collider_handle)) {
            monitor_info.num_contacts += 1;            
            return false;
        } else {
            // Otherwise, this is a totally new contact. Start a new contact monitor and dispatch an event to Godot.
            let handle_pair = (other_collider_handle, this_collider_handle);
            let new_monitor_info = MonitorInfo {
                id: other_collider_id,
                instance_id: other_collider_instance_id,
                object_shape_index: other_collider_shape as u32,
                area_shape_index: this_collider_shape as u32,
                collision_object_type: other_collider_type,
                num_contacts: 1,
            };
            
            self.state.monitored_objects.insert(handle_pair, new_monitor_info);            
            return true;
        }
    }

    // Add an entry event to our queue.
    fn enqueue_unhandled_event(
        &mut self,
        space: &mut RapierSpace,
        other_collider_handle: ColliderHandle,
        new_event: MonitorReport,
    ){   
        self.state.unhandled_event_queue.insert(other_collider_handle, new_event);
      
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
        new_event_state: i32,
    ) -> bool {
        if let Some(current_event_in_queue) = self.state.unhandled_event_queue.get(&other_collider_handle) {
            // See if we currently have a pending unhandled event (eg an event from this frame) that is the opposite of this one.
            // If so, this event will cancel that one out.
            if current_event_in_queue.state == new_event_state * -1 {
                self.state.unhandled_event_queue.remove(&other_collider_handle);

                if self.state.unhandled_event_queue.len() == 0 {
                    space
                    .get_mut_state()
                    .area_remove_from_area_update_list(self.base.get_id());
                    return false;
                }

                
            } else if current_event_in_queue.state == new_event_state {
                // In this scenario, we can just stop here-- the state is already going to report an entry event at the end of this tick.
                return false;
            }        
        }
        
        return true;
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
                &monitor_info.id,
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
                self.gravity_override_mode =
                    AreaSpaceOverrideMode::from_ord(p_value.try_to().unwrap_or_default());
                let has_override = self.has_any_space_override();
                if has_override != had_override {
                    if has_override {
                        return AreaUpdateMode::EnableSpaceOverride;
                    } else {
                        return AreaUpdateMode::DisableSpaceOverride;
                    }
                }
            }

            AreaParameter::GRAVITY => {
                let new_gravity = variant_to_float(&p_value);
                if new_gravity != self.gravity {
                    self.gravity = new_gravity;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) = physics_spaces.get_mut(&space_rid) {
                            space.get_mut_state().area_add_to_area_update_list(id);
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_VECTOR => {
                let new_gravity_vector = p_value.try_to().unwrap_or_default();
                if self.gravity_vector != new_gravity_vector {
                    self.gravity_vector = new_gravity_vector;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_ids))
                        {
                            space.get_mut_state().area_add_to_area_update_list(id);
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_IS_POINT => {
                let new_gravity_is_point = p_value.try_to().unwrap_or_default();
                if self.gravity_is_point != new_gravity_is_point {
                    self.gravity_is_point = new_gravity_is_point;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) = physics_spaces.get_mut(&space_rid) {
                            space.get_mut_state().area_add_to_area_update_list(id);
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_POINT_UNIT_DISTANCE => {
                let new_gravity_point_unit_distance = variant_to_float(&p_value);
                if self.gravity_point_unit_distance != new_gravity_point_unit_distance {
                    self.gravity_point_unit_distance = new_gravity_point_unit_distance;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) = physics_spaces.get_mut(&space_rid) {
                            space.get_mut_state().area_add_to_area_update_list(id);
                        }
                    }
                }
            }
            AreaParameter::LINEAR_DAMP_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                self.linear_damping_override_mode =
                    AreaSpaceOverrideMode::from_ord(p_value.try_to().unwrap_or_default());
                let has_override = self.has_any_space_override();
                if has_override != had_override {
                    if has_override {
                        return AreaUpdateMode::EnableSpaceOverride;
                    } else {
                        return AreaUpdateMode::DisableSpaceOverride;
                    }
                }
            }
            AreaParameter::LINEAR_DAMP => {
                let new_linear_damp = variant_to_float(&p_value);
                if self.linear_damp != new_linear_damp {
                    self.linear_damp = new_linear_damp;
                    if self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) = physics_spaces.get_mut(&space_rid) {
                            space.get_mut_state().area_add_to_area_update_list(id);
                        }
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                self.angular_damping_override_mode =
                    AreaSpaceOverrideMode::from_ord(p_value.try_to().unwrap_or_default());
                let has_override = self.has_any_space_override();
                if has_override != had_override {
                    if has_override {
                        return AreaUpdateMode::EnableSpaceOverride;
                    } else {
                        return AreaUpdateMode::DisableSpaceOverride;
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP => {
                let new_angular_damp = variant_to_float(&p_value);
                if self.angular_damp != new_angular_damp {
                    self.angular_damp = new_angular_damp;
                    if self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) = physics_spaces.get_mut(&space_rid) {
                            space.get_mut_state().area_add_to_area_update_list(id);
                        }
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
        unhandled_events: &HashMap<ColliderHandle, MonitorReport>,
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
                    monitor_report.instance_id.to_variant(),
                    monitor_report.object_shape_index.to_variant(),
                    monitor_report.area_shape_index.to_variant(),
                ]
            } else {
                vec![
                    AreaBodyStatus::REMOVED.to_variant(),
                    rid.to_variant(),
                    monitor_report.instance_id.to_variant(),
                    monitor_report.object_shape_index.to_variant(),
                    monitor_report.area_shape_index.to_variant(),
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
        stale_collider_pairs: &Vec<ColliderPair>,
    )
    {
        let mut add_to_monitored_list = false;
        let mut monitors_to_remove: Vec<(ColliderHandle, ColliderHandle)> = Vec::new();
        for ((handle_1, handle_2), monitor_info) in &mut self.state.monitored_objects
        {
            // If our stale colliders contain a pair in either order:
            let ab = ColliderPair::new(*handle_1, *handle_2);
            let ba = ColliderPair::new(*handle_2, *handle_1);
            if stale_collider_pairs.contains(&ab)
                || stale_collider_pairs.contains(&ba)
            {
                add_to_monitored_list = true;
                let new_exit_event = MonitorReport::from_monitor_info(*monitor_info, -1);
                self.state.unhandled_event_queue.insert(*handle_1, new_exit_event);
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

    pub fn clear_monitored_objects(&mut self) {
        self.state.monitored_objects.clear();
    }

    pub fn clear_event_queue(&mut self) {
        self.state.unhandled_event_queue.clear();
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

    #[cfg(feature = "serde-serialize")]
    fn export_json(&self) -> String {
        let state = AreaExport {
            area_state: &self.state,
            base_state: &self.base.state,
        };
        match serde_json::to_string_pretty(&state) {
            Ok(s) => {
                return s;
            }
            Err(e) => {
                godot_error!("Failed to serialize area to json: {}", e);
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    fn export_binary(&self) -> PackedByteArray {
        let mut buf = PackedByteArray::new();
        let state = AreaExport {
            area_state: &self.state,
            base_state: &self.base.state,
        };
        match bincode::serialize(&state) {
            Ok(binary_data) => {
                buf.resize(binary_data.len());
                for i in 0..binary_data.len() {
                    buf[i] = binary_data[i];
                }
            }
            Err(e) => {
                godot_error!("Failed to serialize area to binary: {}", e);
            }
        }
        buf
    }

    #[cfg(feature = "serde-serialize")]
    fn import_binary(&mut self, data: PackedByteArray) {
        match bincode::deserialize::<AreaImport>(data.as_slice()) {
            Ok(import) => {
                self.state = import.area_state;
                self.base.state = import.base_state;
            }
            Err(e) => {
                godot_error!("Failed to deserialize area from binary: {}", e);
            }
        }
    }
}
