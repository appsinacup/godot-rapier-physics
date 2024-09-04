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
use rapier::prelude::RigidBodyHandle;
use servers::rapier_physics_singleton::get_body_rid;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;
use servers::rapier_physics_singleton::PhysicsRids;
use servers::rapier_physics_singleton::PhysicsShapes;
use servers::rapier_physics_singleton::PhysicsSpaces;

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
struct MonitorInfo {
    // TODO set this correct after import
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "default_rid"))]
    pub rid: Rid,
    pub instance_id: u64,
    pub object_shape_index: u32,
    pub area_shape_index: u32,
    pub collision_object_type: CollisionObjectType,
    pub state: i32,
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
    monitored_objects: HashMap<(ColliderHandle, ColliderHandle), MonitorInfo>,
    detected_bodies: HashMap<RigidBodyHandle, u32>,
    detected_areas: HashMap<RigidBodyHandle, u32>,
}
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
    monitor_callback: Option<Callable>,
    area_monitor_callback: Option<Callable>,
    state: RapierAreaState,
    base: RapierCollisionObjectBase,
}
impl RapierArea {
    pub fn new(rid: Rid) -> Self {
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
            base: RapierCollisionObjectBase::new(rid, CollisionObjectType::Area),
        }
    }

    pub fn enable_space_override(
        area_handle: &RigidBodyHandle,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_rids: &PhysicsRids,
    ) {
        let area_rid = get_body_rid(*area_handle, physics_rids);
        let mut detected_bodies = HashMap::default();
        let mut space_rid = Rid::Invalid;
        if let Some(area_rid) = physics_collision_objects.get(&area_rid) {
            if let Some(area) = area_rid.get_area() {
                detected_bodies = area.state.detected_bodies.clone();
                space_rid = area.get_base().get_space(physics_rids);
            }
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            for (key, _) in detected_bodies.iter() {
                if let Some([body, area]) = physics_collision_objects
                    .get_many_mut([&get_body_rid(*key, physics_rids), &area_rid])
                    && let Some(body) = body.get_mut_body()
                    && let Some(area) = area.get_mut_area()
                {
                    body.add_area(area, space);
                }
            }
            // No need to update anymore if it was scheduled before
            space.area_remove_from_area_update_list(*area_handle);
        }
    }

    pub fn disable_space_override(
        area_handle: &RigidBodyHandle,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_rids: &PhysicsRids,
    ) {
        let area_rid = get_body_rid(*area_handle, physics_rids);
        let mut detected_bodies = HashMap::default();
        let mut space_rid = Rid::Invalid;
        if let Some(area_rid) = physics_collision_objects.get(&area_rid) {
            if let Some(area) = area_rid.get_area() {
                detected_bodies = area.state.detected_bodies.clone();
                space_rid = area.get_base().get_space(physics_rids);
            }
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            for (key, _) in detected_bodies.iter() {
                if let Some(body) =
                    physics_collision_objects.get_mut(&get_body_rid(*key, physics_rids))
                    && let Some(body) = body.get_mut_body()
                {
                    body.remove_area(*area_handle, space);
                }
            }
            // No need to update anymore if it was scheduled before
            space.area_remove_from_area_update_list(*area_handle);
        }
    }

    pub fn reset_space_override(
        area_handle: &RigidBodyHandle,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_rids: &PhysicsRids,
    ) {
        let area_rid = get_body_rid(*area_handle, physics_rids);
        let mut detected_bodies = HashMap::default();
        let mut space_rid: Rid = Rid::Invalid;
        if let Some(area_rid) = physics_collision_objects.get(&area_rid) {
            if let Some(area) = area_rid.get_area() {
                detected_bodies = area.state.detected_bodies.clone();
                space_rid = area.get_base().get_space(physics_rids);
            }
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            for (key, _) in detected_bodies {
                if let Some([body, area]) = physics_collision_objects
                    .get_many_mut([&get_body_rid(key, physics_rids), &area_rid])
                    && let Some(body) = body.get_mut_body()
                    && let Some(area) = area.get_mut_area()
                {
                    body.remove_area(*area_handle, space);
                    body.add_area(area, space);
                }
            }
            space.area_remove_from_area_update_list(*area_handle);
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_body_enter(
        &mut self,
        collider_handle: ColliderHandle,
        body: &mut Option<&mut RapierCollisionObject>,
        body_shape: usize,
        body_rid: Rid,
        body_handle: RigidBodyHandle,
        body_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        // Add to keep track of currently detected bodies
        if let Some(detected_body) = self.state.detected_bodies.get_mut(&body_handle) {
            *detected_body += 1;
        } else {
            self.state.detected_bodies.insert(body_handle, 1);
            if let Some(body) = body {
                if let Some(body) = body.get_mut_body() {
                    body.add_area(self, space);
                }
            } else {
                godot_error!("Body not found when entering area");
            }
        }
        if self.monitor_callback.is_none() {
            return;
        }
        let handle_pair_hash = (collider_handle, area_collider_handle);
        if let Some(monitored_object) = self.state.monitored_objects.get(&handle_pair_hash) {
            // in case it already exited this frame and now it enters, cancel out the event
            if monitored_object.state != -1 {
                godot_error!("Body has not exited the area");
            }
            self.state.monitored_objects.remove(&handle_pair_hash);
        } else {
            self.state.monitored_objects.insert(
                handle_pair_hash,
                MonitorInfo {
                    rid: body_rid,
                    instance_id: body_instance_id,
                    object_shape_index: body_shape as u32,
                    area_shape_index: area_shape as u32,
                    collision_object_type: CollisionObjectType::Body,
                    state: 1,
                },
            );
            space.area_add_to_monitor_query_list(self.base.get_body_handle());
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_body_exit(
        &mut self,
        collider_handle: ColliderHandle,
        body: &mut Option<&mut RapierCollisionObject>,
        body_shape: usize,
        body_rid: Rid,
        body_handle: RigidBodyHandle,
        body_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        // Remove from currently detected bodies
        if let Some(detected_body) = self.state.detected_bodies.get_mut(&body_handle) {
            *detected_body -= 1;
            if *detected_body == 0 {
                self.state.detected_bodies.remove(&body_handle);
                if let Some(body) = body {
                    if let Some(body) = body.get_mut_body() {
                        body.remove_area(self.base.get_body_handle(), space);
                    }
                }
            }
        }
        if self.monitor_callback.is_none() {
            return;
        }
        let handle_pair_hash = (collider_handle, area_collider_handle);
        if let Some(monitored_object) = self.state.monitored_objects.get(&handle_pair_hash) {
            // in case it already entered this frame and now it exits, cancel out the event
            if monitored_object.state != 1 {
                godot_error!("Body has not entered the area");
            }
            self.state.monitored_objects.remove(&handle_pair_hash);
        } else {
            self.state.monitored_objects.insert(
                handle_pair_hash,
                MonitorInfo {
                    rid: body_rid,
                    instance_id: body_instance_id,
                    object_shape_index: body_shape as u32,
                    area_shape_index: area_shape as u32,
                    collision_object_type: CollisionObjectType::Body,
                    state: -1,
                },
            );
            space.area_add_to_monitor_query_list(self.base.get_body_handle());
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_area_enter(
        &mut self,
        collider_handle: ColliderHandle,
        other_area: &mut Option<&mut RapierCollisionObject>,
        other_area_shape: usize,
        other_area_rid: Rid,
        other_area_handle: RigidBodyHandle,
        other_area_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        if self.area_monitor_callback.is_none() {
            return;
        }
        if let Some(other_area) = other_area {
            if let Some(other_area) = other_area.get_mut_area() {
                if !other_area.is_monitorable() {
                    return;
                }
            }
        } else {
            godot_error!("other area is null");
        }
        // Add to keep track of currently detected areas
        if let Some(detected_area) = self.state.detected_areas.get_mut(&other_area_handle) {
            *detected_area += 1;
        } else {
            self.state.detected_areas.insert(other_area_handle, 1);
        }
        let handle_pair_hash = (collider_handle, area_collider_handle);
        if let Some(monitored_object) = self.state.monitored_objects.get(&handle_pair_hash) {
            // in case it already exited this frame and now it enters, cancel out the event
            if monitored_object.state != -1 {
                godot_error!("Area is already being monitored");
            }
            self.state.monitored_objects.remove(&handle_pair_hash);
        } else {
            self.state.monitored_objects.insert(
                handle_pair_hash,
                MonitorInfo {
                    rid: other_area_rid,
                    instance_id: other_area_instance_id,
                    object_shape_index: other_area_shape as u32,
                    area_shape_index: area_shape as u32,
                    collision_object_type: CollisionObjectType::Area,
                    state: 1,
                },
            );
            space.area_add_to_monitor_query_list(self.base.get_body_handle());
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_area_exit(
        &mut self,
        collider_handle: ColliderHandle,
        other_area: &mut Option<&mut RapierCollisionObject>,
        other_area_shape: usize,
        other_area_rid: Rid,
        other_area_handle: RigidBodyHandle,
        other_area_instance_id: u64,
        area_collider_handle: ColliderHandle,
        area_shape: usize,
        space: &mut RapierSpace,
    ) {
        if self.area_monitor_callback.is_none() {
            return;
        }
        if let Some(other_area) = other_area {
            if let Some(other_area) = other_area.get_mut_area() {
                if !other_area.is_monitorable() {
                    return;
                }
            }
        }
        // Remove from currently detected areas
        if let Some(detected_area) = self.state.detected_areas.get_mut(&other_area_handle) {
            *detected_area -= 1;
            if *detected_area == 0 {
                self.state.detected_areas.remove(&other_area_handle);
            }
        } else {
            return;
        }
        let handle_pair_hash = (collider_handle, area_collider_handle);
        if let Some(monitored_object) = self.state.monitored_objects.get(&handle_pair_hash) {
            if monitored_object.state != 1 {
                godot_error!("Area is not being monitored");
            }
            self.state.monitored_objects.remove(&handle_pair_hash);
        } else {
            self.state.monitored_objects.insert(
                handle_pair_hash,
                MonitorInfo {
                    rid: other_area_rid,
                    instance_id: other_area_instance_id,
                    object_shape_index: other_area_shape as u32,
                    area_shape_index: area_shape as u32,
                    collision_object_type: CollisionObjectType::Area,
                    state: -1,
                },
            );
            space.area_add_to_monitor_query_list(self.base.get_body_handle());
        }
    }

    pub fn update_area_override(
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_spaces: &mut PhysicsSpaces,
        physics_engine: &mut PhysicsEngine,
        area_handle: &RigidBodyHandle,
        physics_rids: &PhysicsRids,
    ) {
        let mut detected_bodies = HashMap::default();
        let mut space_rid = Rid::Invalid;
        let area_rid = get_body_rid(*area_handle, physics_rids);
        if let Some(area_rid) = physics_collision_objects.get(&area_rid) {
            if let Some(area) = area_rid.get_area() {
                detected_bodies = area.state.detected_bodies.clone();
                space_rid = area.get_base().get_space(physics_rids);
            }
        }
        if let Some(space) = physics_spaces.get_mut(&space_rid) {
            space.area_remove_from_area_update_list(*area_handle);
        }
        for (detected_body, _) in &detected_bodies {
            RapierBody::apply_area_override_to_body(
                detected_body,
                physics_engine,
                physics_spaces,
                physics_collision_objects,
                physics_rids,
            );
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
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        if callback.is_valid() {
            self.monitor_callback = Some(callback);
        } else {
            self.monitor_callback = None;
        }
        self.recreate_shapes(physics_engine, physics_shapes, physics_spaces, physics_rids);
    }

    pub fn set_area_monitor_callback(
        &mut self,
        callback: Callable,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        if callback.is_valid() {
            self.area_monitor_callback = Some(callback);
        } else {
            self.area_monitor_callback = None;
        }
        self.recreate_shapes(physics_engine, physics_shapes, physics_spaces, physics_rids);
    }

    pub fn set_param(
        &mut self,
        p_param: AreaParameter,
        p_value: Variant,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) -> AreaUpdateMode {
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
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_rids))
                        {
                            space.area_add_to_area_update_list(self.base.get_body_handle());
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
                            physics_spaces.get_mut(&self.base.get_space(physics_rids))
                        {
                            space.area_add_to_area_update_list(self.base.get_body_handle());
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
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_rids))
                        {
                            space.area_add_to_area_update_list(self.base.get_body_handle());
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
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_rids))
                        {
                            space.area_add_to_area_update_list(self.base.get_body_handle());
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
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_rids))
                        {
                            space.area_add_to_area_update_list(self.base.get_body_handle());
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
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_rids))
                        {
                            space.area_add_to_area_update_list(self.base.get_body_handle());
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
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        self.monitorable = monitorable;
        self.recreate_shapes(physics_engine, physics_shapes, physics_spaces, physics_rids);
    }

    pub fn is_monitorable(&self) -> bool {
        self.monitorable
    }

    pub fn get_priority(&self) -> i32 {
        self.priority
    }

    pub fn get_queries(&self) -> Vec<(Callable, Vec<Variant>)> {
        let mut queries = Vec::default();
        if self.state.monitored_objects.is_empty() {
            return queries;
        }
        for (_, monitor_info) in &self.state.monitored_objects {
            if monitor_info.state == 0 {
                godot_error!("Invalid monitor state");
                continue;
            }
            let arg_array = if monitor_info.state > 0 {
                vec![
                    AreaBodyStatus::ADDED.to_variant(),
                    monitor_info.rid.to_variant(),
                    monitor_info.instance_id.to_variant(),
                    monitor_info.object_shape_index.to_variant(),
                    monitor_info.area_shape_index.to_variant(),
                ]
            } else {
                vec![
                    AreaBodyStatus::REMOVED.to_variant(),
                    monitor_info.rid.to_variant(),
                    monitor_info.instance_id.to_variant(),
                    monitor_info.object_shape_index.to_variant(),
                    monitor_info.area_shape_index.to_variant(),
                ]
            };
            if monitor_info.collision_object_type == CollisionObjectType::Body {
                if let Some(ref monitor_callback) = self.monitor_callback {
                    queries.push((monitor_callback.clone(), arg_array));
                }
            } else if let Some(ref area_monitor_callback) = self.area_monitor_callback {
                queries.push((area_monitor_callback.clone(), arg_array));
            }
        }
        queries
    }

    pub fn clear_monitored_objects(&mut self) {
        self.state.monitored_objects.clear();
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

    pub fn clear_detected_bodies(
        area_rid: &Rid,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_rids: &PhysicsRids,
    ) {
        let mut previous_space_rid = Rid::Invalid;
        let mut detected_bodies = HashMap::default();
        let mut area_handle = RigidBodyHandle::invalid();
        if let Some(area) = physics_collision_objects.get_mut(area_rid)
            && let Some(area) = area.get_mut_area()
        {
            previous_space_rid = area.get_base().get_space(physics_rids);
            detected_bodies = area.state.detected_bodies.clone();
            area.state.detected_bodies.clear();
            area.state.monitored_objects.clear();
            area_handle = area.get_base().get_body_handle();
        }
        if let Some(space) = physics_spaces.get_mut(&previous_space_rid) {
            if !detected_bodies.is_empty() {
                space.area_add_to_monitor_query_list(area_handle);
            }
            space.area_remove_from_area_update_list(area_handle);
            for (detected_body, _) in detected_bodies {
                if let Some(body) =
                    physics_collision_objects.get_mut(&get_body_rid(detected_body, physics_rids))
                {
                    if let Some(body) = body.get_mut_body() {
                        body.remove_area(area_handle, space);
                    }
                }
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
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &mut PhysicsRids,
    ) {
        if p_space == self.base.get_space(physics_rids) {
            return;
        }
        self.base
            .set_space(p_space, physics_engine, physics_spaces, physics_rids);
        self.recreate_shapes(physics_engine, physics_shapes, physics_spaces, physics_rids);
    }

    fn add_shape(
        &mut self,
        p_shape: godot::prelude::Rid,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::add_shape(
            self,
            p_shape,
            p_transform,
            p_disabled,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_rids,
        );
    }

    fn set_shape(
        &mut self,
        p_index: usize,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::set_shape(
            self,
            p_index,
            p_shape,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_rids,
        );
    }

    fn set_shape_transform(
        &mut self,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::set_shape_transform(
            self,
            p_index,
            p_transform,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_rids,
        );
    }

    fn set_shape_disabled(
        &mut self,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::set_shape_disabled(
            self,
            p_index,
            p_disabled,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_rids,
        );
    }

    fn remove_shape_rid(
        &mut self,
        shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_rids: &PhysicsRids,
    ) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.base.state.shapes.len() {
            if self.base.state.shapes[i].shape == shape {
                self.remove_shape_idx(
                    i,
                    physics_engine,
                    physics_spaces,
                    physics_shapes,
                    physics_rids,
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
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::remove_shape_idx(
            self,
            p_index,
            physics_engine,
            physics_spaces,
            physics_shapes,
            physics_rids,
        );
    }

    fn create_shape(
        &mut self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
    ) -> ColliderHandle {
        if !self.base.is_valid() {
            return ColliderHandle::invalid();
        }
        let mat = self.init_material();
        self.base
            .create_shape(shape, p_shape_index, mat, physics_engine, physics_shapes)
    }

    fn init_material(&self) -> Material {
        Material::new(
            self.base.get_collision_layer(),
            self.base.get_collision_mask(),
        )
    }

    fn recreate_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::recreate_shapes(
            self,
            physics_engine,
            physics_shapes,
            physics_spaces,
            physics_rids,
        );
    }

    fn shape_changed(
        &mut self,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
        physics_rids: &PhysicsRids,
    ) {
        RapierCollisionObjectBase::shape_changed(
            self,
            p_shape,
            physics_engine,
            physics_shapes,
            physics_spaces,
            physics_rids,
        );
    }

    fn shapes_changed(
        &mut self,
        _physics_engine: &mut PhysicsEngine,
        _physics_spaces: &mut PhysicsSpaces,
        _physics_rids: &PhysicsRids,
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
