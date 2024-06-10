use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::*;
use crate::spaces::rapier_space::*;
use crate::*;
#[cfg(feature = "dim2")]
use godot::engine::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::engine::physics_server_3d::*;
use godot::log::godot_error;
use godot::meta::ToGodot;
use godot::obj::EngineEnum;
use godot::prelude::*;
use std::collections::HashMap;

use super::rapier_body::RapierBody;

struct MonitorInfo {
    pub rid: Rid,
    pub instance_id: u64,
    pub object_shape_index: u32,
    pub area_shape_index: u32,
    pub collision_object_type: CollisionObjectType,
    pub state: i32,
}

#[derive(Clone, Copy)]
struct BodyRefCount {
    count: u32,
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
    monitor_callback: Callable,
    area_monitor_callback: Callable,
    monitored_objects: HashMap<u128, MonitorInfo>,
    detected_bodies: HashMap<Rid, BodyRefCount>,
    base: RapierCollisionObject,
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
            monitor_callback: Callable::invalid(),
            area_monitor_callback: Callable::invalid(),
            monitored_objects: HashMap::new(),
            detected_bodies: HashMap::new(),
            base: RapierCollisionObject::new(rid, CollisionObjectType::Area),
        }
    }

    pub fn _enable_space_override(&self) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            let rid = self.base.get_rid();
            for (key, _) in self.detected_bodies.iter() {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(key) {
                    if let Some(body) = body.get_mut_body() {
                        body.add_area(rid);
                    }
                }
            }
            // No need to update anymore if it was scheduled before
            space.area_remove_from_area_update_list(rid);
        }
    }
    pub fn _disable_space_override(&self) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            let rid = self.base.get_rid();
            for (key, _) in self.detected_bodies.iter() {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(key) {
                    if let Some(body) = body.get_mut_body() {
                        body.remove_area(rid);
                    }
                }
            }
            // No need to update anymore if it was scheduled before
            space.area_remove_from_area_update_list(rid);
        }
    }
    pub fn _reset_space_override(&self) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            let rid = self.base.get_rid();
            for (key, _) in self.detected_bodies.iter() {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(key) {
                    if let Some(body) = body.get_mut_body() {
                        body.remove_area(rid);
                        body.add_area(rid);
                    }
                }
            }
            space.area_remove_from_area_update_list(rid);
        }
    }

    pub fn on_body_enter(
        &mut self,
        collider_handle: Handle,
        body: &mut Option<&mut Box<dyn IRapierCollisionObject>>,
        body_shape: usize,
        body_rid: Rid,
        body_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
        space: &mut Box<RapierSpace>,
    ) {
        if let Some(body) = body {
            if let Some(body) = body.get_mut_body() {
                // Add to keep track of currently detected bodies
                if let Some(detected_body) = self.detected_bodies.get_mut(&body_rid) {
                    detected_body.count += 1;
                } else {
                    self.detected_bodies
                        .insert(body_rid, BodyRefCount { count: 1 });
                    body.add_area(self.base.get_rid())
                }

                if self.monitor_callback.is_null() {
                    return;
                }

                self.base.area_detection_counter += 1;

                let handle_pair_hash = handle_pair_hash(collider_handle, area_collider_handle);
                if self.monitored_objects.contains_key(&handle_pair_hash) {
                    // it's already monitored
                    return;
                }
                self.monitored_objects.insert(
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
                space.area_add_to_monitor_query_list(self.base.get_rid());
            }
        } else {
            godot_error!("other body is null");
        }
    }

    pub fn on_body_exit(
        &mut self,
        collider_handle: Handle,
        body: &mut Option<&mut Box<dyn IRapierCollisionObject>>,
        body_shape: usize,
        body_rid: Rid,
        body_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
        update_detection: bool,
        space: &mut Box<RapierSpace>,
    ) {
        if update_detection {
            // Remove from currently detected bodies
            if let Some(detected_body) = self.detected_bodies.get_mut(&body_rid) {
                detected_body.count -= 1;
                if detected_body.count == 0 {
                    self.detected_bodies.remove(&body_rid);
                    if let Some(body) = body {
                        if let Some(body) = body.get_mut_body() {
                            body.remove_area(self.base.get_rid());
                        }
                    }
                }
            }
        }

        if self.monitor_callback.is_null() {
            return;
        }

        if body.is_some() {
            self.base.area_detection_counter -= 1;
        }

        let handle_pair_hash = handle_pair_hash(collider_handle, area_collider_handle);

        if let std::collections::hash_map::Entry::Vacant(e) =
            self.monitored_objects.entry(handle_pair_hash)
        {
            e.insert(MonitorInfo {
                rid: body_rid,
                instance_id: body_instance_id,
                object_shape_index: body_shape as u32,
                area_shape_index: area_shape as u32,
                collision_object_type: CollisionObjectType::Body,
                state: -1,
            });

            space.area_add_to_monitor_query_list(self.base.get_rid());
        } else {
            if self.monitored_objects[&handle_pair_hash].state != 1 {
                godot_error!("Body is not being monitored");
                return;
            }
            self.monitored_objects.remove(&handle_pair_hash);
        }
    }

    pub fn on_area_enter(
        &mut self,
        collider_handle: Handle,
        other_area: &mut Option<&mut Box<dyn IRapierCollisionObject>>,
        other_area_shape: usize,
        other_area_rid: Rid,
        other_area_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
        space: &mut Box<RapierSpace>,
    ) {
        if self.area_monitor_callback.is_null() {
            return;
        }
        if let Some(other_area) = other_area {
            if let Some(other_area) = other_area.get_mut_area() {
                if !other_area.is_monitorable() {
                    return;
                }

                other_area.base.area_detection_counter += 1;

                let handle_pair_hash = handle_pair_hash(collider_handle, area_collider_handle);
                if self.monitored_objects.contains_key(&handle_pair_hash) {
                    // it's already monitored
                    return;
                }

                self.monitored_objects.insert(
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

                space.area_add_to_monitor_query_list(self.base.get_rid());
            }
        } else {
            godot_error!("other area is null");
        }
    }

    pub fn on_area_exit(
        &mut self,
        collider_handle: Handle,
        other_area: &mut Option<&mut Box<dyn IRapierCollisionObject>>,
        other_area_shape: usize,
        other_area_rid: Rid,
        other_area_instance_id: u64,
        area_collider_handle: Handle,
        area_shape: usize,
        space: &mut Box<RapierSpace>,
    ) {
        if self.area_monitor_callback.is_null() {
            return;
        }

        if let Some(other_area) = other_area {
            if let Some(other_area) = other_area.get_mut_area() {
                if !other_area.is_monitorable() {
                    return;
                }
                if other_area.base.area_detection_counter == 0 {
                    godot_error!("Area is not being monitored");
                    return;
                }
                other_area.base.area_detection_counter -= 1;
            }
        }

        let handle_pair_hash = handle_pair_hash(collider_handle, area_collider_handle);

        if let std::collections::hash_map::Entry::Occupied(mut e) =
            self.monitored_objects.entry(handle_pair_hash)
        {
            e.insert(MonitorInfo {
                rid: other_area_rid,
                instance_id: other_area_instance_id,
                object_shape_index: other_area_shape as u32,
                area_shape_index: area_shape as u32,
                collision_object_type: CollisionObjectType::Area,
                state: -1,
            });
            space.area_add_to_monitor_query_list(self.base.get_rid());
        } else {
            if self.monitored_objects[&handle_pair_hash].state != 1 {
                godot_error!("Area is not being monitored");
                return;
            }
            self.monitored_objects.remove(&handle_pair_hash);
        }
    }

    pub fn update_area_override(&mut self) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            space.area_remove_from_area_update_list(self.base.get_rid());
            for (detected_body, _) in self.detected_bodies.clone() {
                if let Some(body) = bodies_singleton().collision_objects.get_mut(&detected_body) {
                    if let Some(body) = body.get_mut_body() {
                        body.update_area_override();
                    }
                }
            }
        }
    }

    pub fn has_any_space_override(&self) -> bool {
        self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED
            || self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED
            || self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED
    }

    pub fn set_monitor_callback(&mut self, callback: Callable) {
        self.monitor_callback = callback;
    }

    pub fn set_area_monitor_callback(&mut self, callback: Callable) {
        self.area_monitor_callback = callback;
    }

    pub fn set_param(&mut self, p_param: AreaParameter, p_value: Variant) {
        match p_param {
            AreaParameter::GRAVITY_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                self.gravity_override_mode = AreaSpaceOverrideMode::from_ord(p_value.to());
                let has_override = self.has_any_space_override();

                // Update currently detected bodies if needed
                if has_override != had_override {
                    if has_override {
                        self._enable_space_override();
                    } else {
                        self._disable_space_override();
                    }
                }
            }

            AreaParameter::GRAVITY => {
                let new_gravity = p_value.to();
                if new_gravity != self.gravity {
                    self.gravity = new_gravity;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_VECTOR => {
                let new_gravity_vector = p_value.to();
                if self.gravity_vector != new_gravity_vector {
                    self.gravity_vector = new_gravity_vector;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_IS_POINT => {
                let new_gravity_is_point = p_value.to();
                if self.gravity_is_point != new_gravity_is_point {
                    self.gravity_is_point = new_gravity_is_point;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::GRAVITY_POINT_UNIT_DISTANCE => {
                let new_gravity_point_unit_distance = p_value.to();
                if self.gravity_point_unit_distance != new_gravity_point_unit_distance {
                    self.gravity_point_unit_distance = new_gravity_point_unit_distance;
                    if self.gravity_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::LINEAR_DAMP_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                self.linear_damping_override_mode = AreaSpaceOverrideMode::from_ord(p_value.to());
                let has_override = self.has_any_space_override();

                // Update currently detected bodies if needed
                if has_override != had_override {
                    if has_override {
                        self._enable_space_override();
                    } else {
                        self._disable_space_override();
                    }
                }
            }
            AreaParameter::LINEAR_DAMP => {
                let new_linear_damp = p_value.to();
                if self.linear_damp != new_linear_damp {
                    self.linear_damp = new_linear_damp;
                    if self.linear_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE => {
                let had_override = self.has_any_space_override();
                self.angular_damping_override_mode = AreaSpaceOverrideMode::from_ord(p_value.to());
                let has_override = self.has_any_space_override();

                // Update currently detected bodies if needed
                if has_override != had_override {
                    if has_override {
                        self._enable_space_override();
                    } else {
                        self._disable_space_override();
                    }
                }
            }
            AreaParameter::ANGULAR_DAMP => {
                let new_angular_damp = p_value.to();
                if self.angular_damp != new_angular_damp {
                    self.angular_damp = new_angular_damp;
                    if self.angular_damping_override_mode != AreaSpaceOverrideMode::DISABLED {
                        // Update currently detected bodies
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            space.area_add_to_area_update_list(self.base.get_rid());
                        }
                    }
                }
            }
            AreaParameter::PRIORITY => {
                let new_priority = p_value.to();
                if self.priority != new_priority {
                    self.priority = new_priority;
                    if self.has_any_space_override() {
                        // Need to re-process priority list for each body
                        self._reset_space_override();
                    }
                }
            }
            _ => {}
        }
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

    pub fn set_monitorable(&mut self, monitorable: bool) {
        self.monitorable = monitorable;
    }

    pub fn is_monitorable(&self) -> bool {
        self.monitorable
    }

    pub fn call_queries(&self) {
        if self.monitored_objects.is_empty() {
            return;
        }

        for (_, monitor_info) in &self.monitored_objects {
            let mut arg_array = VariantArray::new();
            arg_array.resize(5, &Variant::nil());
            if monitor_info.state == 0 {
                continue;
            }

            if monitor_info.state > 0 {
                arg_array.set(0, AreaBodyStatus::ADDED.to_variant());
            } else {
                arg_array.set(0, AreaBodyStatus::REMOVED.to_variant());
            }
            arg_array.set(1, monitor_info.rid.to_variant());
            arg_array.set(2, monitor_info.instance_id.to_variant());
            arg_array.set(3, monitor_info.object_shape_index.to_variant());
            arg_array.set(4, monitor_info.area_shape_index.to_variant());

            if monitor_info.collision_object_type == CollisionObjectType::Body {
                if self.monitor_callback.is_valid() {
                    self.monitor_callback.callv(arg_array);
                }
            } else if self.area_monitor_callback.is_valid() {
                self.area_monitor_callback.callv(arg_array);
            }
        }
    }

    pub fn compute_gravity(&self, position: Vector) -> Vector {
        if self.gravity_is_point {
            let gr_unit_dist = self.get_gravity_point_unit_distance();
            let v = self.get_base().get_transform() * self.gravity_vector - position;
            if gr_unit_dist > 0.0 {
                let v_length_sq = v.length_squared();
                if v_length_sq > 0.0 {
                    let gravity_strength = self.gravity * gr_unit_dist * gr_unit_dist / v_length_sq;
                    v.normalized() * gravity_strength
                } else {
                    Vector::default()
                }
            } else {
                v.normalized() * self.gravity
            }
        } else {
            self.gravity_vector * self.gravity
        }
    }
}

impl IRapierCollisionObject for RapierArea {
    fn get_base(&self) -> &RapierCollisionObject {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierCollisionObject {
        &mut self.base
    }
    fn set_space(&mut self, p_space: Rid) {
        if p_space == self.base.get_space() {
            return;
        }

        if self.base.get_space().is_invalid() {
            // Need to keep in list to handle remove events for bodies
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                if !self.detected_bodies.is_empty() {
                    space.area_add_to_monitor_query_list(self.base.get_rid());
                }
                for (detected_body, _) in self.detected_bodies.clone() {
                    if let Some(body) = bodies_singleton().collision_objects.get_mut(&detected_body)
                    {
                        if let Some(body) = body.get_mut_body() {
                            body.remove_area(self.base.get_rid());
                        }
                    }
                }
                self.detected_bodies.clear();

                space.area_remove_from_area_update_list(self.base.get_rid());
            }
        }

        self.monitored_objects.clear();

        self.base._set_space(p_space);
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

    fn add_shape(
        &mut self,
        p_shape: godot::prelude::Rid,
        p_transform: Transform,
        p_disabled: bool,
    ) {
        let mut shape = CollisionObjectShape {
            xform: p_transform,
            shape: p_shape,
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: invalid_handle(),
        };

        if !shape.disabled {
            shape.collider_handle = self.create_shape(shape, self.base.shapes.len());
            self.base.update_shape_transform(&shape);
        }

        self.base.shapes.push(shape);
        if let Some(shape) = shapes_singleton().shapes.get_mut(&p_shape) {
            shape.get_mut_base().add_owner(self.base.get_rid());
        }

        if self.base.get_space_handle().is_valid() {
            self._shapes_changed();
        }
    }

    fn set_shape(&mut self, p_index: usize, p_shape: Rid) {
        if p_index >= self.base.shapes.len() {
            return;
        }

        self.base.shapes[p_index].collider_handle =
            self.base._destroy_shape(self.base.shapes[p_index], p_index);
        let shape = self.base.shapes[p_index];
        if let Some(shape) = shapes_singleton().shapes.get_mut(&shape.shape) {
            shape.get_mut_base().remove_owner(self.base.get_rid());
        }
        self.base.shapes[p_index].shape = p_shape;
        if let Some(shape) = shapes_singleton().shapes.get_mut(&p_shape) {
            shape.get_mut_base().add_owner(self.base.get_rid());
        }

        if !shape.disabled {
            self.base
                ._create_shape(shape, p_index, self._init_material());
            self.base.update_shape_transform(&shape);
        }

        if self.base.get_space_handle().is_valid() {
            self._shapes_changed();
        }
    }

    fn set_shape_transform(&mut self, p_index: usize, p_transform: Transform) {
        if p_index >= self.base.shapes.len() {
            return;
        }

        self.base.shapes[p_index].xform = p_transform;
        let shape = &self.base.shapes[p_index];

        self.base.update_shape_transform(shape);

        if self.base.get_space_handle().is_valid() {
            self._shapes_changed();
        }
    }

    fn set_shape_disabled(&mut self, p_index: usize, p_disabled: bool) {
        if p_index >= self.base.shapes.len() {
            return;
        }
        self.base.shapes[p_index].disabled = p_disabled;
        let shape = self.base.shapes[p_index];
        if shape.disabled == p_disabled {
            return;
        }
        if shape.disabled {
            self.base._destroy_shape(shape, p_index);
        }

        if !shape.disabled {
            self.base
                ._create_shape(shape, p_index, self._init_material());
            self.base.update_shape_transform(&shape);
        }

        if self.base.get_space_handle().is_valid() {
            self._shapes_changed();
        }
    }

    fn remove_shape_rid(&mut self, shape: Rid) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.base.shapes.len() {
            if self.base.shapes[i].shape == shape {
                self.remove_shape_idx(i);
            } else {
                i += 1;
            }
        }
    }

    fn remove_shape_idx(&mut self, p_index: usize) {
        // remove anything from shape to be erased to end, so subindices don't change
        if p_index >= self.base.shapes.len() {
            return;
        }

        let shape = &self.base.shapes[p_index];

        if !shape.disabled {
            self.base._destroy_shape(*shape, p_index);
        }
        let shape = &mut self.base.shapes[p_index];
        shape.collider_handle = invalid_handle();
        if let Some(shape) = shapes_singleton().shapes.get_mut(&shape.shape) {
            shape.get_mut_base().remove_owner(self.base.get_rid());
        }

        self.base.shapes.remove(p_index);

        if self.base.get_space_handle().is_valid() {
            self._shapes_changed();
        }
    }

    fn create_shape(&mut self, shape: CollisionObjectShape, p_shape_index: usize) -> Handle {
        if !self.base.get_space_handle().is_valid() {
            return invalid_handle();
        }
        let mat = self._init_material();

        self.base._create_shape(shape, p_shape_index, mat)
    }

    fn _init_material(&self) -> Material {
        default_material()
    }

    fn _shape_changed(&mut self, p_shape: Rid) {
        if !self.base.get_space_handle().is_valid() {
            return;
        }
        for i in 0..self.base.shapes.len() {
            let shape = self.base.shapes[i];
            if shape.shape != p_shape || shape.disabled {
                continue;
            }

            self.base.shapes[i].collider_handle = self.base._destroy_shape(shape, i);

            self.base
                ._create_shape(self.base.shapes[i], i, self._init_material());
            self.base.update_shape_transform(&self.base.shapes[i]);
        }

        self._shapes_changed();
    }

    fn recreate_shapes(&mut self) {
        for i in 0..self.base.get_shape_count() as usize {
            if self.base.shapes[i].disabled {
                continue;
            }
            self.base.shapes[i].collider_handle = self.create_shape(self.base.shapes[i], i);
            self.base.update_shape_transform(&self.base.shapes[i]);
        }
    }

    fn _shapes_changed(&mut self) {}
}
