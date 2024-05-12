use crate::bodies::rapier_collision_object_2d::*;
use crate::rapier2d::body::*;
use crate::rapier2d::collider::collider_set_contact_force_events_enabled;
use crate::rapier2d::collider::Material;
use crate::rapier2d::handle::is_handle_valid;
use crate::rapier2d::handle::Handle;
use crate::rapier2d::vector::Vector;
use crate::servers::rapier_physics_singleton_2d::shapes_singleton;
use crate::servers::rapier_physics_singleton_2d::spaces_singleton;
use godot::engine::physics_server_2d::AreaParameter;
use godot::engine::physics_server_2d::{BodyDampMode, BodyMode, BodyParameter, BodyState, CcdMode};
use godot::engine::PhysicsDirectBodyState2D;
use godot::prelude::*;
use std::collections::HashSet;

use super::rapier_area_2d::RapierArea2D;
use super::rapier_direct_body_state_2d::RapierDirectBodyState2D;

#[derive(Clone)]
struct Contact {
    pub local_pos: Vector2,
    pub local_normal: Vector2,
    pub depth: real,
    pub local_shape: i32,
    pub collider_pos: Vector2,
    pub collider_shape: i32,
    pub collider_instance_id: u64,
    //pub collider_object: Option<Object>,
    pub collider: Rid,
    pub local_velocity_at_pos: Vector2,
    pub collider_velocity_at_pos: Vector2,
    pub impulse: Vector2,
}

impl Default for Contact {
    fn default() -> Self {
        Self {
            local_pos: Vector2::ZERO,
            local_normal: Vector2::ZERO,
            depth: 0.0,
            local_shape: 0,
            collider_pos: Vector2::ZERO,
            collider_shape: 0,
            collider_instance_id: 0,
            //collider_object: None,
            collider: Rid::Invalid,
            local_velocity_at_pos: Vector2::ZERO,
            collider_velocity_at_pos: Vector2::ZERO,
            impulse: Vector2::ZERO,
        }
    }
}

struct ForceIntegrationCallbackData {
    callable: Callable,
    udata: Variant,
}

impl ForceIntegrationCallbackData {
    fn new(callable: Callable, udata: Variant) -> Self {
        Self { callable, udata }
    }
}

// Define the RapierBody2D struct
pub struct RapierBody2D {
    linear_damping_mode: BodyDampMode,
    angular_damping_mode: BodyDampMode,
    linear_damping: real,
    angular_damping: real,
    total_linear_damping: real,
    total_angular_damping: real,
    total_gravity: Vector2,
    gravity_scale: real,
    bounce: real,
    friction: real,
    mass: real,
    inertia: real,
    center_of_mass: Vector2,
    calculate_inertia: bool,
    calculate_center_of_mass: bool,
    using_area_gravity: bool,
    using_area_linear_damping: bool,
    using_area_angular_damping: bool,
    exceptions: HashSet<Rid>,
    ccd_mode: CcdMode,
    omit_force_integration: bool,
    active: bool,
    marked_active: bool,
    can_sleep: bool,
    constant_force: Vector2,
    linear_velocity: Vector2,
    impulse: Vector2,
    torque: real,
    angular_velocity: real,
    constant_torque: real,
    to_add_angular_velocity: real,
    to_add_linear_velocity: Vector2,
    sleep: bool,
    areas: Vec<Rid>,
    contacts: Vec<Contact>,
    contact_count: i32,
    body_state_callback: Callable,
    fi_callback_data: Option<ForceIntegrationCallbackData>,
    direct_state: Option<Gd<PhysicsDirectBodyState2D>>,
    direct_state_query_list: Vec<Rid>,
    base: RapierCollisionObject2D,
}

impl RapierBody2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            linear_damping_mode: BodyDampMode::COMBINE,
            angular_damping_mode: BodyDampMode::COMBINE,
            linear_damping: 0.0,
            angular_damping: 0.0,
            total_linear_damping: 0.0,
            total_angular_damping: 0.0,
            total_gravity: Vector2::ZERO,
            gravity_scale: 1.0,
            bounce: 0.0,
            friction: 1.0,
            mass: 0.0,
            inertia: 0.0,
            center_of_mass: Vector2::ZERO,
            calculate_inertia: false,
            calculate_center_of_mass: false,
            using_area_gravity: false,
            using_area_linear_damping: false,
            using_area_angular_damping: false,
            exceptions: HashSet::new(),
            ccd_mode: CcdMode::DISABLED,
            omit_force_integration: false,
            active: true,
            marked_active: true,
            can_sleep: true,
            constant_force: Vector2::ZERO,
            linear_velocity: Vector2::ZERO,
            impulse: Vector2::ZERO,
            torque: 0.0,
            angular_velocity: 0.0,
            constant_torque: 0.0,
            to_add_angular_velocity: 0.0,
            to_add_linear_velocity: Vector2::ZERO,
            sleep: false,
            areas: Vec::new(),
            contacts: Vec::new(),
            contact_count: 0,
            body_state_callback: Callable::invalid(),
            fi_callback_data: None,
            direct_state: None,
            direct_state_query_list: Vec::new(),
            base: RapierCollisionObject2D::new(rid, CollisionObjectType::Body),
        }
    }

    fn _apply_linear_damping(&mut self, new_value: real, apply_default: bool) {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            self.total_linear_damping = new_value;
            if apply_default {
                let linear_damp: real = space.get_default_area_param(AreaParameter::LINEAR_DAMP).to();
                self.total_linear_damping += linear_damp;
            }
            let space_handle = space.get_handle();
            body_set_linear_damping(space_handle, self.get_base().get_body_handle(), self.total_linear_damping);
        }
    }

    fn _apply_angular_damping(&mut self, new_value: real, apply_default: bool) {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            self.total_angular_damping = new_value;
            if apply_default {
                let angular_damp: real = space.get_default_area_param(AreaParameter::ANGULAR_DAMP).to();
                self.total_angular_damping += angular_damp;
            }
            let space_handle = space.get_handle();
            body_set_angular_damping(space_handle, self.get_base().get_body_handle(), self.total_angular_damping);
        }
    }

    fn _apply_gravity_scale(&self, new_value: real) {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            body_set_gravity_scale(space_handle, self.get_base().get_body_handle(), new_value, true);
        }
    }

    fn _init_material(&self, mat: &mut Material) {
        mat.friction = self.friction;
        mat.restitution = self.bounce;
    }

    fn _init_collider(&self, collider_handle: Handle, space_handle: Handle) {
        // Send contact infos for dynamic bodies
        if self.get_base().mode.ord() >= BodyMode::KINEMATIC.ord() {
            let send_contacts = self.can_report_contacts();
            collider_set_contact_force_events_enabled(space_handle, collider_handle, send_contacts);
        }
    }

    pub fn to_add_static_constant_linear_velocity(&mut self, linear_velocity: Vector2) {
        self.to_add_linear_velocity = linear_velocity;
    }
    pub fn to_add_static_constant_angular_velocity(&mut self, angular_velocity: real) {
        self.to_add_angular_velocity = angular_velocity;
    }

    pub fn set_linear_velocity(&mut self, p_linear_velocity: Vector2) {
        self.linear_velocity = p_linear_velocity;
        if self.get_base().mode == BodyMode::STATIC {
            return;
        }
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            let body_handle = self.get_base().get_body_handle();
        
            if !is_handle_valid(space_handle) || !is_handle_valid(body_handle) {
                return;
            }
            let velocity = Vector::new(self.linear_velocity.x, self.linear_velocity.y);
            self.linear_velocity = Vector2::default();
            body_set_linear_velocity(space_handle, body_handle, &velocity);
        }
    }
    pub fn get_linear_velocity(&self) -> Vector2 {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            let body_handle = self.get_base().get_body_handle();
            if !is_handle_valid(space_handle) || !is_handle_valid(body_handle) {
                return self.linear_velocity;
            }
            let vel = body_get_linear_velocity(space_handle, body_handle);
            return Vector2::new(vel.x, vel.y);
        }
        return self.linear_velocity;
    }
    pub fn get_static_linear_velocity(&self) -> Vector2 {
	    return self.linear_velocity;
    }

    pub fn set_angular_velocity(&mut self, p_angular_velocity: real) {
        self.angular_velocity = p_angular_velocity;
        if self.get_base().mode == BodyMode::STATIC {
            return;
        }
        
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            let body_handle = self.get_base().get_body_handle();
            if !is_handle_valid(space_handle) || !is_handle_valid(body_handle) {
                return;
            }
            self.angular_velocity = 0.0;
            body_set_angular_velocity(space_handle, body_handle, self.angular_velocity);
        }
    }
    pub fn get_angular_velocity(&self) -> real {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            let body_handle = self.get_base().get_body_handle();
            if !is_handle_valid(space_handle) || !is_handle_valid(body_handle) {
                return self.angular_velocity;
            }
            return body_get_angular_velocity(space_handle, body_handle);
        }
        return self.angular_velocity;
    }
    pub fn get_static_angular_velocity(&self) -> real {
        self.angular_velocity
    }

    pub fn set_state_sync_callback(&mut self, p_callable: Callable) {
        self.body_state_callback = p_callable;
    }
    pub fn set_force_integration_callback(&mut self, p_callable: Callable, p_udata: Variant) {
        if p_callable.is_valid() {
            self.fi_callback_data = Some(ForceIntegrationCallbackData::new(p_callable, p_udata));
        } else {
            self.fi_callback_data = None;
        }
    }

    pub fn get_direct_state(&mut self) -> Option<Gd<PhysicsDirectBodyState2D>> {
        if self.direct_state.is_none() {
            let direct_space_state = RapierDirectBodyState2D::new_alloc();
            //direct_space_state.set_body(self.rid);
            self.direct_state = Some(direct_space_state.upcast());
        }
        self.direct_state
    }

    pub fn add_area(&mut self, p_area: Rid) {
        self.base.area_detection_counter += 1;
        if (self.p_area.has_any_space_override()) {
            self.areas.ordered_insert(AreaCMP(p_area));
            on_area_updated(p_area);
        }
    }
    pub fn remove_area(&mut self, area: Rid) {}
    pub fn on_area_updated(&mut self, area: Rid) {
        if (!area_override_update_list.in_list()) {
            get_space()->body_add_to_area_update_list(&area_override_update_list);
        }
    }

    pub fn update_area_override(&self) {}
    pub fn update_gravity(&mut self, p_step: real) {
        if !self.using_area_gravity {
            return;
        }
        if !self.areas.is_empty() {
            self.update_area_override();
        }
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            let body_handle = self.get_base().get_body_handle();

            if !is_handle_valid(space_handle) || !is_handle_valid(body_handle) {
                return;
            }
            let gravity_impulse = self.total_gravity * self.mass * p_step;
            let impulse = Vector::new(gravity_impulse.x, gravity_impulse.y);
            body_apply_impulse(space_handle, body_handle, &impulse);
        }
    }

    pub fn set_max_contacts_reported(&mut self, size: i32) {
        self.contacts.resize(size as usize, Contact::default());
        self.contact_count = 0;
    }
    pub fn reset_contact_count(&mut self) {
        self.contact_count = 0;
    }
    pub fn get_max_contacts_reported(&self) -> i32 {
        return self.contacts.len() as i32;
    }
    pub fn can_report_contacts(&self) -> bool {
        return !self.contacts.is_empty();
    }
    fn add_contact(
        &mut self,
        local_pos: Vector2,
        local_normal: Vector2,
        depth: real,
        local_shape: i32,
        local_velocity_at_pos: Vector2,
        collider_pos: Vector2,
        collider_shape: i32,
        collider_instance_id: u64,
        //collider_object: Option<Object>,
        collider: Rid,
        collider_velocity_at_pos: Vector2,
        impulse: Vector2,
    ) {
        let c_max = self.contacts.len();
        
        if c_max == 0 {
            return;
        }
        
        let mut idx = -1;
        
        if self.contact_count < c_max as i32 {
            idx = self.contact_count;
            self.contact_count += 1;
        } else {
            let mut least_depth = f32::INFINITY;
            let mut least_deep: i32 = -1;
            for (i, contact) in self.contacts.iter().enumerate() {
                if i == 0 || contact.depth < least_depth {
                    least_deep = i as i32;
                    least_depth = contact.depth;
                }
            }
        
            if least_deep >= 0 && least_depth < depth {
                idx = least_deep;
            }
            if idx == -1 {
                return; // none less deep than this
            }
        }
        
        let c = &mut self.contacts[idx as usize];
        c.local_pos = local_pos;
        c.local_normal = local_normal;
        c.depth = depth;
        c.local_shape = local_shape;
        c.collider_pos = collider_pos;
        c.collider_shape = collider_shape;
        c.collider_instance_id = collider_instance_id;
        //c.collider_object = p_collider_object;
        c.collider = collider;
        c.collider_velocity_at_pos = collider_velocity_at_pos;
        c.local_velocity_at_pos = local_velocity_at_pos;
        c.impulse = impulse;
        
    }

    pub fn add_exception(&mut self, exception: Rid) {
        self.exceptions.insert(exception);
    }
    pub fn remove_exception(&mut self, exception: Rid) {
        self.exceptions.remove(&exception);
    }
    pub fn has_exception(&self, exception: Rid) -> bool {
        return self.exceptions.contains(&exception);
    }
    pub fn get_exceptions(&self) -> HashSet<Rid> {
        return self.exceptions.clone();
    }

    pub fn set_omit_force_integration(&mut self, omit_force_integration: bool) {
        self.omit_force_integration = omit_force_integration;
    }
    pub fn get_omit_force_integration(&self) -> bool {
        return self.omit_force_integration;
    }

    pub fn apply_central_impulse(&self, p_impulse: Vector2) {
        let lock = spaces_singleton().lock().unwrap();
        if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
            let space_handle = space.get_handle();
            let body_handle = self.get_base().get_body_handle();
            if !is_handle_valid(space_handle) || !is_handle_valid(body_handle) {
                return;
            }
            self.angular_velocity = 0.0;
            body_set_angular_velocity(space_handle, body_handle, self.angular_velocity);
        } else {
            self.impulse += p_impulse
        }
    }

    pub fn apply_impulse(&self, impulse: Vector2, position: Vector2) {}

    pub fn apply_torque_impulse(&self, torque: real) {}

    pub fn apply_central_force(&self, force: Vector2) {}

    pub fn apply_force(&self, force: Vector2, position: Vector2) {}

    pub fn apply_torque(&self, torque: real) {}

    pub fn add_constant_central_force(&self, force: Vector2) {}

    pub fn add_constant_force(&self, force: Vector2, position: Vector2) {}

    pub fn add_constant_torque(&self, torque: real) {}

    pub fn set_constant_force(&self, force: Vector2) {}
    pub fn get_constant_force(&self) -> Vector2 {}

    pub fn set_constant_torque(&self, torque: real) {}
    pub fn get_constant_torque(&self) -> real {}

    pub fn set_active(&self, active: bool) {}
    pub fn is_active(&self) -> bool {
        return self.active;
    }

    pub fn set_can_sleep(&self, can_sleep: bool) {}

    pub fn on_marked_active(&self) {
        if (self.base.mode == BodyMode::STATIC) {
            return;
        }
        self.marked_active = true;
        if (!self.active) {
            self.active = true;
            get_space().body_add_to_active_list(self.base.get_rid());
        }
    }
    pub fn on_update_active(&self) {}

    pub fn wakeup(&self) {}
    pub fn force_sleep(&self) {}

    pub fn set_param(&mut self, param: BodyParameter, value: Variant) {}
    pub fn get_param(&self, param: BodyParameter) -> Variant {}

    pub fn set_mode(&mut self, p_mode: BodyMode) {
        let mode = self.base.mode;
        if mode == p_mode {
            return;
        }
    
        let prev_mode = mode;
        mode = p_mode;
    let spaces_lock = spaces_singleton().lock().unwrap();
    if let Some(space) = spaces_lock.spaces.get(&self.get_base().get_space()) {
        match p_mode {
            BodyMode::KINEMATIC => {
                body_change_mode(space.get_handle(), get_body_handle(), rapier2d::BodyType::Kinematic, true);
            }
            BodyMode::STATIC => {
                body_change_mode(space.get_handle(), get_body_handle(), rapier2d::BodyType::Static, true);
            }
            BodyMode::RIGID | BodyMode::RIGID_LINEAR => {
                body_change_mode(space.get_handle(), get_body_handle(), rapier2d::BodyType::Dynamic, true);
            }
        }
    }
    
    if p_mode == BodyMode::STATIC {
        self.force_sleep();
    
        if self.marked_active {
            return;
        }
        self.active_list.remove_from_list();
        self.mass_properties_update_list.remove_from_list();
        self.gravity_update_list.remove_from_list();
        self.area_override_update_list.remove_from_list();
    
        return;
    }
    
    if active && prev_mode == PhysicsServer2D::BODY_MODE_STATIC {
        if let Some(space) = get_space() {
            space.body_add_to_active_list(&active_list);
        }
    }
    
    if p_mode >= PhysicsServer2D::BODY_MODE_RIGID {
        _mass_properties_changed();
    
        if let Some(space) = get_space() {
            update_area_override();
            _apply_gravity_scale(gravity_scale);
        }
    }
    
    }

    pub fn set_state(&mut self, state: BodyState, variant: Variant) {}

    pub fn get_state(&self, state: BodyState) -> Variant {}

    pub fn set_continuous_collision_detection_mode(&mut self, mode: CcdMode) {}
    pub fn get_continuous_collision_detection_mode(&self) -> CcdMode {
        return self.ccd_mode;
    }

    pub fn update_mass_properties(&mut self, force_update: bool) {}
    pub fn reset_mass_properties(&mut self) {}

    pub fn get_center_of_mass(&self) -> Vector2 {
        return self.center_of_mass;
    }
    pub fn get_mass(&self) -> real {
        return self.mass;
    }
    pub fn get_inv_mass(&self) -> real {
        if self.mass != 0.0 {
            return 1.0 / self.mass;
        }
        return 0.0;
    }
    pub fn get_inertia(&self) -> real {
        return self.inertia;
    }
    pub fn get_inv_inertia(&self) -> real {
        if self.inertia != 0.0 {
            return 1.0 / self.inertia;
        }
        return 0.0;
    }
    pub fn get_friction(&self) -> real {
        return self.friction;
    }
    pub fn get_bounce(&self) -> real {
        return self.bounce;
    }

    pub fn get_velocity_at_local_point(&self, rel_pos: Vector2) -> Vector2 {
        let linear_velocity = self.get_linear_velocity();
        let angular_velocity = self.get_angular_velocity();
        return linear_velocity
            + Vector2::new(
                -angular_velocity * (rel_pos.y - self.center_of_mass.y),
                angular_velocity * (rel_pos.x - self.center_of_mass.x),
            );
    }

    pub fn call_queries(&self) {
        if let Some(direct_state) = &self.direct_state {
            if let Some (fi_callback_data) = &self.fi_callback_data {
                if fi_callback_data.callable.is_valid() {
                    let mut arg_array = Array::new();
        
                    arg_array.push(direct_state.to_variant());
                    arg_array.push(fi_callback_data.udata);
        
                    fi_callback_data.callable.callv(arg_array);
                }
            }
        
            //  Sync body server with Godot by sending body direct state
            if self.body_state_callback.is_valid() {
                let mut arg_array = Array::new();
    
                arg_array.push(direct_state.to_variant());
    
                self.body_state_callback.callv(arg_array);
            }
        }
        
        if !self.active {
            let lock = spaces_singleton().lock().unwrap();
            if let Some(space) = lock.spaces.get(&self.get_base().get_space()) {
                space.body_remove_from_state_query_list(self.base.get_rid());
            }
        }
    }

    pub fn get_aabb(&self) -> Rect2 {
        let mut shapes_found = false;
        let mut body_aabb = Rect2::default();
        let shape_count = self.base.get_shape_count() as usize;
        for i in 0..shape_count {
            if (self.base.is_shape_disabled(i)) {
                continue;
            }
            let shape_lock = shapes_singleton().lock().unwrap();
            if let Some(shape) = shape_lock.shapes.get(&self.base.get_shape(i)) {
                if (!shapes_found) {
                    // TODO not 100% correct, we don't take into consideration rotation here.
                    body_aabb = shape.get_base().get_aabb(self.get_base().get_shape_transform(i).origin);
                    shapes_found = true;
                } else {
                    // TODO not 100% correct, we don't take into consideration rotation here.
                    body_aabb = body_aabb.merge(shape.get_base().get_aabb(self.get_base().get_shape_transform(i).origin));
                }
            }
        }
        return body_aabb;
    }


    pub fn total_linear_damping(&self)-> real {
        self.total_linear_damping
    }
    pub fn total_angular_damping(&self)-> real {
        self.total_angular_damping
    }
    pub fn total_gravity(&self)-> Vector2 {
        self.total_gravity
    }
    pub fn gravity_scale(&self)-> real {
        self.gravity_scale
    }
    pub fn using_area_gravity(&self)-> bool {
        self.using_area_gravity
    }
    pub fn contact_count(&self)-> i32 {
        self.contact_count
    }
    pub fn contacts(&self)-> &Vec<Contact> {
        &self.contacts
    }
}

impl IRapierCollisionObject2D for RapierBody2D {
    fn get_base(&self) -> &RapierCollisionObject2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierCollisionObject2D {
        &mut self.base
    }

    fn set_space(&mut self, space: Rid) {}

    fn get_body(&self) -> Option<&RapierBody2D> {
        Some(self)
    }

    fn get_area(&self) -> Option<&RapierArea2D> {
        None
    }

    fn get_mut_body(&mut self) -> Option<&mut RapierBody2D> {
        Some(self)
    }

    fn get_mut_area(&mut self) -> Option<&mut RapierArea2D> {
        None
    }
    
    fn add_shape(
        &mut self,
        p_shape: &Box<dyn crate::shapes::rapier_shape_2d::IRapierShape2D>,
        p_transform: Transform2D,
        p_disabled: bool,
    ) {
        todo!()
    }
}
