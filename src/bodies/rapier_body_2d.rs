use crate::bodies::rapier_collision_object_2d::CollisionObjectType;
use crate::bodies::rapier_collision_object_2d::IRapierCollisionObject2D;
use crate::bodies::rapier_collision_object_2d::RapierCollisionObject2D;
use crate::rapier2d::collider::Material;
use crate::rapier2d::handle::Handle;
use godot::engine::native::ObjectId;
use godot::engine::physics_server_2d::{BodyDampMode, BodyMode, BodyParameter, BodyState, CcdMode};
use godot::engine::PhysicsDirectBodyState2D;
use godot::prelude::*;
use std::collections::HashSet;

use super::rapier_area_2d::RapierArea2D;
use super::rapier_direct_body_state_2d::RapierDirectBodyState2D;

struct Contact {
    pub local_pos: Vector2,
    pub local_normal: Vector2,
    pub depth: real,
    pub local_shape: i32,
    pub collider_pos: Vector2,
    pub collider_shape: i32,
    pub collider_instance_id: InstanceId,
    //pub collider_object: Option<Object>,
    pub collider: Rid,
    pub local_velocity_at_pos: Vector2,
    pub collider_velocity_at_pos: Vector2,
    pub impulse: Vector2,
}

struct ForceIntegrationCallbackData {
    callable: Callable,
    udata: Variant,
}

// Define the RapierBody2D struct
pub struct RapierBody2D {
    linear_damping_mode: BodyDampMode,
    angular_damping_mode: BodyDampMode,
    linear_damping: real,
    angular_damping: real,
    pub total_linear_damping: real,
    pub total_angular_damping: real,
    pub total_gravity: Vector2,
    pub gravity_scale: real,
    bounce: real,
    friction: real,
    mass: real,
    inertia: real,
    center_of_mass: Vector2,
    calculate_inertia: bool,
    calculate_center_of_mass: bool,
    pub using_area_gravity: bool,
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
    active_list: Vec<Rid>,
    mass_properties_update_list: Vec<Rid>,
    gravity_update_list: Vec<Rid>,
    areas: Vec<Rid>,
    area_override_update_list: Vec<Rid>,
    pub contacts: Vec<Contact>,
    pub contact_count: i32,
    body_state_callback: Callable,
    fi_callback_data: ForceIntegrationCallbackData,
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
            active_list: Vec::new(),
            mass_properties_update_list: Vec::new(),
            gravity_update_list: Vec::new(),
            areas: Vec::new(),
            area_override_update_list: Vec::new(),
            contacts: Vec::new(),
            contact_count: 0,
            body_state_callback: Callable::invalid(),
            fi_callback_data: ForceIntegrationCallbackData {
                callable: Callable::invalid(),
                udata: Variant::nil(),
            },
            direct_state: None,
            direct_state_query_list: Vec::new(),
            base: RapierCollisionObject2D::new(rid, CollisionObjectType::Body),
        }
    }

    fn _apply_linear_damping(&self, new_value: real, apply_default: bool) {}

    fn _apply_angular_damping(&self, new_value: real, apply_default: bool) {}

    fn _apply_gravity_scale(&self, new_value: real) {}

    fn _init_material(&self, mat: &Material) {}
    fn _init_collider(&self, collider_handle: Handle) {}

    pub fn to_add_static_constant_linear_velocity(&mut self, linear_velocity: Vector2) {
        self.to_add_linear_velocity = linear_velocity;
    }
    pub fn to_add_static_constant_angular_velocity(&mut self, angular_velocity: real) {
        self.to_add_angular_velocity = angular_velocity;
    }

    pub fn set_linear_velocity(&mut self, linear_velocity: &Vector2) {}
    pub fn get_linear_velocity(&self) -> Vector2 {}
    pub fn get_static_linear_velocity(&self) -> Vector2 {}

    pub fn set_angular_velocity(&self, angular_velocity: real) {}
    pub fn get_angular_velocity(&self) -> real {}
    pub fn get_static_angular_velocity(&self) -> real {}

    pub fn set_state_sync_callback(&mut self, callable: Callable) {}
    pub fn set_force_integration_callback(&mut self, callable: Callable, udata: Variant) {}

    pub fn get_direct_state(&mut self) -> Option<Gd<PhysicsDirectBodyState2D>> {
        if self.direct_state.is_none() {
            let direct_space_state = RapierDirectBodyState2D::new_alloc();
            //direct_space_state.set_body(self.rid);
            self.direct_state = Some(direct_space_state.upcast());
        }
        self.direct_state
    }

    pub fn add_area(&mut self, area: Rid) {}
    pub fn remove_area(&mut self, area: Rid) {}
    pub fn on_area_updated(&mut self, area: Rid) {}

    pub fn update_area_override(&self) {}
    pub fn update_gravity(&mut self, step: real) {}

    pub fn set_max_contacts_reported(&mut self, size: i32) {
        self.contacts.resize(size);
        self.contact_count = 0;
    }
    pub fn reset_contact_count(&mut self) {
        self.contact_count = 0;
    }
    pub fn get_max_contacts_reported(&self) -> i32 {
        return self.contacts.len();
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
        collider_instance_id: InstanceId,
        //collider_object: Option<Object>,
        collider: Rid,
        collider_velocity_at_pos: Vector2,
        impulse: Vector2,
    ) {
    }

    pub fn add_exception(&mut self, exception: Rid) {
        self.exceptions.insert(exception);
    }
    pub fn remove_exception(&mut self, exception: Rid) {
        self.exceptions.erase(exception);
    }
    pub fn has_exception(&mut self, exception: Rid) -> bool {
        return self.exceptions.has(exception);
    }
    pub fn get_exceptions(&self) -> HashSet<Rid> {
        return self.exceptions;
    }

    pub fn set_omit_force_integration(&mut self, omit_force_integration: bool) {
        self.omit_force_integration = omit_force_integration;
    }
    pub fn get_omit_force_integration(&self) -> bool {
        return self.omit_force_integration;
    }

    pub fn apply_central_impulse(&self, impulse: Vector2) {}

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

    pub fn set_mode(&mut self, mode: BodyMode) {}
    pub fn get_mode(&self) -> BodyMode {}

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

    pub fn call_queries(&self) {}

    fn get_aabb(&self) -> Rect2 {}
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
