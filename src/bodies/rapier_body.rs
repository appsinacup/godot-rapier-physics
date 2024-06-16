use bodies::transform_scale;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::hash_set::HashSet;
use rapier::geometry::ColliderHandle;
use rapier::math::Real;

use super::rapier_area::RapierArea;
use super::PhysicsDirectBodyState;
use super::RapierDirectBodyState;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::RapierBodyParam;
use crate::servers::rapier_physics_singleton::*;
use crate::servers::rapier_project_settings::*;
use crate::spaces::rapier_space::RapierSpace;
use crate::*;
#[derive(Clone)]
pub struct Contact {
    pub local_pos: Vector,
    pub local_normal: Vector,
    pub depth: real,
    pub local_shape: i32,
    pub collider_pos: Vector,
    pub collider_shape: i32,
    pub collider_instance_id: u64,
    pub collider: Rid,
    pub local_velocity_at_pos: Vector,
    pub collider_velocity_at_pos: Vector,
    pub impulse: Vector,
}
impl Default for Contact {
    fn default() -> Self {
        Self {
            local_pos: Vector::default(),
            local_normal: Vector::default(),
            depth: 0.0,
            local_shape: 0,
            collider_pos: Vector::default(),
            collider_shape: 0,
            collider_instance_id: 0,
            collider: Rid::Invalid,
            local_velocity_at_pos: Vector::default(),
            collider_velocity_at_pos: Vector::default(),
            impulse: Vector::default(),
        }
    }
}
#[derive(Clone)]
pub struct ForceIntegrationCallbackData {
    pub callable: Callable,
    pub udata: Variant,
}
// Define the RapierBody struct
pub struct RapierBody {
    linear_damping_mode: BodyDampMode,
    angular_damping_mode: BodyDampMode,
    linear_damping: real,
    angular_damping: real,
    total_linear_damping: real,
    total_angular_damping: real,
    total_gravity: Vector,
    gravity_scale: real,
    bounce: real,
    friction: real,
    mass: real,
    mass_properties_update_pending: bool,
    inertia: Angle,
    contact_skin: real,
    center_of_mass: Vector,
    calculate_inertia: bool,
    calculate_center_of_mass: bool,
    using_area_gravity: bool,
    using_area_linear_damping: bool,
    using_area_angular_damping: bool,
    exceptions: HashSet<Rid>,
    ccd_enabled: bool,
    omit_force_integration: bool,
    active: bool,
    marked_active: bool,
    can_sleep: bool,
    constant_force: Vector,
    linear_velocity: Vector,
    impulse: Vector,
    torque: Angle,
    angular_velocity: Angle,
    constant_torque: Angle,
    to_add_angular_velocity: Angle,
    to_add_linear_velocity: Vector,
    sleep: bool,
    areas: Vec<Rid>,
    contacts: Vec<Contact>,
    contact_count: i32,
    body_state_callback: Callable,
    fi_callback_data: Option<ForceIntegrationCallbackData>,
    direct_state: Option<Gd<PhysicsDirectBodyState>>,
    base: RapierCollisionObject,
}
impl RapierBody {
    pub fn new(rid: Rid) -> Self {
        Self {
            linear_damping_mode: BodyDampMode::COMBINE,
            angular_damping_mode: BodyDampMode::COMBINE,
            linear_damping: 0.0,
            angular_damping: 0.0,
            total_linear_damping: 0.0,
            total_angular_damping: 0.0,
            total_gravity: Vector::default(),
            gravity_scale: 1.0,
            bounce: 0.0,
            friction: 1.0,
            mass: 1.0,
            mass_properties_update_pending: false,
            inertia: ANGLE_ZERO,
            contact_skin: RapierProjectSettings::get_contact_skin(),
            center_of_mass: Vector::default(),
            calculate_inertia: true,
            calculate_center_of_mass: true,
            using_area_gravity: false,
            using_area_linear_damping: false,
            using_area_angular_damping: false,
            exceptions: HashSet::default(),
            ccd_enabled: false,
            omit_force_integration: false,
            active: true,
            marked_active: false,
            can_sleep: true,
            constant_force: Vector::default(),
            linear_velocity: Vector::default(),
            impulse: Vector::default(),
            torque: ANGLE_ZERO,
            angular_velocity: ANGLE_ZERO,
            constant_torque: ANGLE_ZERO,
            to_add_angular_velocity: ANGLE_ZERO,
            to_add_linear_velocity: Vector::default(),
            sleep: false,
            areas: Vec::new(),
            contacts: Vec::new(),
            contact_count: 0,
            body_state_callback: Callable::invalid(),
            fi_callback_data: None,
            direct_state: None,
            base: RapierCollisionObject::new(rid, CollisionObjectType::Body),
        }
    }

    fn _mass_properties_changed(&mut self) {
        if self.base.mode.ord() < BodyMode::RIGID.ord() {
            return;
        }
        if self.calculate_inertia || self.calculate_center_of_mass {
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_add_to_mass_properties_update_list(self.base.get_rid());
                self.mass_properties_update_pending = true;
            }
        }
        else {
            self._apply_mass_properties(false);
        }
    }

    fn _apply_mass_properties(&mut self, force_update: bool) {
        if self.base.mode.ord() < BodyMode::RIGID.ord() || !self.base.is_valid() {
            return;
        }
        let mut inertia_value = self.inertia;
        if self.base.mode == BodyMode::RIGID_LINEAR {
            inertia_value = ANGLE_ZERO;
        }
        // Force update means local properties will be re-calculated internally,
        // it's needed for applying forces right away (otherwise it's updated on next step)
        body_set_mass_properties(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            self.mass,
            angle_to_rapier(inertia_value),
            vector_to_rapier(self.center_of_mass),
            false,
            force_update,
        );
    }

    fn _shapes_changed(&mut self) {
        self._mass_properties_changed();
        self.wakeup();
    }

    fn _apply_linear_damping(&mut self, new_value: real, apply_default: bool) {
        if let Some(space) = spaces_singleton().spaces.get(&self.base.get_space()) {
            self.total_linear_damping = new_value;
            if apply_default {
                let linear_damp: real = space
                    .get_default_area_param(AreaParameter::LINEAR_DAMP)
                    .to();
                self.total_linear_damping += linear_damp;
            }
            body_set_linear_damping(
                self.base.get_space_handle(),
                self.base.get_body_handle(),
                self.total_linear_damping,
            );
        }
    }

    fn _apply_angular_damping(&mut self, new_value: real, apply_default: bool) {
        if let Some(space) = spaces_singleton().spaces.get(&self.base.get_space()) {
            self.total_angular_damping = new_value;
            if apply_default {
                let angular_damp: real = space
                    .get_default_area_param(AreaParameter::ANGULAR_DAMP)
                    .to();
                self.total_angular_damping += angular_damp;
            }
            body_set_angular_damping(
                self.base.get_space_handle(),
                self.base.get_body_handle(),
                self.total_angular_damping,
            );
        }
    }

    fn _apply_gravity_scale(&self, new_value: real) {
        if !self.base.is_valid() {
            return;
        }
        body_set_gravity_scale(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            new_value,
            true,
        );
    }

    fn _init_collider(&self, collider_handle: ColliderHandle, space_handle: Handle) {
        // Send contact infos for dynamic bodies
        if self.base.mode.ord() >= BodyMode::KINEMATIC.ord() {
            let mut send_contacts = self.can_report_contacts();
            if godot::engine::Os::singleton().is_debug_build() {
                send_contacts = true;
            }
            collider_set_contact_force_events_enabled(space_handle, collider_handle, send_contacts);
        }
    }

    pub fn to_add_static_constant_linear_velocity(&mut self, linear_velocity: Vector) {
        self.to_add_linear_velocity = linear_velocity;
    }

    pub fn to_add_static_constant_angular_velocity(&mut self, angular_velocity: Angle) {
        self.to_add_angular_velocity = angular_velocity;
    }

    pub fn set_linear_velocity(&mut self, p_linear_velocity: Vector) {
        self.linear_velocity = p_linear_velocity;
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        if !self.base.is_valid() {
            return;
        }
        body_set_linear_velocity(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(self.linear_velocity),
        );
        self.linear_velocity = Vector::default();
    }

    pub fn get_linear_velocity(&self) -> Vector {
        if !self.base.is_valid() {
            return self.linear_velocity;
        }
        let vel =
            body_get_linear_velocity(self.base.get_space_handle(), self.base.get_body_handle());
        vector_to_godot(vel)
    }

    pub fn get_static_linear_velocity(&self) -> Vector {
        self.linear_velocity
    }

    pub fn set_angular_velocity(&mut self, p_angular_velocity: Angle) {
        self.angular_velocity = p_angular_velocity;
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        if !self.base.is_valid() {
            return;
        }
        body_set_angular_velocity(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(self.angular_velocity),
        );
        self.angular_velocity = ANGLE_ZERO;
    }

    pub fn get_angular_velocity(&self) -> Angle {
        if !self.base.is_valid() {
            return self.angular_velocity;
        }
        angle_to_godot(body_get_angular_velocity(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
        ))
    }

    pub fn get_static_angular_velocity(&self) -> Angle {
        self.angular_velocity
    }

    pub fn set_state_sync_callback(&mut self, p_callable: Callable) {
        self.body_state_callback = p_callable;
    }

    pub fn get_state_sync_callback(&self) -> &Callable {
        &self.body_state_callback
    }

    pub fn set_force_integration_callback(&mut self, callable: Callable, udata: Variant) {
        if callable.is_valid() {
            self.fi_callback_data = Some(ForceIntegrationCallbackData { callable, udata });
        }
        else {
            self.fi_callback_data = None;
        }
    }

    pub fn get_force_integration_callback(&self) -> Option<&ForceIntegrationCallbackData> {
        self.fi_callback_data.as_ref()
    }

    pub fn get_direct_state(&mut self) -> Option<&Gd<PhysicsDirectBodyState>> {
        if self.direct_state.is_none() {
            let mut direct_space_state = RapierDirectBodyState::new_alloc();
            direct_space_state.bind_mut().set_body(self.base.get_rid());
            self.direct_state = Some(direct_space_state.upcast());
        }
        self.direct_state.as_ref()
    }

    pub fn add_area(&mut self, p_area: &RapierArea) {
        self.base.area_detection_counter += 1;
        if p_area.has_any_space_override() {
            // TODO sort
            let area_rid = p_area.get_base().get_rid();
            self.areas.push(area_rid);
            self.on_area_updated(area_rid);
        }
    }

    pub fn remove_area(&mut self, area: Rid) {
        if self.base.area_detection_counter == 0 {
            godot_error!("Area detection counter is zero.");
            return;
        }
        self.base.area_detection_counter -= 1;
        self.areas.retain(|&x| x != area);
        self.on_area_updated(area);
    }

    pub fn on_area_updated(&mut self, _area: Rid) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            space.body_add_to_area_update_list(self.base.get_rid());
        }
    }

    pub fn update_area_override(&mut self) {
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            space.body_remove_from_area_update_list(self.base.get_rid());
        }
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        if !self.base.get_space_handle().is_valid() {
            godot_error!("Space handle is invalid.");
            return;
        }
        // Reset area override flags.
        self.using_area_gravity = false;
        self.using_area_linear_damping = false;
        self.using_area_angular_damping = false;
        // Start with no effect.
        self.total_gravity = Vector::default();
        let mut total_linear_damping = 0.0;
        let mut total_angular_damping = 0.0;
        // Combine gravity and damping from overlapping areas in priority order.
        let ac = self.areas.len();
        let mut gravity_done = false; // always calculate to be able to change scale on area gravity
        let mut linear_damping_done = self.linear_damping_mode == BodyDampMode::REPLACE;
        let mut angular_damping_done = self.angular_damping_mode == BodyDampMode::REPLACE;
        if ac > 0 {
            let mut areas = self.areas.clone();
            areas.reverse();
            for area_rid in areas {
                if let Some(area) = bodies_singleton().collision_objects.get(&area_rid) {
                    if let Some(aa) = area.get_area() {
                        if !gravity_done {
                            let area_gravity_mode =
                                aa.get_param(AreaParameter::GRAVITY_OVERRIDE_MODE).to();
                            if area_gravity_mode != AreaSpaceOverrideMode::DISABLED {
                                let area_gravity =
                                    aa.compute_gravity(self.base.get_transform().origin);
                                match area_gravity_mode {
                                    AreaSpaceOverrideMode::COMBINE
                                    | AreaSpaceOverrideMode::COMBINE_REPLACE => {
                                        self.using_area_gravity = true;
                                        self.total_gravity += area_gravity;
                                        gravity_done = area_gravity_mode
                                            == AreaSpaceOverrideMode::COMBINE_REPLACE;
                                    }
                                    AreaSpaceOverrideMode::REPLACE
                                    | AreaSpaceOverrideMode::REPLACE_COMBINE => {
                                        self.using_area_gravity = true;
                                        self.total_gravity = area_gravity;
                                        gravity_done =
                                            area_gravity_mode == AreaSpaceOverrideMode::REPLACE;
                                    }
                                    _ => {}
                                }
                            }
                        }
                        if !linear_damping_done {
                            let area_linear_damping_mode =
                                aa.get_param(AreaParameter::LINEAR_DAMP_OVERRIDE_MODE).to();
                            if area_linear_damping_mode != AreaSpaceOverrideMode::DISABLED {
                                let area_linear_damping = aa.get_linear_damp();
                                match area_linear_damping_mode {
                                    AreaSpaceOverrideMode::COMBINE
                                    | AreaSpaceOverrideMode::COMBINE_REPLACE => {
                                        self.using_area_linear_damping = true;
                                        total_linear_damping += area_linear_damping;
                                        linear_damping_done = area_linear_damping_mode
                                            == AreaSpaceOverrideMode::COMBINE_REPLACE;
                                    }
                                    AreaSpaceOverrideMode::REPLACE
                                    | AreaSpaceOverrideMode::REPLACE_COMBINE => {
                                        self.using_area_linear_damping = true;
                                        total_linear_damping = area_linear_damping;
                                        linear_damping_done = area_linear_damping_mode
                                            == AreaSpaceOverrideMode::REPLACE;
                                    }
                                    _ => {}
                                }
                            }
                        }
                        if !angular_damping_done {
                            let area_angular_damping_mode =
                                aa.get_param(AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE).to();
                            if area_angular_damping_mode != AreaSpaceOverrideMode::DISABLED {
                                let area_angular_damping = aa.get_angular_damp();
                                match area_angular_damping_mode {
                                    AreaSpaceOverrideMode::COMBINE
                                    | AreaSpaceOverrideMode::COMBINE_REPLACE => {
                                        self.using_area_angular_damping = true;
                                        total_angular_damping += area_angular_damping;
                                        angular_damping_done = area_angular_damping_mode
                                            == AreaSpaceOverrideMode::COMBINE_REPLACE;
                                    }
                                    AreaSpaceOverrideMode::REPLACE
                                    | AreaSpaceOverrideMode::REPLACE_COMBINE => {
                                        self.using_area_angular_damping = true;
                                        total_angular_damping = area_angular_damping;
                                        angular_damping_done = area_angular_damping_mode
                                            == AreaSpaceOverrideMode::REPLACE;
                                    }
                                    _ => {}
                                }
                            }
                        }
                        if gravity_done && linear_damping_done && angular_damping_done {
                            break;
                        }
                    }
                }
            }
        }
        // Override or combine damping with body's values.
        total_linear_damping += self.linear_damping;
        total_angular_damping += self.angular_damping;
        // Apply to the simulation.
        self._apply_linear_damping(total_linear_damping, !linear_damping_done);
        self._apply_angular_damping(total_angular_damping, !angular_damping_done);
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            if self.using_area_gravity {
                // Add default gravity from space.
                if !gravity_done {
                    self.total_gravity += space.get_default_area_param(AreaParameter::GRAVITY).to();
                }
                // Apply gravity scale to computed value.
                self.total_gravity *= self.gravity_scale;
                // Disable simulation gravity and apply it manually instead.
                self._apply_gravity_scale(0.0);
                space.body_add_to_gravity_update_list(self.base.get_rid());
            }
            else {
                // Enable simulation gravity.
                self._apply_gravity_scale(self.gravity_scale);
                space.body_remove_from_gravity_update_list(self.base.get_rid());
            }
        }
    }

    pub fn update_gravity(&mut self, p_step: real) {
        if !self.using_area_gravity {
            return;
        }
        if !self.areas.is_empty() {
            self.update_area_override();
        }
        if !self.base.is_valid() {
            return;
        }
        let gravity_impulse = self.total_gravity * self.mass * p_step;
        body_apply_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(gravity_impulse),
        );
    }

    pub fn set_max_contacts_reported(&mut self, size: i32) {
        self.contacts.resize(size as usize, Contact::default());
        self.contact_count = 0;
    }

    pub fn reset_contact_count(&mut self) {
        self.contact_count = 0;
    }

    pub fn get_max_contacts_reported(&self) -> i32 {
        self.contacts.len() as i32
    }

    pub fn can_report_contacts(&self) -> bool {
        !self.contacts.is_empty()
    }

    pub fn add_contact(
        &mut self,
        local_pos: Vector,
        local_normal: Vector,
        depth: real,
        local_shape: i32,
        local_velocity_at_pos: Vector,
        collider_pos: Vector,
        collider_shape: i32,
        collider_instance_id: u64,
        collider: Rid,
        collider_velocity_at_pos: Vector,
        impulse: Vector,
    ) {
        let c_max = self.contacts.len();
        if c_max == 0 {
            return;
        }
        let mut idx = -1;
        if self.contact_count < c_max as i32 {
            idx = self.contact_count;
            self.contact_count += 1;
        }
        else {
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
        self.exceptions.contains(&exception)
    }

    pub fn get_exceptions(&self) -> &HashSet<Rid> {
        &self.exceptions
    }

    pub fn set_omit_force_integration(&mut self, omit_force_integration: bool) {
        self.omit_force_integration = omit_force_integration;
    }

    pub fn get_omit_force_integration(&self) -> bool {
        self.omit_force_integration
    }

    pub fn apply_central_impulse(&mut self, p_impulse: Vector) {
        if !self.base.is_valid() {
            self.impulse += p_impulse;
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_apply_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_impulse),
        );
        self.impulse = Vector::default();
    }

    pub fn apply_impulse(&mut self, p_impulse: Vector, p_position: Vector) {
        if !self.base.is_valid() {
            self.impulse += p_impulse;
            self.torque += (p_position - self.get_center_of_mass()).cross(p_impulse);
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_apply_impulse_at_point(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_impulse),
            vector_to_rapier(p_position),
        );
        self.impulse = Vector::default();
        self.torque = ANGLE_ZERO;
    }

    pub fn apply_torque_impulse(&mut self, p_torque: Angle) {
        if !self.base.is_valid() {
            self.torque += p_torque;
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_apply_torque_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
        self.torque = ANGLE_ZERO;
    }

    pub fn apply_central_force(&mut self, p_force: Vector) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.impulse += p_force * last_delta;
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_apply_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force * last_delta),
        );
        self.impulse = Vector::default();
    }

    pub fn apply_force(&mut self, p_force: Vector, p_position: Vector) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.impulse += p_force * last_delta;
            self.torque += (p_position - self.get_center_of_mass()).cross(p_force) * last_delta;
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_apply_impulse_at_point(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force * last_delta),
            vector_to_rapier(p_position),
        );
        self.impulse = Vector::default();
        self.torque = ANGLE_ZERO;
    }

    pub fn apply_torque(&mut self, p_torque: Angle) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.torque += p_torque * last_delta;
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_apply_torque_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque * last_delta),
        );
        self.torque = ANGLE_ZERO;
    }

    pub fn add_constant_central_force(&mut self, p_force: Vector) {
        self.constant_force += p_force;
        if !self.base.is_valid() {
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_add_force(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
        );
    }

    pub fn add_constant_force(&mut self, p_force: Vector, p_position: Vector) {
        self.constant_torque += (p_position - self.get_center_of_mass()).cross(p_force);
        self.constant_force += p_force;
        if !self.base.is_valid() {
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_add_force_at_point(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
            vector_to_rapier(p_position),
        );
    }

    pub fn add_constant_torque(&mut self, p_torque: Angle) {
        self.constant_torque += p_torque;
        if !self.base.is_valid() {
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_add_torque(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
    }

    pub fn set_constant_force(&mut self, p_force: Vector) {
        self.constant_force = p_force;
        if !self.base.is_valid() {
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_reset_forces(self.base.get_space_handle(), self.base.get_body_handle());
        body_add_force(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
        );
    }

    pub fn get_constant_force(&self) -> Vector {
        if !self.base.is_valid() {
            return self.constant_force;
        }
        let force =
            body_get_constant_force(self.base.get_space_handle(), self.base.get_body_handle());
        vector_to_godot(force)
    }

    pub fn set_constant_torque(&mut self, p_torque: Angle) {
        self.constant_torque = p_torque;
        if !self.base.is_valid() {
            return;
        }
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true);
        }
        body_reset_torques(self.base.get_space_handle(), self.base.get_body_handle());
        body_add_torque(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
    }

    pub fn get_constant_torque(&self) -> Angle {
        if !self.base.is_valid() {
            return self.constant_torque;
        }
        angle_to_godot(body_get_constant_torque(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
        ))
    }

    pub fn set_active(&mut self, p_active: bool, space: &mut RapierSpace) {
        if self.active == p_active {
            return;
        }
        self.active = p_active;
        if self.active {
            if self.base.mode == BodyMode::STATIC {
                // Static bodies can't be active.
                self.active = false;
            }
            else {
                space.body_add_to_active_list(self.base.get_rid());
            }
        }
        else {
            space.body_remove_from_active_list(self.base.get_rid());
        }
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn set_can_sleep(&mut self, p_can_sleep: bool) {
        if !self.base.is_valid() {
            return;
        }
        body_set_can_sleep(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            p_can_sleep,
        );
    }

    pub fn on_marked_active(&mut self) {
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        self.marked_active = true;
        if !self.active {
            self.active = true;
            if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                space.body_add_to_active_list(self.base.get_rid());
            }
        }
    }

    pub fn on_update_active(&mut self, space: &mut RapierSpace) {
        if !self.marked_active {
            self.set_active(false, space);
            return;
        }
        self.marked_active = false;
        self.base.update_transform();
        space.body_add_to_state_query_list(self.base.get_rid());
        if self.base.mode.ord() >= BodyMode::RIGID.ord() {
            if self.to_add_angular_velocity != ANGLE_ZERO {
                self.set_angular_velocity(
                    self.get_angular_velocity() + self.to_add_angular_velocity,
                );
                self.to_add_angular_velocity = ANGLE_ZERO;
            }
            if self.to_add_linear_velocity != Vector::default() {
                self.set_linear_velocity(self.get_linear_velocity() + self.to_add_linear_velocity);
                self.to_add_linear_velocity = Vector::default();
            }
        }
    }

    pub fn wakeup(&mut self) {
        self.sleep = false;
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        if !self.base.is_valid() {
            return;
        }
        body_wake_up(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            true,
        );
    }

    pub fn force_sleep(&mut self) {
        self.sleep = true;
        if !self.base.is_valid() {
            return;
        }
        body_force_sleep(self.base.get_space_handle(), self.base.get_body_handle());
    }

    pub fn set_param(&mut self, p_param: BodyParameter, p_value: Variant) {
        match p_param {
            BodyParameter::BOUNCE | BodyParameter::FRICTION => {
                if p_value.get_type() != VariantType::FLOAT {
                    return;
                }
                if p_param == BodyParameter::BOUNCE {
                    self.bounce = p_value.to();
                }
                else {
                    self.friction = p_value.to();
                }
                if !self.base.is_valid() {
                    return;
                }
                let mat = self._init_material();
                body_update_material(
                    self.base.get_space_handle(),
                    self.base.get_body_handle(),
                    &mat,
                );
            }
            BodyParameter::MASS => {
                if p_value.get_type() != VariantType::FLOAT {
                    return;
                }
                let mass_value = p_value.to();
                if mass_value <= 0.0 {
                    return;
                }
                self.mass = mass_value;
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self._mass_properties_changed();
                }
            }
            BodyParameter::INERTIA => {
                if p_value.get_type() != VariantType::FLOAT {
                    return;
                }
                let inertia_value = p_value.to();
                if inertia_value == ANGLE_ZERO {
                    self.calculate_inertia = true;
                }
                else {
                    self.calculate_inertia = false;
                    self.inertia = inertia_value;
                }
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self._mass_properties_changed();
                }
            }
            BodyParameter::CENTER_OF_MASS => {
                if p_value.get_type() != VariantType::VECTOR2 {
                    return;
                }
                self.center_of_mass = p_value.to();
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self._mass_properties_changed();
                }
            }
            BodyParameter::GRAVITY_SCALE => {
                if p_value.get_type() != VariantType::FLOAT {
                    return;
                }
                let new_gravity_scale = p_value.to();
                if self.gravity_scale != new_gravity_scale {
                    self.gravity_scale = new_gravity_scale;
                    if !self.using_area_gravity {
                        self._apply_gravity_scale(self.gravity_scale);
                    }
                }
            }
            BodyParameter::LINEAR_DAMP_MODE => {
                if p_value.get_type() != VariantType::INT {
                    return;
                }
                let mode_value = p_value.to();
                if self.linear_damping_mode.ord() != mode_value {
                    self.linear_damping_mode = BodyDampMode::from_ord(mode_value);
                    if self.linear_damping_mode == BodyDampMode::REPLACE {
                        self.using_area_linear_damping = false;
                    }
                    if self.using_area_linear_damping {
                        // Update linear damping from areas
                    }
                    else {
                        self._apply_linear_damping(self.linear_damping, true);
                    }
                }
            }
            BodyParameter::ANGULAR_DAMP_MODE => {
                if p_value.get_type() != VariantType::INT {
                    return;
                }
                let mode_value = p_value.to();
                if self.angular_damping_mode.ord() != mode_value {
                    self.angular_damping_mode = BodyDampMode::from_ord(mode_value);
                    if self.angular_damping_mode == BodyDampMode::REPLACE {
                        self.using_area_angular_damping = false;
                    }
                    if self.using_area_angular_damping {
                        // Update angular damping from areas
                    }
                    else {
                        self._apply_angular_damping(self.angular_damping, true);
                    }
                }
            }
            BodyParameter::LINEAR_DAMP => {
                if p_value.get_type() != VariantType::FLOAT {
                    return;
                }
                let new_value = p_value.to();
                if new_value != self.linear_damping {
                    self.linear_damping = new_value;
                    if !self.using_area_linear_damping {
                        self._apply_linear_damping(self.linear_damping, true);
                    }
                }
            }
            BodyParameter::ANGULAR_DAMP => {
                if p_value.get_type() != VariantType::FLOAT {
                    return;
                }
                let new_value = p_value.to();
                if new_value != self.angular_damping {
                    self.angular_damping = new_value;
                    if !self.using_area_angular_damping {
                        self._apply_angular_damping(self.angular_damping, true);
                    }
                }
            }
            _ => {}
        }
    }

    pub fn get_param(&self, p_param: BodyParameter) -> Variant {
        match p_param {
            BodyParameter::BOUNCE => self.bounce.to_variant(),
            BodyParameter::FRICTION => self.friction.to_variant(),
            BodyParameter::MASS => self.mass.to_variant(),
            BodyParameter::INERTIA => self.inertia.to_variant(),
            BodyParameter::CENTER_OF_MASS => self.center_of_mass.to_variant(),
            BodyParameter::GRAVITY_SCALE => self.gravity_scale.to_variant(),
            BodyParameter::LINEAR_DAMP_MODE => self.linear_damping_mode.to_variant(),
            BodyParameter::ANGULAR_DAMP_MODE => self.angular_damping_mode.to_variant(),
            BodyParameter::LINEAR_DAMP => self.linear_damping.to_variant(),
            BodyParameter::ANGULAR_DAMP => self.angular_damping.to_variant(),
            _ => 0.0.to_variant(),
        }
    }

    pub fn set_extra_param(&mut self, p_param: RapierBodyParam, p_value: Variant) {
        match p_param {
            RapierBodyParam::ContactSkin => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                self.contact_skin = p_value.to();
                let mat = self._init_material();
                let body_handle = self.base.get_body_handle();
                let space_handle = self.base.get_space_handle();
                if !self.base.is_valid() {
                    body_update_material(space_handle, body_handle, &mat);
                }
            }
        }
    }

    pub fn get_extra_param(&self, p_param: RapierBodyParam) -> Variant {
        match p_param {
            RapierBodyParam::ContactSkin => self.contact_skin.to_variant(),
        }
    }

    pub fn set_mode(&mut self, p_mode: BodyMode) {
        if self.base.mode == p_mode {
            return;
        }
        let prev_mode = self.base.mode;
        self.base.mode = p_mode;
        let rid = self.base.get_rid();
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            match p_mode {
                BodyMode::KINEMATIC => {
                    body_change_mode(
                        space.get_handle(),
                        self.base.get_body_handle(),
                        BodyType::Kinematic,
                        true,
                    );
                }
                BodyMode::STATIC => {
                    body_change_mode(
                        space.get_handle(),
                        self.base.get_body_handle(),
                        BodyType::Static,
                        true,
                    );
                }
                BodyMode::RIGID | BodyMode::RIGID_LINEAR => {
                    body_change_mode(
                        space.get_handle(),
                        self.base.get_body_handle(),
                        BodyType::Dynamic,
                        true,
                    );
                }
                _ => {}
            }
            if p_mode == BodyMode::STATIC {
                self.force_sleep();
                if self.marked_active {
                    return;
                }
                space.body_remove_from_active_list(rid);
                space.body_remove_from_mass_properties_update_list(rid);
                space.body_remove_from_gravity_update_list(rid);
                space.body_remove_from_area_update_list(rid);
                return;
            }
            if self.active && prev_mode == BodyMode::STATIC {
                space.body_add_to_active_list(rid);
            }
        }
        if p_mode.ord() >= BodyMode::RIGID.ord() {
            self._mass_properties_changed();
            if self.base.get_space_handle().is_valid() {
                self.update_area_override();
                self._apply_gravity_scale(self.gravity_scale);
            }
        }
    }

    pub fn set_state(&mut self, p_state: BodyState, p_variant: Variant) {
        match p_state {
            BodyState::TRANSFORM => {
                let transform = p_variant.to();
                self.base.set_transform(transform, true);
            }
            BodyState::LINEAR_VELOCITY => {
                self.set_linear_velocity(p_variant.to());
            }
            BodyState::ANGULAR_VELOCITY => {
                self.set_angular_velocity(p_variant.to());
            }
            BodyState::SLEEPING => {
                if self.base.mode == BodyMode::STATIC {
                    return;
                }
                self.sleep = p_variant.to();
                if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                    if self.sleep {
                        if self.can_sleep {
                            self.force_sleep();
                            self.set_active(false, space);
                        }
                    }
                    else if self.base.mode != BodyMode::STATIC {
                        self.wakeup();
                        self.set_active(true, space);
                    }
                }
            }
            BodyState::CAN_SLEEP => {
                self.can_sleep = p_variant.to();
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.set_can_sleep(self.can_sleep);
                    if !self.active && !self.can_sleep {
                        self.wakeup();
                        if let Some(space) =
                            spaces_singleton().spaces.get_mut(&self.base.get_space())
                        {
                            self.set_active(true, space);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    pub fn get_state(&self, p_state: BodyState) -> Variant {
        match p_state {
            BodyState::TRANSFORM => self.base.get_transform().to_variant(),
            BodyState::LINEAR_VELOCITY => self.get_linear_velocity().to_variant(),
            BodyState::ANGULAR_VELOCITY => self.get_angular_velocity().to_variant(),
            BodyState::SLEEPING => (!self.active).to_variant(),
            BodyState::CAN_SLEEP => self.can_sleep.to_variant(),
            _ => Variant::nil(),
        }
    }

    pub fn set_continuous_collision_detection_mode(&mut self, enabled: bool) {
        self.ccd_enabled = enabled;
        if !self.base.is_valid() {
            return;
        }
        body_set_ccd_enabled(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            self.ccd_enabled,
        );
    }

    pub fn get_continuous_collision_detection_mode(&self) -> bool {
        self.ccd_enabled
    }

    pub fn update_mass_properties(&mut self, force_update: bool) {
        self.mass_properties_update_pending = false;
        if self.base.mode.ord() < BodyMode::RIGID.ord() {
            return;
        }
        let mut total_area = 0.0;
        let shape_count = self.base.get_shape_count() as usize;
        for i in 0..shape_count {
            if self.base.is_shape_disabled(i) {
                continue;
            }
            if let Some(shape) = shapes_singleton().shapes.get(&self.base.get_shape(i)) {
                total_area += shape.get_base().get_aabb_area();
            }
        }
        if self.calculate_center_of_mass {
            self.center_of_mass = Vector::default();
            if total_area != 0.0 {
                for i in 0..shape_count {
                    if self.base.is_shape_disabled(i) {
                        continue;
                    }
                    if let Some(shape) = shapes_singleton().shapes.get(&self.base.get_shape(i)) {
                        let shape_area = shape.get_base().get_aabb_area();
                        if shape_area == 0.0 || self.mass == 0.0 {
                            continue;
                        }
                        let shape_mass = shape_area * self.mass / total_area;
                        // NOTE: we assume that the shape origin is also its center of mass.
                        self.center_of_mass += shape_mass * self.base.get_shape_transform(i).origin;
                    }
                }
                self.center_of_mass /= self.mass;
            }
        }
        if self.calculate_inertia {
            self.inertia = ANGLE_ZERO;
            if total_area != 0.0 {
                for i in 0..shape_count {
                    if self.base.is_shape_disabled(i) {
                        continue;
                    }
                    if let Some(shape) = shapes_singleton().shapes.get(&self.base.get_shape(i)) {
                        let shape_area = shape.get_base().get_aabb_area();
                        if shape_area == 0.0 || self.mass == 0.0 {
                            continue;
                        }
                        let shape_mass = shape_area * self.mass / total_area;
                        let mtx = self.base.get_shape_transform(i);
                        let scale = transform_scale(&mtx);
                        self.inertia += self.get_inertia_for_shape(
                            shape.get_moment_of_inertia(shape_mass, scale),
                            shape_mass,
                            mtx,
                            i,
                        );
                    }
                }
            }
        }
        self._apply_mass_properties(force_update);
    }

    #[cfg(feature = "dim2")]
    fn get_inertia_for_shape(
        &self,
        moment_of_inertia: Angle,
        shape_mass: Real,
        shape_transform: Transform,
        i: usize,
    ) -> Real {
        let mtx = shape_transform;
        let scale = transform_scale(&mtx);
        let shape_origin = mtx.origin - self.center_of_mass;
        moment_of_inertia + shape_mass * shape_origin.length_squared()
    }

    #[cfg(feature = "dim3")]
    fn get_inertia_for_shape(
        &self,
        moment_of_inertia: Angle,
        shape_mass: Real,
        shape_transform: Transform,
        i: usize,
    ) -> Vector3 {
        let mut shape_inertia_tensor = Basis::from_scale(moment_of_inertia);
        let shape_transform = shape_transform;
        let shape_basis = shape_transform.basis.orthonormalized();
        // NOTE: we don't take the scale of collision shapes into account when computing the inertia tensor!
        shape_inertia_tensor = shape_basis * shape_inertia_tensor * shape_basis.transposed();
        let shape_origin = shape_transform.origin - self.center_of_mass;
        let shape_outer = shape_origin.outer(shape_origin);
        let mut shape_dot = Basis::IDENTITY * (shape_origin.dot(shape_origin));
        shape_dot.set_col_a(shape_dot.col_a() - shape_outer.col_a());
        shape_dot.set_col_b(shape_dot.col_b() - shape_outer.col_b());
        shape_dot.set_col_c(shape_dot.col_c() - shape_outer.col_c());
        shape_dot *= shape_mass;
        let mut inertia_tensor = shape_inertia_tensor;
        inertia_tensor.set_col_a(inertia_tensor.col_a() + shape_dot.col_a());
        inertia_tensor.set_col_b(inertia_tensor.col_b() + shape_dot.col_b());
        inertia_tensor.set_col_c(inertia_tensor.col_c() + shape_dot.col_c());
        //return inertia_tensor.diagonalize().transposed();
        // TODO
        moment_of_inertia
    }

    pub fn reset_mass_properties(&mut self) {
        if self.calculate_inertia && self.calculate_center_of_mass {
            // Nothing to do, already calculated
            return;
        }
        self.calculate_inertia = true;
        self.calculate_center_of_mass = true;
        self._mass_properties_changed();
    }

    pub fn get_center_of_mass(&self) -> Vector {
        self.center_of_mass
    }

    pub fn get_inv_mass(&self) -> real {
        if self.mass != 0.0 {
            return 1.0 / self.mass;
        }
        0.0
    }

    #[cfg(feature = "dim2")]
    pub fn get_inv_inertia(&self) -> Angle {
        if self.inertia != ANGLE_ZERO {
            return 1.0 / self.inertia;
        }
        ANGLE_ZERO
    }

    #[cfg(feature = "dim3")]
    pub fn get_inv_inertia(&self) -> Angle {
        if self.inertia != ANGLE_ZERO {
            return Vector3::new(
                1.0 / self.inertia.x,
                1.0 / self.inertia.y,
                1.0 / self.inertia.z,
            );
        }
        ANGLE_ZERO
    }

    #[cfg(feature = "dim2")]
    pub fn get_velocity_at_local_point(&self, rel_pos: Vector) -> Vector {
        let linear_velocity = self.get_linear_velocity();
        let angular_velocity = self.get_angular_velocity();
        linear_velocity
            + Vector::new(
                -angular_velocity * (rel_pos.y - self.center_of_mass.y),
                angular_velocity * (rel_pos.x - self.center_of_mass.x),
            )
    }

    #[cfg(feature = "dim3")]
    pub fn get_velocity_at_local_point(&self, rel_pos: Vector) -> Vector {
        let linear_velocity = self.get_linear_velocity();
        let angular_velocity = self.get_angular_velocity();
        linear_velocity + angular_velocity.cross(rel_pos - self.center_of_mass)
    }

    pub fn get_aabb(&self) -> Rect {
        let mut shapes_found = false;
        let mut body_aabb = Rect::default();
        let shape_count = self.base.get_shape_count() as usize;
        for i in 0..shape_count {
            if self.base.is_shape_disabled(i) {
                continue;
            }
            if let Some(shape) = shapes_singleton().shapes.get(&self.base.get_shape(i)) {
                if !shapes_found {
                    // TODO not 100% correct, we don't take into consideration rotation here.
                    body_aabb = shape
                        .get_base()
                        .get_aabb(self.base.get_shape_transform(i).origin);
                    shapes_found = true;
                }
                else {
                    // TODO not 100% correct, we don't take into consideration rotation here.
                    body_aabb = body_aabb.merge(
                        shape
                            .get_base()
                            .get_aabb(self.base.get_shape_transform(i).origin),
                    );
                }
            }
        }
        body_aabb
    }

    pub fn total_linear_damping(&self) -> real {
        self.total_linear_damping
    }

    pub fn total_angular_damping(&self) -> real {
        self.total_angular_damping
    }

    pub fn total_gravity(&self) -> Vector {
        self.total_gravity
    }

    pub fn gravity_scale(&self) -> real {
        self.gravity_scale
    }

    pub fn using_area_gravity(&self) -> bool {
        self.using_area_gravity
    }

    pub fn contact_count(&self) -> i32 {
        self.contact_count
    }

    pub fn contacts(&self) -> &Vec<Contact> {
        &self.contacts
    }
}
impl IRapierCollisionObject for RapierBody {
    fn get_base(&self) -> &RapierCollisionObject {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierCollisionObject {
        &mut self.base
    }

    fn recreate_shapes(&mut self) {
        for i in 0..self.base.get_shape_count() as usize {
            if self.base.shapes[i].disabled {
                continue;
            }
            self.base.shapes[i].collider_handle = self.create_shape(self.base.shapes[i], i);
            if self.base.shapes[i].collider_handle == ColliderHandle::invalid() {
                self.base.shapes[i].disabled = true;
                continue;
            }
            self.base.update_shape_transform(&self.base.shapes[i]);
        }
    }

    fn set_space(&mut self, space: Rid) {
        // delete previous space
        if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
            space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            space.body_remove_from_gravity_update_list(self.base.get_rid());
            space.body_remove_from_active_list(self.base.get_rid());
            space.body_remove_from_state_query_list(self.base.get_rid());
            space.body_remove_from_area_update_list(self.base.get_rid());
        }
        self.base._set_space(space);
        self.recreate_shapes();
        if self.base.get_space_handle().is_valid()
            && self.base.mode.ord() >= BodyMode::KINEMATIC.ord()
        {
            if !self.can_sleep {
                self.set_can_sleep(false);
            }
            if self.active || !self.sleep {
                self.wakeup();
                if let Some(space) = spaces_singleton().spaces.get_mut(&self.base.get_space()) {
                    space.body_add_to_active_list(self.base.get_rid());
                }
            }
            else if self.can_sleep && self.sleep {
                self.force_sleep();
            }
            if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                self._apply_gravity_scale(self.gravity_scale);
                if self.linear_damping_mode == BodyDampMode::COMBINE {
                    self._apply_linear_damping(self.linear_damping, true);
                }
                if self.angular_damping_mode == BodyDampMode::COMBINE {
                    self._apply_angular_damping(self.angular_damping, true);
                }
                self._mass_properties_changed();
                if self.linear_velocity != Vector::default() {
                    self.set_linear_velocity(self.linear_velocity);
                }
                if self.angular_velocity != ANGLE_ZERO {
                    self.set_angular_velocity(self.angular_velocity);
                }
                if self.constant_force != Vector::default() {
                    self.set_constant_force(self.constant_force);
                }
                if self.constant_torque != ANGLE_ZERO {
                    self.set_constant_torque(self.constant_torque);
                }
                if self.impulse != Vector::default() {
                    self.apply_impulse(self.impulse, Vector::default());
                }
                if self.torque != ANGLE_ZERO {
                    self.apply_torque_impulse(self.torque);
                }
                self.set_continuous_collision_detection_mode(self.ccd_enabled);
                body_update_material(
                    self.base.get_space_handle(),
                    self.base.get_body_handle(),
                    &self._init_material(),
                );
            }
        }
    }

    fn get_body(&self) -> Option<&RapierBody> {
        Some(self)
    }

    fn get_area(&self) -> Option<&RapierArea> {
        None
    }

    fn get_mut_body(&mut self) -> Option<&mut RapierBody> {
        Some(self)
    }

    fn get_mut_area(&mut self) -> Option<&mut RapierArea> {
        None
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
            collider_handle: ColliderHandle::invalid(),
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
            self.base.shapes[p_index].collider_handle = self.base._destroy_shape(shape, p_index);
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
            }
            else {
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
        shape.collider_handle = ColliderHandle::invalid();
        if let Some(shape) = shapes_singleton().shapes.get_mut(&shape.shape) {
            shape.get_mut_base().remove_owner(self.base.get_rid());
        }
        self.base.shapes.remove(p_index);
        if self.base.get_space_handle().is_valid() {
            self._shapes_changed();
        }
    }

    fn create_shape(
        &mut self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
    ) -> ColliderHandle {
        if !self.base.get_space_handle().is_valid() {
            return ColliderHandle::invalid();
        }
        let mat = self._init_material();
        let handle = self.base._create_shape(shape, p_shape_index, mat);
        self._init_collider(handle, self.base.get_space_handle());
        handle
    }

    fn _init_material(&self) -> Material {
        Material {
            friction: self.friction,
            restitution: self.bounce,
            contact_skin: self.contact_skin,
        }
    }

    fn _shapes_changed(&mut self) {
        self._mass_properties_changed();
        self.wakeup();
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
            if self.base.shapes[i].collider_handle != ColliderHandle::invalid() {
                self.base.shapes[i].collider_handle = self.base._destroy_shape(shape, i);
            }
            self.base
                ._create_shape(self.base.shapes[i], i, self._init_material());
            self.base.update_shape_transform(&self.base.shapes[i]);
        }
        self._shapes_changed();
    }
}
impl Drop for RapierBody {
    fn drop(&mut self) {
        if let Some(direct_state) = &self.direct_state {
            // TODO
            direct_state.clone().free();
        }
    }
}
