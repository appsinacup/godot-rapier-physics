use bodies::transform_scale;
use bodies::variant_to_float;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::hash_set::HashSet;
use rapier::geometry::ColliderHandle;
use rapier::math::Real;
use servers::rapier_physics_server_extra::PhysicsCollisionObjects;
use servers::rapier_physics_server_extra::PhysicsShapes;
use servers::rapier_physics_server_extra::PhysicsSpaces;

use super::rapier_area::RapierArea;
use super::PhysicsDirectBodyState;
use super::RapierDirectBodyState;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::RapierBodyParam;
use crate::servers::rapier_project_settings::*;
use crate::spaces::rapier_space::RapierSpace;
use crate::*;
#[derive(Debug, Clone, Copy, PartialEq)]
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
pub struct AreaOverrideSettings {
    using_area_gravity: bool,
    using_area_linear_damping: bool,
    using_area_angular_damping: bool,
    total_gravity: Vector,
    total_linear_damping: real,
    total_angular_damping: real,
    gravity_done: bool,
    linear_damping_done: bool,
    angular_damping_done: bool,
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
//#[derive(Serialize, Deserialize, Debug, Clone, Copy, Debug, PartialEq)]
pub struct ForceIntegrationCallbackData {
    pub callable: Callable,
    pub udata: Variant,
}
//#[derive(Serialize, Deserialize, Debug, Clone, Copy, Debug, PartialEq)]
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
    #[cfg(feature = "dim3")]
    inv_inertia_tensor: Basis,
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
            #[cfg(feature = "dim3")]
            inv_inertia_tensor: Basis::IDENTITY,
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

    fn mass_properties_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        if self.base.mode.ord() < BodyMode::RIGID.ord() {
            return;
        }
        if self.calculate_inertia || self.calculate_center_of_mass {
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
                space.body_add_to_mass_properties_update_list(self.base.get_rid());
                self.mass_properties_update_pending = true;
            }
        } else {
            self.apply_mass_properties(false, physics_engine);
        }
    }

    fn apply_mass_properties(&mut self, force_update: bool, physics_engine: &mut PhysicsEngine) {
        if self.base.mode.ord() < BodyMode::RIGID.ord() || !self.base.is_valid() {
            return;
        }
        let mut inertia_value = self.inertia;
        if self.base.mode == BodyMode::RIGID_LINEAR {
            inertia_value = ANGLE_ZERO;
        }
        // Force update means local properties will be re-calculated internally,
        // it's needed for applying forces right away (otherwise it's updated on next step)
        physics_engine.body_set_mass_properties(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            self.mass,
            angle_to_rapier(inertia_value),
            vector_to_rapier(self.center_of_mass),
            false,
            force_update,
        );
    }

    fn shapes_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        self.mass_properties_changed(physics_engine, physics_spaces);
        self.wakeup(physics_engine);
    }

    fn apply_linear_damping(
        &mut self,
        new_value: real,
        apply_default: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &PhysicsSpaces,
    ) {
        if let Some(space) = physics_spaces.get(&self.base.get_space()) {
            self.total_linear_damping = new_value;
            if apply_default {
                let linear_damp: real = space
                    .get_default_area_param(AreaParameter::LINEAR_DAMP)
                    .to();
                self.total_linear_damping += linear_damp;
            }
            physics_engine.body_set_linear_damping(
                self.base.get_space_handle(),
                self.base.get_body_handle(),
                self.total_linear_damping,
            );
        }
    }

    fn apply_angular_damping(
        &mut self,
        new_value: real,
        apply_default: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &PhysicsSpaces,
    ) {
        if let Some(space) = physics_spaces.get(&self.base.get_space()) {
            self.total_angular_damping = new_value;
            if apply_default {
                let angular_damp: real = space
                    .get_default_area_param(AreaParameter::ANGULAR_DAMP)
                    .to();
                self.total_angular_damping += angular_damp;
            }
            physics_engine.body_set_angular_damping(
                self.base.get_space_handle(),
                self.base.get_body_handle(),
                self.total_angular_damping,
            );
        }
    }

    fn apply_gravity_scale(&self, new_value: real, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_gravity_scale(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            new_value,
            true,
        );
    }

    fn init_collider(
        &self,
        collider_handle: ColliderHandle,
        space_handle: WorldHandle,
        physics_engine: &mut PhysicsEngine,
    ) {
        // Send contact infos for dynamic bodies
        if self.base.mode.ord() >= BodyMode::KINEMATIC.ord() {
            let mut send_contacts = self.can_report_contacts();
            if godot::engine::Os::singleton().is_debug_build() {
                send_contacts = true;
            }
            physics_engine.collider_set_contact_force_events_enabled(
                space_handle,
                collider_handle,
                send_contacts,
            );
        }
    }

    pub fn to_add_static_constant_linear_velocity(&mut self, linear_velocity: Vector) {
        self.to_add_linear_velocity = linear_velocity;
    }

    pub fn to_add_static_constant_angular_velocity(&mut self, angular_velocity: Angle) {
        self.to_add_angular_velocity = angular_velocity;
    }

    pub fn set_linear_velocity(
        &mut self,
        p_linear_velocity: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.linear_velocity = p_linear_velocity;
        if self.base.mode == BodyMode::STATIC || !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_linear_velocity(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(self.linear_velocity),
        );
        self.linear_velocity = Vector::default();
    }

    pub fn get_linear_velocity(&self, physics_engine: &PhysicsEngine) -> Vector {
        if !self.base.is_valid() {
            return self.linear_velocity;
        }
        let vel = physics_engine
            .body_get_linear_velocity(self.base.get_space_handle(), self.base.get_body_handle());
        vector_to_godot(vel)
    }

    pub fn get_static_linear_velocity(&self) -> Vector {
        self.linear_velocity
    }

    pub fn set_angular_velocity(
        &mut self,
        p_angular_velocity: Angle,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.angular_velocity = p_angular_velocity;
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_angular_velocity(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(self.angular_velocity),
        );
        self.angular_velocity = ANGLE_ZERO;
    }

    pub fn get_angular_velocity(&self, physics_engine: &PhysicsEngine) -> Angle {
        if !self.base.is_valid() {
            return self.angular_velocity;
        }
        angle_to_godot(
            physics_engine.body_get_angular_velocity(
                self.base.get_space_handle(),
                self.base.get_body_handle(),
            ),
        )
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
        } else {
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

    pub fn add_area(&mut self, p_area: &RapierArea, space: &mut RapierSpace) {
        self.base.area_detection_counter += 1;
        if p_area.has_any_space_override() {
            // TODO sort
            let area_rid = p_area.get_base().get_rid();
            self.areas.push(area_rid);
            self.on_area_updated(area_rid, space);
        }
    }

    pub fn remove_area(&mut self, area: Rid, space: &mut RapierSpace) {
        if self.base.area_detection_counter == 0 {
            godot_error!("Area detection counter is zero.");
            return;
        }
        self.base.area_detection_counter -= 1;
        self.areas.retain(|&x| x != area);
        self.on_area_updated(area, space);
    }

    pub fn on_area_updated(&mut self, _area: Rid, space: &mut RapierSpace) {
        space.body_add_to_area_update_list(self.base.get_rid());
    }

    pub fn get_area_override_settings(
        &self,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> AreaOverrideSettings {
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
            space.body_remove_from_area_update_list(self.base.get_rid());
        }
        // Reset area override flags.
        let mut using_area_gravity = false;
        let mut using_area_linear_damping = false;
        let mut using_area_angular_damping = false;
        // Start with no effect.
        let mut total_gravity = Vector::default();
        let mut total_linear_damping = 0.0;
        let mut total_angular_damping = 0.0;
        // Combine gravity and damping from overlapping areas in priority order.
        let ac = self.areas.len();
        let mut gravity_done = false; // always calculate to be able to change scale on area gravity
        let mut linear_damping_done = self.linear_damping_mode == BodyDampMode::REPLACE;
        let mut angular_damping_done = self.angular_damping_mode == BodyDampMode::REPLACE;
        let origin = self.get_base().get_transform().origin;
        if ac > 0 {
            let mut areas = self.areas.clone();
            areas.reverse();
            for area_rid in areas {
                if let Some(area) = physics_collision_objects.get(&area_rid) {
                    if let Some(aa) = area.get_area() {
                        if !gravity_done {
                            let area_gravity_mode =
                                aa.get_param(AreaParameter::GRAVITY_OVERRIDE_MODE).to();
                            if area_gravity_mode != AreaSpaceOverrideMode::DISABLED {
                                let area_gravity = aa.compute_gravity(origin);
                                match area_gravity_mode {
                                    AreaSpaceOverrideMode::COMBINE
                                    | AreaSpaceOverrideMode::COMBINE_REPLACE => {
                                        using_area_gravity = true;
                                        total_gravity += area_gravity;
                                        gravity_done = area_gravity_mode
                                            == AreaSpaceOverrideMode::COMBINE_REPLACE;
                                    }
                                    AreaSpaceOverrideMode::REPLACE
                                    | AreaSpaceOverrideMode::REPLACE_COMBINE => {
                                        using_area_gravity = true;
                                        total_gravity = area_gravity;
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
                                        using_area_linear_damping = true;
                                        total_linear_damping += area_linear_damping;
                                        linear_damping_done = area_linear_damping_mode
                                            == AreaSpaceOverrideMode::COMBINE_REPLACE;
                                    }
                                    AreaSpaceOverrideMode::REPLACE
                                    | AreaSpaceOverrideMode::REPLACE_COMBINE => {
                                        using_area_linear_damping = true;
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
                                        using_area_angular_damping = true;
                                        total_angular_damping += area_angular_damping;
                                        angular_damping_done = area_angular_damping_mode
                                            == AreaSpaceOverrideMode::COMBINE_REPLACE;
                                    }
                                    AreaSpaceOverrideMode::REPLACE
                                    | AreaSpaceOverrideMode::REPLACE_COMBINE => {
                                        using_area_angular_damping = true;
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
        AreaOverrideSettings {
            using_area_gravity,
            using_area_linear_damping,
            using_area_angular_damping,
            total_gravity,
            total_linear_damping,
            total_angular_damping,
            gravity_done,
            linear_damping_done,
            angular_damping_done,
        }
    }

    pub fn apply_area_override(
        &mut self,
        area_override_settings: AreaOverrideSettings,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        let using_area_gravity = area_override_settings.using_area_gravity;
        let using_area_linear_damping = area_override_settings.using_area_linear_damping;
        let using_area_angular_damping = area_override_settings.using_area_angular_damping;
        let total_gravity = area_override_settings.total_gravity;
        let total_linear_damping = area_override_settings.total_linear_damping;
        let total_angular_damping = area_override_settings.total_angular_damping;
        let gravity_done = area_override_settings.gravity_done;
        let linear_damping_done = area_override_settings.linear_damping_done;
        let angular_damping_done = area_override_settings.angular_damping_done;
        self.using_area_gravity = using_area_gravity;
        self.using_area_linear_damping = using_area_linear_damping;
        self.using_area_angular_damping = using_area_angular_damping;
        self.total_gravity = total_gravity;
        self.total_linear_damping = total_linear_damping;
        self.total_angular_damping = total_angular_damping;
        // Apply to the simulation.
        self.apply_linear_damping(
            total_linear_damping,
            !linear_damping_done,
            physics_engine,
            physics_spaces,
        );
        self.apply_angular_damping(
            total_angular_damping,
            !angular_damping_done,
            physics_engine,
            physics_spaces,
        );
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
            if self.using_area_gravity {
                // Add default gravity from space.
                if !gravity_done {
                    self.total_gravity += space.get_default_area_param(AreaParameter::GRAVITY).to();
                }
                // Apply gravity scale to computed value.
                self.total_gravity *= self.gravity_scale;
                // Disable simulation gravity and apply it manually instead.
                self.apply_gravity_scale(0.0, physics_engine);
                space.body_add_to_gravity_update_list(self.base.get_rid());
            } else {
                // Enable simulation gravity.
                self.apply_gravity_scale(self.gravity_scale, physics_engine);
                space.body_remove_from_gravity_update_list(self.base.get_rid());
            }
        }
    }

    pub fn update_gravity(&mut self, p_step: real, physics_engine: &mut PhysicsEngine) {
        if !self.using_area_gravity || !self.base.is_valid() {
            return;
        }
        let gravity_impulse = self.total_gravity * self.mass * p_step;
        physics_engine.body_apply_impulse(
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

    pub fn force_mass_update(
        &mut self,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_engine: &mut PhysicsEngine,
    ) {
        if self.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
                space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            }
            self.update_mass_properties(true, physics_shapes, physics_engine);
        }
    }

    pub fn apply_central_impulse(&mut self, p_impulse: Vector, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            self.impulse += p_impulse;
            return;
        }
        physics_engine.body_apply_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_impulse),
        );
        self.impulse = Vector::default();
    }

    pub fn apply_impulse(
        &mut self,
        p_impulse: Vector,
        p_position: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        if !self.base.is_valid() {
            self.impulse += p_impulse;
            self.torque += (p_position - self.get_center_of_mass()).cross(p_impulse);
            return;
        }
        physics_engine.body_apply_impulse_at_point(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_impulse),
            vector_to_rapier(p_position),
        );
        self.impulse = Vector::default();
        self.torque = ANGLE_ZERO;
    }

    pub fn apply_torque_impulse(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            self.torque += p_torque;
            return;
        }
        physics_engine.body_apply_torque_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
        self.torque = ANGLE_ZERO;
    }

    pub fn apply_central_force(&mut self, p_force: Vector, physics_engine: &mut PhysicsEngine) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.impulse += p_force * last_delta;
            return;
        }
        physics_engine.body_apply_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force * last_delta),
        );
        self.impulse = Vector::default();
    }

    pub fn apply_force(
        &mut self,
        p_force: Vector,
        p_position: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.impulse += p_force * last_delta;
            self.torque += (p_position - self.get_center_of_mass()).cross(p_force) * last_delta;
            return;
        }
        physics_engine.body_apply_impulse_at_point(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force * last_delta),
            vector_to_rapier(p_position),
        );
        self.impulse = Vector::default();
        self.torque = ANGLE_ZERO;
    }

    pub fn apply_torque(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.torque += p_torque * last_delta;
            return;
        }
        physics_engine.body_apply_torque_impulse(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque * last_delta),
        );
        self.torque = ANGLE_ZERO;
    }

    pub fn add_constant_central_force(
        &mut self,
        p_force: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.constant_force += p_force;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_add_force(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
        );
    }

    pub fn add_constant_force(
        &mut self,
        p_force: Vector,
        p_position: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.constant_torque += (p_position - self.get_center_of_mass()).cross(p_force);
        self.constant_force += p_force;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_add_force_at_point(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
            vector_to_rapier(p_position),
        );
    }

    pub fn add_constant_torque(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        self.constant_torque += p_torque;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_add_torque(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
    }

    pub fn set_constant_force(&mut self, p_force: Vector, physics_engine: &mut PhysicsEngine) {
        self.constant_force = p_force;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_reset_forces(self.base.get_space_handle(), self.base.get_body_handle());
        physics_engine.body_add_force(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
        );
    }

    pub fn get_constant_force(&self, physics_engine: &PhysicsEngine) -> Vector {
        if !self.base.is_valid() {
            return self.constant_force;
        }
        let force = physics_engine
            .body_get_constant_force(self.base.get_space_handle(), self.base.get_body_handle());
        vector_to_godot(force)
    }

    pub fn set_constant_torque(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        self.constant_torque = p_torque;
        if !self.base.is_valid() {
            return;
        }
        physics_engine
            .body_reset_torques(self.base.get_space_handle(), self.base.get_body_handle());
        physics_engine.body_add_torque(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
    }

    pub fn get_constant_torque(&self, physics_engine: &PhysicsEngine) -> Angle {
        if !self.base.is_valid() {
            return self.constant_torque;
        }
        angle_to_godot(
            physics_engine.body_get_constant_torque(
                self.base.get_space_handle(),
                self.base.get_body_handle(),
            ),
        )
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
            } else {
                space.body_add_to_active_list(self.base.get_rid());
            }
        } else {
            space.body_remove_from_active_list(self.base.get_rid());
        }
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn set_can_sleep(&mut self, p_can_sleep: bool, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_can_sleep(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            p_can_sleep,
        );
    }

    pub fn on_marked_active(&mut self, space: &mut RapierSpace) {
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        self.marked_active = true;
        if !self.active {
            self.active = true;
            space.body_add_to_active_list(self.base.get_rid());
        }
    }

    pub fn on_update_active(
        &mut self,
        space: &mut RapierSpace,
        physics_engine: &mut PhysicsEngine,
    ) {
        if !self.marked_active {
            self.set_active(false, space);
            return;
        }
        self.marked_active = false;
        self.base.update_transform(physics_engine);
        space.body_add_to_state_query_list(self.base.get_rid());
        if self.base.mode.ord() >= BodyMode::RIGID.ord() {
            if self.to_add_angular_velocity != ANGLE_ZERO {
                self.set_angular_velocity(
                    self.get_angular_velocity(physics_engine) + self.to_add_angular_velocity,
                    physics_engine,
                );
                self.to_add_angular_velocity = ANGLE_ZERO;
            }
            if self.to_add_linear_velocity != Vector::default() {
                self.set_linear_velocity(
                    self.get_linear_velocity(physics_engine) + self.to_add_linear_velocity,
                    physics_engine,
                );
                self.to_add_linear_velocity = Vector::default();
            }
        }
    }

    pub fn wakeup(&mut self, physics_engine: &mut PhysicsEngine) {
        self.sleep = false;
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_wake_up(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            true,
        );
    }

    pub fn force_sleep(&mut self, physics_engine: &mut PhysicsEngine) {
        self.sleep = true;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_force_sleep(self.base.get_space_handle(), self.base.get_body_handle());
    }

    pub fn set_param(
        &mut self,
        p_param: BodyParameter,
        p_value: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        match p_param {
            BodyParameter::BOUNCE | BodyParameter::FRICTION => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                if p_param == BodyParameter::BOUNCE {
                    self.bounce = variant_to_float(&p_value);
                } else {
                    self.friction = variant_to_float(&p_value);
                }
                if !self.base.is_valid() {
                    return;
                }
                let mat = self.init_material();
                physics_engine.body_update_material(
                    self.base.get_space_handle(),
                    self.base.get_body_handle(),
                    &mat,
                );
            }
            BodyParameter::MASS => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                let mass_value = variant_to_float(&p_value);
                if mass_value <= 0.0 {
                    return;
                }
                self.mass = mass_value;
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces);
                }
            }
            #[cfg(feature = "dim2")]
            BodyParameter::INERTIA => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                let inertia_value = variant_to_float(&p_value);
                if inertia_value == ANGLE_ZERO {
                    self.calculate_inertia = true;
                } else {
                    self.calculate_inertia = false;
                    self.inertia = inertia_value;
                }
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces);
                }
            }
            #[cfg(feature = "dim3")]
            BodyParameter::INERTIA => {
                if p_value.get_type() != VariantType::VECTOR3
                {
                    return;
                }
                let inertia_value = p_value.to::<Vector3>();
                if inertia_value == ANGLE_ZERO {
                    self.calculate_inertia = true;
                } else {
                    self.calculate_inertia = false;
                    self.inertia = inertia_value;
                }
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces);
                }
            }
            BodyParameter::CENTER_OF_MASS => {
                if p_value.get_type() != VariantType::VECTOR2 {
                    return;
                }
                self.center_of_mass = p_value.to();
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces);
                }
            }
            BodyParameter::GRAVITY_SCALE => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                let new_gravity_scale = variant_to_float(&p_value);
                if self.gravity_scale != new_gravity_scale {
                    self.gravity_scale = new_gravity_scale;
                    if !self.using_area_gravity {
                        self.apply_gravity_scale(self.gravity_scale, physics_engine);
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
                    } else {
                        self.apply_linear_damping(
                            self.linear_damping,
                            true,
                            physics_engine,
                            physics_spaces,
                        );
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
                    } else {
                        self.apply_angular_damping(
                            self.angular_damping,
                            true,
                            physics_engine,
                            physics_spaces,
                        );
                    }
                }
            }
            BodyParameter::LINEAR_DAMP => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                let new_value = variant_to_float(&p_value);
                if new_value != self.linear_damping {
                    self.linear_damping = new_value;
                    if !self.using_area_linear_damping {
                        self.apply_linear_damping(
                            self.linear_damping,
                            true,
                            physics_engine,
                            physics_spaces,
                        );
                    }
                }
            }
            BodyParameter::ANGULAR_DAMP => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                let new_value = variant_to_float(&p_value);
                if new_value != self.angular_damping {
                    self.angular_damping = new_value;
                    if !self.using_area_angular_damping {
                        self.apply_angular_damping(
                            self.angular_damping,
                            true,
                            physics_engine,
                            physics_spaces,
                        );
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

    pub fn set_extra_param(
        &mut self,
        p_param: RapierBodyParam,
        p_value: Variant,
        physics_engine: &mut PhysicsEngine,
    ) {
        match p_param {
            RapierBodyParam::ContactSkin => {
                if p_value.get_type() != VariantType::FLOAT
                    && p_value.get_type() != VariantType::INT
                {
                    return;
                }
                self.contact_skin = variant_to_float(&p_value);
                let mat = self.init_material();
                let body_handle = self.base.get_body_handle();
                let space_handle = self.base.get_space_handle();
                if !self.base.is_valid() {
                    physics_engine.body_update_material(space_handle, body_handle, &mat);
                }
            }
        }
    }

    pub fn get_extra_param(&self, p_param: RapierBodyParam) -> Variant {
        match p_param {
            RapierBodyParam::ContactSkin => self.contact_skin.to_variant(),
        }
    }

    pub fn set_mode(
        &mut self,
        p_mode: BodyMode,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        if self.base.mode == p_mode {
            return;
        }
        let prev_mode = self.base.mode;
        self.base.mode = p_mode;
        let rid = self.base.get_rid();
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
            match p_mode {
                BodyMode::KINEMATIC => {
                    physics_engine.body_change_mode(
                        space.get_handle(),
                        self.base.get_body_handle(),
                        BodyType::Kinematic,
                        true,
                    );
                }
                BodyMode::STATIC => {
                    physics_engine.body_change_mode(
                        space.get_handle(),
                        self.base.get_body_handle(),
                        BodyType::Static,
                        true,
                    );
                }
                BodyMode::RIGID | BodyMode::RIGID_LINEAR => {
                    physics_engine.body_change_mode(
                        space.get_handle(),
                        self.base.get_body_handle(),
                        BodyType::Dynamic,
                        true,
                    );
                }
                _ => {}
            }
            if p_mode == BodyMode::STATIC {
                self.force_sleep(physics_engine);
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
            self.mass_properties_changed(physics_engine, physics_spaces);
        }
    }

    pub fn set_state(
        &mut self,
        p_state: BodyState,
        p_variant: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        match p_state {
            BodyState::TRANSFORM => {
                #[cfg(feature = "dim2")]
                if p_variant.get_type() != VariantType::TRANSFORM2D {
                    godot_error!("Invalid body data.");
                    return;
                }
                #[cfg(feature = "dim3")]
                if p_variant.get_type() != VariantType::TRANSFORM3D {
                    godot_error!("Invalid body data.");
                    return;
                }
                let transform = p_variant.to();
                self.base.set_transform(transform, true, physics_engine);
            }
            BodyState::LINEAR_VELOCITY => {
                if p_variant.get_type() != VariantType::VECTOR3 {
                    godot_error!("Invalid body data.");
                    return;
                }
                self.set_linear_velocity(p_variant.to(), physics_engine);
            }
            BodyState::ANGULAR_VELOCITY => {
                #[cfg(feature = "dim2")]
                if p_variant.get_type() != VariantType::FLOAT
                    || p_variant.get_type() != VariantType::INT
                {
                    godot_error!("Invalid body data.");
                } else {
                    self.set_angular_velocity(variant_to_float(&p_variant), physics_engine);
                }
                #[cfg(feature = "dim3")]
                if p_variant.get_type() != VariantType::VECTOR3 {
                    godot_error!("Invalid body data.");
                    return;
                } else {
                    self.set_angular_velocity(p_variant.to(), physics_engine);
                }
            }
            BodyState::SLEEPING => {
                if p_variant.get_type() != VariantType::BOOL {
                    godot_error!("Invalid body data.");
                    return;
                }
                if self.base.mode == BodyMode::STATIC {
                    return;
                }
                self.sleep = p_variant.to();
                if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
                    if self.sleep {
                        if self.can_sleep {
                            self.force_sleep(physics_engine);
                            self.set_active(false, space);
                        }
                    } else if self.base.mode != BodyMode::STATIC {
                        self.wakeup(physics_engine);
                        self.set_active(true, space);
                    }
                }
            }
            BodyState::CAN_SLEEP => {
                if p_variant.get_type() != VariantType::BOOL {
                    godot_error!("Invalid body data.");
                    return;
                }
                self.can_sleep = p_variant.to();
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.set_can_sleep(self.can_sleep, physics_engine);
                    if !self.active && !self.can_sleep {
                        self.wakeup(physics_engine);
                        if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
                            self.set_active(true, space);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    pub fn get_state(&self, p_state: BodyState, physics_engine: &PhysicsEngine) -> Variant {
        match p_state {
            BodyState::TRANSFORM => self.base.get_transform().to_variant(),
            BodyState::LINEAR_VELOCITY => self.get_linear_velocity(physics_engine).to_variant(),
            BodyState::ANGULAR_VELOCITY => self.get_angular_velocity(physics_engine).to_variant(),
            BodyState::SLEEPING => (!self.active).to_variant(),
            BodyState::CAN_SLEEP => self.can_sleep.to_variant(),
            _ => Variant::nil(),
        }
    }

    pub fn set_continuous_collision_detection_mode(
        &mut self,
        enabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.ccd_enabled = enabled;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_ccd_enabled(
            self.base.get_space_handle(),
            self.base.get_body_handle(),
            self.ccd_enabled,
        );
    }

    pub fn get_continuous_collision_detection_mode(&self) -> bool {
        self.ccd_enabled
    }

    pub fn update_mass_properties(
        &mut self,
        force_update: bool,
        physics_shapes: &mut PhysicsShapes,
        physics_engine: &mut PhysicsEngine,
    ) {
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
            if let Some(shape) = physics_shapes.get(&self.base.get_shape(i)) {
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
                    if let Some(shape) = physics_shapes.get(&self.base.get_shape(i)) {
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
                    if let Some(shape) = physics_shapes.get(&self.base.get_shape(i)) {
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
        self.apply_mass_properties(force_update, physics_engine);
    }

    #[cfg(feature = "dim2")]
    fn get_inertia_for_shape(
        &self,
        moment_of_inertia: Angle,
        shape_mass: Real,
        shape_transform: Transform,
        _i: usize,
    ) -> Real {
        let mtx = shape_transform;
        let _scale = transform_scale(&mtx);
        let shape_origin = mtx.origin - self.center_of_mass;
        moment_of_inertia + shape_mass * shape_origin.length_squared()
    }

    #[cfg(feature = "dim3")]
    fn get_inertia_for_shape(
        &self,
        moment_of_inertia: Angle,
        shape_mass: Real,
        shape_transform: Transform,
        _i: usize,
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

    pub fn reset_mass_properties(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        if self.calculate_inertia && self.calculate_center_of_mass {
            // Nothing to do, already calculated
            return;
        }
        self.calculate_inertia = true;
        self.calculate_center_of_mass = true;
        self.mass_properties_changed(physics_engine, physics_spaces);
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

    #[cfg(feature = "dim3")]
    pub fn get_inv_inertia_tensor(&self) -> Basis {
        self.inv_inertia_tensor
    }

    #[cfg(feature = "dim2")]
    pub fn get_velocity_at_local_point(
        &self,
        rel_pos: Vector,
        physics_engine: &PhysicsEngine,
    ) -> Vector {
        let linear_velocity = self.get_linear_velocity(physics_engine);
        let angular_velocity = self.get_angular_velocity(physics_engine);
        linear_velocity
            + Vector::new(
                -angular_velocity * (rel_pos.y - self.center_of_mass.y),
                angular_velocity * (rel_pos.x - self.center_of_mass.x),
            )
    }

    #[cfg(feature = "dim3")]
    pub fn get_velocity_at_local_point(&self, rel_pos: Vector,
        physics_engine: &PhysicsEngine,) -> Vector {
        let linear_velocity = self.get_linear_velocity(physics_engine);
        let angular_velocity = self.get_angular_velocity(physics_engine);
        linear_velocity + angular_velocity.cross(rel_pos - self.center_of_mass)
    }

    pub fn get_aabb(&self, physics_shapes: &PhysicsShapes) -> Rect {
        let mut shapes_found = false;
        let mut body_aabb = Rect::default();
        let shape_count = self.base.get_shape_count() as usize;
        for i in 0..shape_count {
            if self.base.is_shape_disabled(i) {
                continue;
            }
            if let Some(shape) = physics_shapes.get(&self.base.get_shape(i)) {
                if !shapes_found {
                    // TODO not 100% correct, we don't take into consideration rotation here.
                    body_aabb = shape
                        .get_base()
                        .get_aabb(self.base.get_shape_transform(i).origin);
                    shapes_found = true;
                } else {
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

    fn set_space_before(&mut self, physics_spaces: &mut PhysicsSpaces) {
        // remove body from previous space
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
            space.body_remove_from_mass_properties_update_list(self.base.get_rid());
            space.body_remove_from_gravity_update_list(self.base.get_rid());
            space.body_remove_from_active_list(self.base.get_rid());
            space.body_remove_from_state_query_list(self.base.get_rid());
            space.body_remove_from_area_update_list(self.base.get_rid());
        }
    }

    fn set_space_after(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        if self.base.is_space_valid() && self.base.mode.ord() >= BodyMode::KINEMATIC.ord() {
            if !self.can_sleep {
                self.set_can_sleep(false, physics_engine);
            }
            if self.active || !self.sleep {
                self.wakeup(physics_engine);
                if let Some(space) = physics_spaces.get_mut(&self.base.get_space()) {
                    space.body_add_to_active_list(self.base.get_rid());
                }
            } else if self.can_sleep && self.sleep {
                self.force_sleep(physics_engine);
            }
            if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                self.apply_gravity_scale(self.gravity_scale, physics_engine);
                if self.linear_damping_mode == BodyDampMode::COMBINE {
                    self.apply_linear_damping(
                        self.linear_damping,
                        true,
                        physics_engine,
                        physics_spaces,
                    );
                }
                if self.angular_damping_mode == BodyDampMode::COMBINE {
                    self.apply_angular_damping(
                        self.angular_damping,
                        true,
                        physics_engine,
                        physics_spaces,
                    );
                }
                self.mass_properties_changed(physics_engine, physics_spaces);
                if self.linear_velocity != Vector::default() {
                    self.set_linear_velocity(self.linear_velocity, physics_engine);
                }
                if self.angular_velocity != ANGLE_ZERO {
                    self.set_angular_velocity(self.angular_velocity, physics_engine);
                }
                if self.constant_force != Vector::default() {
                    self.set_constant_force(self.constant_force, physics_engine);
                }
                if self.constant_torque != ANGLE_ZERO {
                    self.set_constant_torque(self.constant_torque, physics_engine);
                }
                if self.impulse != Vector::default() {
                    self.apply_impulse(self.impulse, Vector::default(), physics_engine);
                }
                if self.torque != ANGLE_ZERO {
                    self.apply_torque_impulse(self.torque, physics_engine);
                }
                self.set_continuous_collision_detection_mode(self.ccd_enabled, physics_engine);
                physics_engine.body_update_material(
                    self.base.get_space_handle(),
                    self.base.get_body_handle(),
                    &self.init_material(),
                );
            }
        }
    }
}
// We won't use the pointers between threads, so it should be safe.
unsafe impl Sync for RapierBody {}
impl IRapierCollisionObject for RapierBody {
    fn get_base(&self) -> &RapierCollisionObject {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierCollisionObject {
        &mut self.base
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

    fn set_space(
        &mut self,
        space: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        if space == self.base.get_space() {
            return;
        }
        self.set_space_before(physics_spaces);
        self.base.set_space(space, physics_engine, physics_spaces);
        self.recreate_shapes(physics_engine, physics_shapes);
        self.set_space_after(physics_engine, physics_spaces);
    }

    fn add_shape(
        &mut self,
        p_shape: godot::prelude::Rid,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        RapierCollisionObject::add_shape(
            self,
            p_shape,
            p_transform,
            p_disabled,
            physics_engine,
            physics_spaces,
            physics_shapes,
        );
    }

    fn set_shape(
        &mut self,
        p_index: usize,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        RapierCollisionObject::set_shape(
            self,
            p_index,
            p_shape,
            physics_engine,
            physics_spaces,
            physics_shapes,
        );
    }

    fn set_shape_transform(
        &mut self,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        RapierCollisionObject::set_shape_transform(
            self,
            p_index,
            p_transform,
            physics_engine,
            physics_spaces,
            physics_shapes,
        );
    }

    fn set_shape_disabled(
        &mut self,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        RapierCollisionObject::set_shape_disabled(
            self,
            p_index,
            p_disabled,
            physics_engine,
            physics_spaces,
            physics_shapes,
        );
    }

    fn remove_shape_rid(
        &mut self,
        shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
    ) {
        // remove a shape, all the times it appears
        let mut i = 0;
        while i < self.base.shapes.len() {
            if self.base.shapes[i].shape == shape {
                self.remove_shape_idx(i, physics_engine, physics_spaces, physics_shapes);
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
    ) {
        RapierCollisionObject::remove_shape_idx(
            self,
            p_index,
            physics_engine,
            physics_spaces,
            physics_shapes,
        );
    }

    fn create_shape(
        &mut self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
    ) -> ColliderHandle {
        if !self.base.is_space_valid() {
            return ColliderHandle::invalid();
        }
        let mat = self.init_material();
        let handle =
            self.base
                .create_shape(shape, p_shape_index, mat, physics_engine, physics_shapes);
        self.init_collider(handle, self.base.get_space_handle(), physics_engine);
        handle
    }

    fn recreate_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
    ) {
        RapierCollisionObject::recreate_shapes(self, physics_engine, physics_shapes);
    }

    fn init_material(&self) -> Material {
        Material {
            friction: self.friction,
            restitution: self.bounce,
            contact_skin: self.contact_skin,
        }
    }

    fn shapes_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        self.mass_properties_changed(physics_engine, physics_spaces);
        self.wakeup(physics_engine);
    }

    fn shape_changed(
        &mut self,
        p_shape: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_shapes: &mut PhysicsShapes,
        physics_spaces: &mut PhysicsSpaces,
    ) {
        RapierCollisionObject::shape_changed(
            self,
            p_shape,
            physics_engine,
            physics_shapes,
            physics_spaces,
        );
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
