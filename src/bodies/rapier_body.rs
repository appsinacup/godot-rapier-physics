use bodies::rapier_collision_object_base::CollisionObjectShape;
use bodies::rapier_collision_object_base::CollisionObjectType;
use bodies::rapier_collision_object_base::RapierCollisionObjectBase;
use bodies::rapier_collision_object_base::RapierCollisionObjectBaseState;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use hashbrown::hash_set::HashSet;
#[cfg(feature = "dim3")]
use rapier::dynamics::LockedAxes;
use rapier::geometry::ColliderHandle;
#[cfg(feature = "dim3")]
use rapier::math::DEFAULT_EPSILON;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::PhysicsShapes;
use servers::rapier_physics_singleton::PhysicsSpaces;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;
use shapes::rapier_shape::IRapierShape;

use super::rapier_area::RapierArea;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::RapierBodyParam;
use crate::spaces::rapier_space::RapierSpace;
use crate::types::*;
use crate::*;
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Contact {
    pub local_pos: Vector,
    pub local_normal: Vector,
    pub depth: real,
    pub local_shape: i32,
    pub collider_pos: Vector,
    pub collider_shape: i32,
    pub collider_instance_id: u64,
    pub collider: RapierId,
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
            collider: RapierId::default(),
            local_velocity_at_pos: Vector::default(),
            collider_velocity_at_pos: Vector::default(),
            impulse: Vector::default(),
        }
    }
}
#[derive(Clone, Copy, Default, Debug)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct IdWithPriority {
    pub id: RapierId,
    pub priority: i32,
}
impl IdWithPriority {
    pub fn new(id: RapierId, priority: i32) -> Self {
        Self { id, priority }
    }
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Serialize))]
pub struct BodyExport<'a> {
    body_state: &'a RapierBodyState,
    base_state: &'a RapierCollisionObjectBaseState,
}
#[cfg_attr(feature = "serde-serialize", derive(serde::Deserialize))]
pub struct BodyImport {
    body_state: RapierBodyState,
    base_state: RapierCollisionObjectBaseState,
}
#[derive(Default, Debug)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierBodyState {
    pub(crate) total_gravity: Vector,
    pub(crate) total_linear_damping: real,
    pub(crate) total_angular_damping: real,
    pub(crate) mass: real,
    pub(crate) inv_mass: real,
    pub(crate) mass_properties_update_pending: bool,
    pub(crate) inertia: Angle,
    #[cfg(feature = "dim3")]
    pub(crate) principal_inertia_axes: Basis,
    pub(crate) inv_inertia: Angle,
    #[cfg(feature = "dim3")]
    pub(crate) inv_inertia_tensor: Basis,
    pub(crate) center_of_mass: Vector,
    pub(crate) marked_active: bool,
    pub(crate) constant_force: Vector,
    pub(crate) linear_velocity: Vector,
    pub(crate) previous_linear_velocity: Vector,
    pub(crate) active: bool,
    pub(crate) impulse: Vector,
    pub(crate) torque: Angle,
    pub(crate) angular_velocity: Angle,
    pub(crate) constant_torque: Angle,
    pub(crate) to_add_angular_velocity: Angle,
    pub(crate) to_add_linear_velocity: Vector,
    pub(crate) areas: Vec<IdWithPriority>,
    pub(crate) contacts: Vec<Contact>,
    pub(crate) contact_count: i32,
}
#[derive(Debug)]
pub struct RapierBody {
    linear_damping_mode: BodyDampMode,
    angular_damping_mode: BodyDampMode,
    linear_damping: real,
    angular_damping: real,
    gravity_scale: real,
    bounce: real,
    friction: real,
    #[cfg(feature = "dim3")]
    axis_lock: u8,
    contact_skin: real,
    calculate_inertia: bool,
    calculate_center_of_mass: bool,
    using_area_gravity: bool,
    using_area_linear_damping: bool,
    using_area_angular_damping: bool,
    exceptions: HashSet<Rid>,
    ccd_enabled: bool,
    omit_force_integration: bool,
    can_sleep: bool,
    sleep: bool,
    body_state_callback: Option<Callable>,
    force_integration_callback: Option<Callable>,
    direct_state: Option<Gd<PhysicsDirectBodyState>>,
    direct_state_array: VariantArray,
    force_integration_array: VariantArray,
    state: RapierBodyState,
    base: RapierCollisionObjectBase,
}
impl RapierBody {
    pub fn new(id: RapierId, rid: Rid) -> Self {
        let state = RapierBodyState {
            mass: 1.0,
            inv_mass: 1.0,
            ..Default::default()
        };
        Self {
            linear_damping_mode: BodyDampMode::COMBINE,
            angular_damping_mode: BodyDampMode::COMBINE,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            bounce: 0.0,
            friction: 1.0,
            #[cfg(feature = "dim3")]
            axis_lock: 0,
            contact_skin: 0.0,
            calculate_inertia: true,
            calculate_center_of_mass: true,
            using_area_gravity: false,
            using_area_linear_damping: false,
            using_area_angular_damping: false,
            exceptions: HashSet::default(),
            ccd_enabled: false,
            omit_force_integration: false,
            can_sleep: true,
            sleep: false,
            body_state_callback: None,
            force_integration_callback: None,
            direct_state: None,
            direct_state_array: VariantArray::new(),
            force_integration_array: VariantArray::new(),
            state,
            base: RapierCollisionObjectBase::new(id, rid, CollisionObjectType::Body),
        }
    }

    fn mass_properties_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if self.base.mode.ord() < BodyMode::RIGID.ord() {
            return;
        }
        if self.calculate_inertia || self.calculate_center_of_mass {
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
                space
                    .get_mut_state()
                    .body_add_to_mass_properties_update_list(self.base.get_id());
                self.state.mass_properties_update_pending = true;
            }
        } else {
            self.apply_mass_properties(false, physics_engine);
        }
    }

    fn apply_mass_properties(&mut self, force_update: bool, physics_engine: &mut PhysicsEngine) {
        if self.base.mode.ord() < BodyMode::RIGID.ord() || !self.base.is_valid() {
            return;
        }
        let mut inertia_value = self.state.inertia;
        if self.base.mode == BodyMode::RIGID_LINEAR {
            inertia_value = ANGLE_ZERO;
        }
        // Force update means local properties will be re-calculated internally,
        // it's needed for applying forces right away (otherwise it's updated on next step)
        physics_engine.body_set_mass_properties(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            self.state.mass,
            angle_to_rapier(inertia_value),
            vector_to_rapier(self.state.center_of_mass),
            false,
            force_update,
        );
        #[cfg(feature = "dim3")]
        self.apply_axis_lock(physics_engine);
    }

    fn apply_linear_damping(
        &mut self,
        new_value: real,
        apply_default: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if let Some(space) = physics_spaces.get(&self.base.get_space(physics_ids)) {
            self.state.total_linear_damping = new_value;
            if apply_default {
                let linear_damp =
                    variant_to_float(&space.get_default_area_param(AreaParameter::LINEAR_DAMP));
                self.state.total_linear_damping += linear_damp;
            }
            physics_engine.body_set_linear_damping(
                self.base.get_space_id(),
                self.base.get_body_handle(),
                self.state.total_linear_damping,
            );
        }
    }

    fn apply_angular_damping(
        &mut self,
        new_value: real,
        apply_default: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if let Some(space) = physics_spaces.get(&self.base.get_space(physics_ids)) {
            self.state.total_angular_damping = new_value;
            if apply_default {
                let angular_damp =
                    variant_to_float(&space.get_default_area_param(AreaParameter::ANGULAR_DAMP));
                self.state.total_angular_damping += angular_damp;
            }
            physics_engine.body_set_angular_damping(
                self.base.get_space_id(),
                self.base.get_body_handle(),
                self.state.total_angular_damping,
            );
        }
    }

    fn apply_gravity_scale(&self, new_value: real, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_gravity_scale(
            self.base.get_space_id(),
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
        // Send contact infos for dynamic bodies, or for static bodies with static linear velocity
        let is_with_static_linear_velocity = self.get_static_linear_velocity() != Vector::default();
        if self.base.mode.ord() >= BodyMode::KINEMATIC.ord() || is_with_static_linear_velocity {
            let mut send_contacts = self.can_report_contacts();
            if self.base.is_debugging_contacts && godot::classes::Os::singleton().is_debug_build() {
                send_contacts = true;
            }
            if is_with_static_linear_velocity {
                send_contacts = true;
            }
            physics_engine.collider_set_contact_force_events_enabled(
                space_handle,
                collider_handle,
                send_contacts,
            );
        }
        self.update_collider_filters(collider_handle, space_handle, physics_engine, false);
    }

    fn update_colliders_filters(&self, physics_engine: &mut PhysicsEngine) {
        let colliders = physics_engine
            .body_get_colliders(self.base.get_space_id(), self.base.get_body_handle())
            .to_vec();
        let mut override_modify_contacts = false;
        for shape in self.base.state.shapes.clone() {
            if shape.one_way_collision && !shape.disabled {
                override_modify_contacts = true;
                break;
            }
        }
        for collider in colliders {
            self.update_collider_filters(
                collider,
                self.base.get_space_id(),
                physics_engine,
                override_modify_contacts,
            );
        }
    }

    fn update_colliders_contact_events(&self, physics_engine: &mut PhysicsEngine) {
        let is_with_static_linear_velocity = self.get_static_linear_velocity() != Vector::default();
        if self.base.mode.ord() < BodyMode::KINEMATIC.ord() && !is_with_static_linear_velocity {
            return;
        }
        let colliders = physics_engine
            .body_get_colliders(self.base.get_space_id(), self.base.get_body_handle())
            .to_vec();
        let mut send_contacts = self.can_report_contacts();
        if self.base.is_debugging_contacts && godot::classes::Os::singleton().is_debug_build() {
            send_contacts = true;
        }
        if is_with_static_linear_velocity {
            send_contacts = true;
        }
        for collider in colliders {
            physics_engine.collider_set_contact_force_events_enabled(
                self.base.get_space_id(),
                collider,
                send_contacts,
            );
        }
    }

    fn update_collider_filters(
        &self,
        collider_handle: ColliderHandle,
        space_handle: WorldHandle,
        physics_engine: &mut PhysicsEngine,
        override_modify_contacts: bool,
    ) {
        // if it has any exception, it needs to filter for them
        let filter_contacts_enabled = !self.exceptions.is_empty();
        physics_engine.collider_set_filter_contacts_enabled(
            space_handle,
            collider_handle,
            filter_contacts_enabled,
        );
        // if we are a conveyer belt, we need to modify contacts
        // also if any shape is one-way
        let modify_contacts_enabled = self.base.mode == BodyMode::STATIC
            || self.base.mode == BodyMode::KINEMATIC
            || override_modify_contacts;
        physics_engine.collider_set_modify_contacts_enabled(
            space_handle,
            collider_handle,
            modify_contacts_enabled,
        );
    }

    #[allow(clippy::wrong_self_convention)]
    pub fn to_add_static_constant_linear_velocity(&mut self, linear_velocity: Vector) {
        self.state.to_add_linear_velocity = linear_velocity;
    }

    #[allow(clippy::wrong_self_convention)]
    pub fn to_add_static_constant_angular_velocity(&mut self, angular_velocity: Angle) {
        self.state.to_add_angular_velocity = angular_velocity;
    }

    pub fn set_linear_velocity(
        &mut self,
        p_linear_velocity: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.state.linear_velocity = p_linear_velocity;
        self.update_colliders_filters(physics_engine);
        if self.base.mode == BodyMode::STATIC || !self.base.is_valid() {
            self.update_colliders_contact_events(physics_engine);
            return;
        }
        physics_engine.body_set_linear_velocity(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(self.state.linear_velocity),
        );
        self.state.linear_velocity = Vector::default();
    }

    #[cfg(feature = "dim3")]
    pub fn set_axis_lock(
        &mut self,
        axis: BodyAxis,
        lock: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.axis_lock = if lock {
            self.axis_lock | (axis.ord() as u8)
        } else {
            self.axis_lock & (!axis.ord() as u8)
        };
        self.apply_axis_lock(physics_engine);
    }

    #[cfg(feature = "dim3")]
    fn apply_axis_lock(&mut self, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            return;
        }
        if let Some(axis_lock) = LockedAxes::from_bits(self.axis_lock) {
            physics_engine.body_set_axis_lock(
                self.base.get_space_id(),
                self.base.get_body_handle(),
                axis_lock,
            );
        } else {
            godot_error!("Invalid axis lock: {}", self.axis_lock);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn is_axis_locked(&self, axis: BodyAxis) -> bool {
        self.axis_lock & (axis.ord() as u8) != 0
    }

    pub fn get_linear_velocity(&self, physics_engine: &PhysicsEngine) -> Vector {
        if !self.base.is_valid() {
            return self.state.linear_velocity;
        }
        let vel = physics_engine
            .body_get_linear_velocity(self.base.get_space_id(), self.base.get_body_handle());
        vector_to_godot(vel)
    }

    pub fn get_static_linear_velocity(&self) -> Vector {
        if self.base.mode == BodyMode::STATIC {
            return self.state.linear_velocity;
        }
        Vector::default()
    }
    
    pub fn predict_next_frame_position(&self, timestep: f64, physics_engine: &mut PhysicsEngine) -> Vector {
        let vel = physics_engine
            .body_predict_next_frame_position(timestep, self.base.get_space_id(), self.base.get_body_handle());
        vector_to_godot(vel)
    }

    pub fn set_angular_velocity(
        &mut self,
        p_angular_velocity: Angle,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.state.angular_velocity = p_angular_velocity;
        self.update_colliders_filters(physics_engine);
        if self.base.mode == BodyMode::STATIC || !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_angular_velocity(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            angle_to_rapier(self.state.angular_velocity),
        );
        self.state.angular_velocity = ANGLE_ZERO;
    }

    pub fn get_angular_velocity(&self, physics_engine: &PhysicsEngine) -> Angle {
        if !self.base.is_valid() {
            return self.state.angular_velocity;
        }
        angle_to_godot(
            physics_engine
                .body_get_angular_velocity(self.base.get_space_id(), self.base.get_body_handle()),
        )
    }

    pub fn get_static_angular_velocity(&self) -> Angle {
        if self.base.mode == BodyMode::STATIC {
            return self.state.angular_velocity;
        }
        ANGLE_ZERO
    }

    pub fn set_state_sync_callback(
        &mut self,
        p_callable: Callable,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if !p_callable.is_valid() {
            self.body_state_callback = None;
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
                space
                    .get_mut_state()
                    .body_remove_from_state_query_list(self.base.get_id());
            }
        } else {
            self.body_state_callback = Some(p_callable);
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
                space
                    .get_mut_state()
                    .body_add_to_state_query_list(self.base.get_id());
            }
        }
    }

    pub fn get_state_sync_callback(&self) -> Option<&Callable> {
        self.body_state_callback.as_ref()
    }

    pub fn set_force_integration_callback(
        &mut self,
        callable: Callable,
        udata: Variant,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        self.force_integration_array.clear();
        self.force_integration_callback = None;
        if callable.is_valid() {
            self.force_integration_callback = Some(callable);
            if let Some(ds) = &self.direct_state {
                self.force_integration_array.push(&ds.to_variant());
                if !udata.is_nil() {
                    self.force_integration_array.push(&udata);
                }
            }
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
                space
                    .get_mut_state()
                    .body_add_to_force_integrate_list(self.base.get_id());
            }
        } else if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
            space
                .get_mut_state()
                .body_remove_from_force_integrate_list(self.base.get_id());
        }
    }

    pub fn get_force_integration_array(&self) -> &VariantArray {
        &self.force_integration_array
    }

    pub fn get_force_integration_callable(&self) -> Option<&Callable> {
        self.force_integration_callback.as_ref()
    }

    pub fn create_direct_state(&mut self) {
        if self.direct_state.is_none() {
            self.direct_state_array.clear();
            let mut direct_space_state = RapierDirectBodyState::new_alloc();
            {
                let mut direct_state = direct_space_state.bind_mut();
                direct_state.set_body(self.base.get_rid());
            }
            self.direct_state_array
                .push(&direct_space_state.clone().to_variant());
            self.direct_state = Some(direct_space_state.upcast());
        }
    }

    pub fn get_direct_state(&self) -> Option<&Gd<PhysicsDirectBodyState>> {
        self.direct_state.as_ref()
    }

    pub fn get_direct_state_array(&self) -> &VariantArray {
        &self.direct_state_array
    }

    pub fn add_area(&mut self, p_area: &RapierArea, space: &mut RapierSpace) {
        if p_area.has_any_space_override() {
            let area_id = p_area.get_base().get_id();
            let priority = p_area.get_priority();
            self.state
                .areas
                .push(IdWithPriority::new(area_id, priority));
            self.state.areas.sort_by(|a, b| a.priority.cmp(&b.priority));
            self.on_area_updated(space);
        }
    }

    pub fn remove_area(&mut self, area: RapierId, space: &mut RapierSpace) {
        if !self.base.is_space_valid() {
            return;
        }
        self.state.areas.retain(|&x| x.id != area);
        self.on_area_updated(space);
    }

    pub fn on_area_updated(&mut self, space: &mut RapierSpace) {
        space
            .get_mut_state()
            .body_add_to_area_update_list(self.base.get_id());
    }

    pub fn apply_area_override_to_body(
        body: &RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &mut PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) {
        let mut area_override_settings = None;
        if let Some(body) = physics_collision_objects.get(&get_id_rid(*body, physics_ids))
            && let Some(body) = body.get_body()
        {
            area_override_settings = Some(body.get_area_override_settings(
                physics_spaces,
                physics_collision_objects,
                physics_ids,
            ));
        }
        if let Some(area_override_settings) = area_override_settings
            && let Some(body) = physics_collision_objects.get_mut(&get_id_rid(*body, physics_ids))
            && let Some(body) = body.get_mut_body()
        {
            body.apply_area_override(
                area_override_settings,
                physics_engine,
                physics_spaces,
                physics_ids,
            );
        }
    }

    pub fn get_area_override_settings(
        &self,
        physics_spaces: &mut PhysicsSpaces,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> AreaOverrideSettings {
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
            space
                .get_mut_state()
                .body_remove_from_area_update_list(self.base.get_id());
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
        let ac = self.state.areas.len();
        let mut gravity_done = false; // always calculate to be able to change scale on area gravity
        let mut linear_damping_done = self.linear_damping_mode == BodyDampMode::REPLACE;
        let mut angular_damping_done = self.angular_damping_mode == BodyDampMode::REPLACE;
        let origin = self.get_base().get_transform().origin;
        // only compute if we don't omit force integration
        if ac > 0 {
            let mut areas = self.state.areas.clone();
            areas.reverse();
            for area_handle in areas.iter() {
                if let Some(area) =
                    physics_collision_objects.get(&get_id_rid(area_handle.id, physics_ids))
                    && let Some(aa) = area.get_area()
                {
                    if !gravity_done {
                        let area_gravity_mode = aa
                            .get_param(AreaParameter::GRAVITY_OVERRIDE_MODE)
                            .try_to()
                            .unwrap_or(AreaSpaceOverrideMode::DISABLED);
                        if area_gravity_mode != AreaSpaceOverrideMode::DISABLED {
                            let area_gravity = aa.compute_gravity(origin);
                            match area_gravity_mode {
                                AreaSpaceOverrideMode::COMBINE
                                | AreaSpaceOverrideMode::COMBINE_REPLACE => {
                                    using_area_gravity = true;
                                    total_gravity += area_gravity;
                                    gravity_done =
                                        area_gravity_mode == AreaSpaceOverrideMode::COMBINE_REPLACE;
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
                        let area_linear_damping_mode = aa
                            .get_param(AreaParameter::LINEAR_DAMP_OVERRIDE_MODE)
                            .try_to()
                            .unwrap_or(AreaSpaceOverrideMode::DISABLED);
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
                                    linear_damping_done =
                                        area_linear_damping_mode == AreaSpaceOverrideMode::REPLACE;
                                }
                                _ => {}
                            }
                        }
                    }
                    if !angular_damping_done {
                        let area_angular_damping_mode = aa
                            .get_param(AreaParameter::ANGULAR_DAMP_OVERRIDE_MODE)
                            .try_to()
                            .unwrap_or(AreaSpaceOverrideMode::DISABLED);
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
                                    angular_damping_done =
                                        area_angular_damping_mode == AreaSpaceOverrideMode::REPLACE;
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
        // Override or combine damping with body's values.
        if !self.omit_force_integration {
            total_linear_damping += self.linear_damping;
            total_angular_damping += self.angular_damping;
        } else {
            linear_damping_done = true;
            angular_damping_done = true;
        }
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
        physics_ids: &PhysicsIds,
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
        self.state.total_gravity = total_gravity;
        self.state.total_linear_damping = total_linear_damping;
        self.state.total_angular_damping = total_angular_damping;
        // Apply to the simulation.
        self.apply_linear_damping(
            total_linear_damping,
            !linear_damping_done,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
        self.apply_angular_damping(
            total_angular_damping,
            !angular_damping_done,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
            // Add default gravity from space.
            if !gravity_done {
                let default_gravity =
                    variant_to_float(&space.get_default_area_param(AreaParameter::GRAVITY));
                let default_gravity_vector: Vector = space
                    .get_default_area_param(AreaParameter::GRAVITY_VECTOR)
                    .try_to()
                    .unwrap_or_default();
                self.state.total_gravity += default_gravity_vector * default_gravity;
            }
            // Apply gravity scale to computed value.
            self.state.total_gravity *= self.gravity_scale;
        }
        if self.omit_force_integration || self.using_area_gravity {
            self.apply_gravity_scale(0.0, physics_engine);
        } else {
            // Enable simulation gravity.
            self.apply_gravity_scale(self.gravity_scale, physics_engine);
        }
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
            if self.using_area_gravity && !self.omit_force_integration {
                // Disable simulation gravity and apply it manually instead.
                space
                    .get_mut_state()
                    .body_add_to_gravity_update_list(self.base.get_id());
            } else {
                space
                    .get_mut_state()
                    .body_remove_from_gravity_update_list(self.base.get_id());
            }
        }
    }

    pub fn update_gravity(&mut self, p_step: real, physics_engine: &mut PhysicsEngine) {
        if !self.using_area_gravity || !self.base.is_valid() {
            return;
        }
        let gravity_impulse = self.state.total_gravity * self.state.mass * p_step;
        physics_engine.body_apply_impulse(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(gravity_impulse),
        );
    }

    pub fn set_max_contacts_reported(
        &mut self,
        size: i32,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        self.state
            .contacts
            .resize(size as usize, Contact::default());
        self.state.contact_count = 0;
        // update all contact forces
        self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
    }

    pub fn reset_contact_count(&mut self) {
        self.state.contact_count = 0;
    }

    pub fn get_max_contacts_reported(&self) -> i32 {
        self.state.contacts.len() as i32
    }

    pub fn can_report_contacts(&self) -> bool {
        !self.state.contacts.is_empty()
    }

    #[allow(clippy::too_many_arguments)]
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
        collider: RapierId,
        collider_velocity_at_pos: Vector,
        impulse: Vector,
    ) {
        let c_max = self.state.contacts.len();
        if c_max == 0 {
            return;
        }
        let mut idx = -1;
        if self.state.contact_count < c_max as i32 {
            idx = self.state.contact_count;
            self.state.contact_count += 1;
        } else {
            let mut least_depth = f32::INFINITY;
            let mut least_deep: i32 = -1;
            for (i, contact) in self.state.contacts.iter().enumerate() {
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
        let c = &mut self.state.contacts[idx as usize];
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

    pub fn add_exception(&mut self, exception: Rid, physics_engine: &mut PhysicsEngine) {
        self.exceptions.insert(exception);
        self.update_colliders_filters(physics_engine);
    }

    pub fn remove_exception(&mut self, exception: Rid, physics_engine: &mut PhysicsEngine) {
        self.exceptions.remove(&exception);
        self.update_colliders_filters(physics_engine);
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
        physics_engine: &mut PhysicsEngine,
        physics_ids: &PhysicsIds,
    ) {
        if self.state.mass_properties_update_pending {
            // Force update internal mass properties to calculate proper impulse
            if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
                space
                    .get_mut_state()
                    .body_remove_from_mass_properties_update_list(self.base.get_id());
            }
            self.update_mass_properties(true, physics_engine);
        }
    }

    pub fn apply_central_impulse(&mut self, p_impulse: Vector, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            self.state.impulse += p_impulse;
            return;
        }
        physics_engine.body_apply_impulse(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(p_impulse),
        );
        self.state.impulse = Vector::default();
    }

    pub fn apply_impulse(
        &mut self,
        p_impulse: Vector,
        p_position: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        if !self.base.is_valid() {
            self.state.impulse += p_impulse;
            self.state.torque += (p_position - self.get_center_of_mass()).cross(p_impulse);
            return;
        }
        physics_engine.body_apply_impulse_at_point(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(p_impulse),
            vector_to_rapier(p_position),
        );
        self.state.impulse = Vector::default();
        self.state.torque = ANGLE_ZERO;
    }

    pub fn apply_torque_impulse(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            self.state.torque += p_torque;
            return;
        }
        physics_engine.body_apply_torque_impulse(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
        self.state.torque = ANGLE_ZERO;
    }

    pub fn apply_central_force(&mut self, p_force: Vector, physics_engine: &mut PhysicsEngine) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.state.impulse += p_force * last_delta;
            return;
        }
        physics_engine.body_apply_impulse(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force * last_delta),
        );
        self.state.impulse = Vector::default();
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
            self.state.impulse += p_force * last_delta;
            self.state.torque +=
                (p_position - self.get_center_of_mass()).cross(p_force) * last_delta;
            return;
        }
        physics_engine.body_apply_impulse_at_point(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force * last_delta),
            vector_to_rapier(p_position),
        );
        self.state.impulse = Vector::default();
        self.state.torque = ANGLE_ZERO;
    }

    pub fn apply_torque(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        // Note: using last delta assuming constant physics time
        let last_delta = RapierSpace::get_last_step();
        if !self.base.is_valid() {
            self.state.torque += p_torque * last_delta;
            return;
        }
        physics_engine.body_apply_torque_impulse(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque * last_delta),
        );
        self.state.torque = ANGLE_ZERO;
    }

    pub fn add_constant_central_force(
        &mut self,
        p_force: Vector,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.state.constant_force += p_force;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_add_force(
            self.base.get_space_id(),
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
        self.state.constant_torque += (p_position - self.get_center_of_mass()).cross(p_force);
        self.state.constant_force += p_force;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_add_force_at_point(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
            vector_to_rapier(p_position),
        );
    }

    pub fn add_constant_torque(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        self.state.constant_torque += p_torque;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_add_torque(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
    }

    pub fn set_constant_force(&mut self, p_force: Vector, physics_engine: &mut PhysicsEngine) {
        self.state.constant_force = p_force;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_reset_forces(self.base.get_space_id(), self.base.get_body_handle());
        physics_engine.body_add_force(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            vector_to_rapier(p_force),
        );
    }

    pub fn get_constant_force(&self, physics_engine: &PhysicsEngine) -> Vector {
        if !self.base.is_valid() {
            return self.state.constant_force;
        }
        let force = physics_engine
            .body_get_constant_force(self.base.get_space_id(), self.base.get_body_handle());
        vector_to_godot(force)
    }

    pub fn set_constant_torque(&mut self, p_torque: Angle, physics_engine: &mut PhysicsEngine) {
        self.state.constant_torque = p_torque;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_reset_torques(self.base.get_space_id(), self.base.get_body_handle());
        physics_engine.body_add_torque(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            angle_to_rapier(p_torque),
        );
    }

    pub fn get_constant_torque(&self, physics_engine: &PhysicsEngine) -> Angle {
        if !self.base.is_valid() {
            return self.state.constant_torque;
        }
        angle_to_godot(
            physics_engine
                .body_get_constant_torque(self.base.get_space_id(), self.base.get_body_handle()),
        )
    }

    pub fn set_active(&mut self, p_active: bool, space: &mut RapierSpace) {
        if self.state.active == p_active {
            return;
        }
        self.state.active = p_active;
        if self.state.active {
            if self.base.mode == BodyMode::STATIC {
                // Static bodies can't be active.
                self.state.active = false;
            } else {
                space
                    .get_mut_state()
                    .body_add_to_active_list(self.base.get_id());
            }
        } else {
            space
                .get_mut_state()
                .body_remove_from_active_list(self.base.get_id());
        }
    }

    pub fn is_active(&self) -> bool {
        self.state.active
    }

    pub fn set_can_sleep(&mut self, p_can_sleep: bool, physics_engine: &mut PhysicsEngine) {
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_set_can_sleep(
            self.base.get_space_id(),
            self.base.get_body_handle(),
            p_can_sleep,
            self.base.activation_angular_threshold,
            self.base.activation_linear_threshold,
            self.base.activation_time_until_sleep,
        );
    }

    pub fn on_marked_active(&mut self, space: &mut RapierSpace) {
        if self.base.mode == BodyMode::STATIC {
            return;
        }
        self.state.marked_active = true;
        if !self.state.active {
            self.state.active = true;
            space
                .get_mut_state()
                .body_add_to_active_list(self.base.get_id());
        }
    }

    pub fn set_previous_linear_velocity(&mut self, p_velocity: Vector) {
        self.state.previous_linear_velocity = p_velocity;
    }

    pub fn get_previous_linear_velocity(&self) -> Vector {
        self.state.previous_linear_velocity
    }

    pub fn on_update_active(
        &mut self,
        space: &mut RapierSpace,
        physics_engine: &mut PhysicsEngine,
    ) {
        if !self.state.marked_active {
            self.set_active(false, space);
            return;
        }
        self.state.marked_active = false;
        self.base.update_transform(physics_engine);
        if self.base.mode.ord() >= BodyMode::RIGID.ord() {
            if self.state.to_add_angular_velocity != ANGLE_ZERO {
                self.set_angular_velocity(self.state.to_add_angular_velocity, physics_engine);
                self.state.to_add_angular_velocity = ANGLE_ZERO;
            }
            if self.state.to_add_linear_velocity != Vector::default() {
                self.set_linear_velocity(self.state.to_add_linear_velocity, physics_engine);
                self.state.to_add_linear_velocity = Vector::default();
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
        physics_engine.body_wake_up(self.base.get_space_id(), self.base.get_body_handle(), true);
    }

    pub fn force_sleep(&mut self, physics_engine: &mut PhysicsEngine) {
        self.sleep = true;
        if !self.base.is_valid() {
            return;
        }
        physics_engine.body_force_sleep(self.base.get_space_id(), self.base.get_body_handle());
    }

    pub fn set_param(
        &mut self,
        p_param: BodyParameter,
        p_value: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
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
                    self.base.get_space_id(),
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
                self.state.mass = mass_value;
                if self.state.mass.is_zero_approx() {
                    self.state.inv_mass = 0.0;
                } else {
                    self.state.inv_mass = 1.0 / self.state.mass;
                }
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
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
                    self.state.inertia = inertia_value;
                }
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
                }
            }
            #[cfg(feature = "dim3")]
            BodyParameter::INERTIA => {
                if p_value.get_type() != VariantType::VECTOR3 {
                    return;
                }
                let inertia_value = p_value.to::<Vector3>();
                if inertia_value == ANGLE_ZERO {
                    self.calculate_inertia = true;
                } else {
                    self.calculate_inertia = false;
                    self.state.inertia = inertia_value;
                }
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
                }
            }
            BodyParameter::CENTER_OF_MASS => {
                #[cfg(feature = "dim2")]
                if p_value.get_type() != VariantType::VECTOR2 {
                    godot_error!("Invalid body data.");
                    return;
                }
                #[cfg(feature = "dim3")]
                if p_value.get_type() != VariantType::VECTOR3 {
                    godot_error!("Invalid body data.");
                    return;
                }
                self.state.center_of_mass = p_value.try_to().unwrap_or_default();
                self.calculate_center_of_mass = false;
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
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
                let mode_value = p_value.try_to().unwrap_or_default();
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
                            self.linear_damping_mode == BodyDampMode::COMBINE,
                            physics_engine,
                            physics_spaces,
                            physics_ids,
                        );
                    }
                }
            }
            BodyParameter::ANGULAR_DAMP_MODE => {
                if p_value.get_type() != VariantType::INT {
                    return;
                }
                let mode_value = p_value.try_to().unwrap_or_default();
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
                            self.angular_damping_mode == BodyDampMode::COMBINE,
                            physics_engine,
                            physics_spaces,
                            physics_ids,
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
                            physics_ids,
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
                            physics_ids,
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
            BodyParameter::MASS => self.state.mass.to_variant(),
            BodyParameter::INERTIA => self.state.inertia.to_variant(),
            BodyParameter::CENTER_OF_MASS => self.state.center_of_mass.to_variant(),
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
                let space_handle = self.base.get_space_id();
                if self.base.is_valid() {
                    physics_engine.body_update_material(space_handle, body_handle, &mat);
                }
            }
            RapierBodyParam::Dominance => {
                if p_value.get_type() != VariantType::INT {
                    return;
                }
                self.base
                    .set_dominance(variant_to_int(&p_value) as i8, physics_engine);
                let mat = self.init_material();
                let body_handle = self.base.get_body_handle();
                let space_handle = self.base.get_space_id();
                if self.base.is_valid() {
                    physics_engine.body_update_material(space_handle, body_handle, &mat);
                }
            }
        }
    }

    pub fn get_extra_param(&self, p_param: RapierBodyParam) -> Variant {
        match p_param {
            RapierBodyParam::ContactSkin => self.contact_skin.to_variant(),
            RapierBodyParam::Dominance => self.base.get_dominance().to_variant(),
        }
    }

    pub fn set_mode(
        &mut self,
        p_mode: BodyMode,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if self.base.mode == p_mode {
            return;
        }
        let prev_mode = self.base.mode;
        self.base.mode = p_mode;
        let id = self.base.get_id();
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
            match p_mode {
                BodyMode::KINEMATIC => {
                    physics_engine.body_change_mode(
                        space.get_state().get_id(),
                        self.base.get_body_handle(),
                        BodyType::Kinematic,
                        true,
                    );
                }
                BodyMode::STATIC => {
                    physics_engine.body_change_mode(
                        space.get_state().get_id(),
                        self.base.get_body_handle(),
                        BodyType::Static,
                        true,
                    );
                }
                BodyMode::RIGID | BodyMode::RIGID_LINEAR => {
                    physics_engine.body_change_mode(
                        space.get_state().get_id(),
                        self.base.get_body_handle(),
                        BodyType::Dynamic,
                        true,
                    );
                }
                _ => {}
            }
            if p_mode == BodyMode::STATIC {
                self.force_sleep(physics_engine);
                if self.state.marked_active {
                    return;
                }
                space.get_mut_state().body_remove_from_active_list(id);
                space
                    .get_mut_state()
                    .body_remove_from_mass_properties_update_list(id);
                space
                    .get_mut_state()
                    .body_remove_from_gravity_update_list(id);
                space.get_mut_state().body_remove_from_area_update_list(id);
                return;
            }
            if self.state.active && prev_mode == BodyMode::STATIC {
                space.get_mut_state().body_add_to_active_list(id);
            }
        }
        self.update_colliders_filters(physics_engine);
        self.update_colliders_contact_events(physics_engine);
        if p_mode.ord() >= BodyMode::RIGID.ord() {
            self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
        }
        self.set_space_after(physics_engine, physics_spaces, physics_ids);
    }

    pub fn set_state(
        &mut self,
        p_state: BodyState,
        p_variant: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
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
                let old_scale = transform_scale(&self.base.get_transform());
                let transform = p_variant.try_to().unwrap_or_default();
                let new_scale = transform_scale(&transform);
                self.base.set_transform(transform, true, physics_engine);
                if old_scale != new_scale {
                    self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
                }
                // set_transform updates mass properties
                self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
            }
            BodyState::LINEAR_VELOCITY => {
                #[cfg(feature = "dim2")]
                if p_variant.get_type() != VariantType::VECTOR2 {
                    godot_error!("Invalid body data.");
                    return;
                }
                #[cfg(feature = "dim3")]
                if p_variant.get_type() != VariantType::VECTOR3 {
                    godot_error!("Invalid body data.");
                    return;
                }
                self.set_linear_velocity(p_variant.try_to().unwrap_or_default(), physics_engine);
            }
            BodyState::ANGULAR_VELOCITY => {
                #[cfg(feature = "dim2")]
                if p_variant.get_type() != VariantType::FLOAT
                    && p_variant.get_type() != VariantType::INT
                {
                    godot_error!("Invalid body data.");
                } else {
                    self.set_angular_velocity(variant_to_float(&p_variant), physics_engine);
                }
                #[cfg(feature = "dim3")]
                if p_variant.get_type() != VariantType::VECTOR3 {
                    godot_error!("Invalid body data.");
                } else {
                    self.set_angular_velocity(
                        p_variant.try_to().unwrap_or_default(),
                        physics_engine,
                    );
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
                self.sleep = p_variant.try_to().unwrap_or_default();
                if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
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
                self.can_sleep = p_variant.try_to().unwrap_or_default();
                if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                    self.set_can_sleep(self.can_sleep, physics_engine);
                    if !self.state.active && !self.can_sleep {
                        self.wakeup(physics_engine);
                        if let Some(space) =
                            physics_spaces.get_mut(&self.base.get_space(physics_ids))
                        {
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
            BodyState::SLEEPING => (!self.state.active).to_variant(),
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
            self.base.get_space_id(),
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
        physics_engine: &mut PhysicsEngine,
    ) {
        self.state.mass_properties_update_pending = false;
        if self.base.mode.ord() < BodyMode::RIGID.ord() {
            return;
        }
        // compute rigidbody mass properties by changing collider mass. Will get overriden later
        let rigid_body_mass_properties = physics_engine
            .body_get_mass_properties(self.base.get_space_id(), self.base.get_body_handle());
        if self.calculate_center_of_mass {
            self.state.center_of_mass =
                vector_to_godot(rigid_body_mass_properties.0.local_mprops.local_com.coords);
        }
        if self.calculate_inertia {
            let angular_inertia = rigid_body_mass_properties
                .0
                .local_mprops
                .principal_inertia();
            self.state.inertia = angle_to_godot(angular_inertia) * self.state.mass
                / (rigid_body_mass_properties.1 as real);
        }
        if self.state.inertia.is_zero_approx() {
            self.state.inv_inertia = ANGLE_ZERO;
        }
        #[cfg(feature = "dim2")]
        if !self.state.inertia.is_zero_approx() {
            self.state.inv_inertia = 1.0 / self.state.inertia;
        }
        #[cfg(feature = "dim3")]
        if !self.state.inertia.is_zero_approx() {
            // inv inertia
            if !self.state.inv_inertia.x.is_zero_approx() {
                self.state.inv_inertia.x = 1.0 / self.state.inertia.x;
            } else {
                self.state.inv_inertia.x = 0.0;
            }
            if !self.state.inv_inertia.y.is_zero_approx() {
                self.state.inv_inertia.y = 1.0 / self.state.inertia.y;
            } else {
                self.state.inv_inertia.y = 0.0;
            }
            if !self.state.inv_inertia.z.is_zero_approx() {
                self.state.inv_inertia.z = 1.0 / self.state.inertia.z;
            } else {
                self.state.inv_inertia.z = 0.0;
            }
            // inv inertia tensor
            let rotation_matrix = rigid_body_mass_properties
                .0
                .local_mprops
                .principal_inertia_local_frame
                .to_rotation_matrix();
            let vector = rotation_matrix.matrix();
            let column_0 = vector
                .column(0)
                .pseudo_inverse(DEFAULT_EPSILON)
                .unwrap_or_default();
            let column_1 = vector
                .column(1)
                .pseudo_inverse(DEFAULT_EPSILON)
                .unwrap_or_default();
            let column_2 = vector
                .column(2)
                .pseudo_inverse(DEFAULT_EPSILON)
                .unwrap_or_default();
            self.state.principal_inertia_axes = Basis::from_cols(
                Vector3::new(column_0.x, column_0.y, column_0.z),
                Vector3::new(column_1.x, column_1.y, column_1.z),
                Vector3::new(column_2.x, column_2.y, column_2.z),
            );
            let inv_inertia = rigid_body_mass_properties
                .0
                .local_mprops
                .inv_principal_inertia;
            let tb = self.state.principal_inertia_axes;
            let tbt = tb.transposed();
            let diag =
                Basis::IDENTITY.scaled(Vector3::new(inv_inertia.x, inv_inertia.y, inv_inertia.z));
            self.state.inv_inertia_tensor = tb * diag * tbt;
        }
        self.apply_mass_properties(force_update, physics_engine);
    }

    pub fn reset_mass_properties(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if self.calculate_inertia && self.calculate_center_of_mass {
            // Nothing to do, already calculated
            return;
        }
        self.calculate_inertia = true;
        self.calculate_center_of_mass = true;
        self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
    }

    pub fn get_center_of_mass(&self) -> Vector {
        self.state.center_of_mass
    }

    pub fn get_inv_mass(&self) -> real {
        self.state.inv_mass
    }

    pub fn get_inv_inertia(&self) -> Angle {
        self.state.inv_inertia
    }

    #[cfg(feature = "dim3")]
    pub fn get_inv_inertia_tensor(&self) -> Basis {
        self.state.inv_inertia_tensor
    }

    #[cfg(feature = "dim3")]
    pub fn get_principal_inertia_axes(&self) -> Basis {
        self.state.principal_inertia_axes
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
                -angular_velocity * (rel_pos.y - self.state.center_of_mass.y),
                angular_velocity * (rel_pos.x - self.state.center_of_mass.x),
            )
    }

    #[cfg(feature = "dim3")]
    pub fn get_velocity_at_local_point(
        &self,
        rel_pos: Vector,
        physics_engine: &PhysicsEngine,
    ) -> Vector {
        let linear_velocity = self.get_linear_velocity(physics_engine);
        let angular_velocity = self.get_angular_velocity(physics_engine);
        linear_velocity + angular_velocity.cross(rel_pos - self.state.center_of_mass)
    }

    pub fn get_aabb(&self, physics_shapes: &PhysicsShapes, physics_ids: &PhysicsIds) -> Rect {
        let mut shapes_found = false;
        let mut body_aabb = Rect::default();
        let shape_count = self.base.get_shape_count() as usize;
        for i in 0..shape_count {
            if self.base.is_shape_disabled(i) {
                continue;
            }
            if let Some(shape) = physics_shapes.get(&self.base.get_shape(physics_ids, i)) {
                let mut shape_transform = self.base.get_shape_transform(i);
                if !shapes_found {
                    body_aabb = shape.get_base().get_aabb(shape_transform.origin);
                    shape_transform.origin = Vector::default();
                    body_aabb.size = transform_scale(&shape_transform) * body_aabb.size;
                    shapes_found = true;
                } else {
                    let mut shape_aabb = shape.get_base().get_aabb(shape_transform.origin);
                    shape_transform.origin = Vector::default();
                    shape_aabb.size = transform_scale(&shape_transform) * shape_aabb.size;
                    body_aabb = body_aabb.merge(shape_aabb);
                }
            }
        }
        // HACK Rotation fix hack, do different
        body_aabb.size.x = body_aabb.size.x.max(body_aabb.size.y);
        body_aabb.size.y = body_aabb.size.x;
        #[cfg(feature = "dim3")]
        if crate::rapier::prelude::DIM == 3 {
            body_aabb.size.z = body_aabb.size.x;
        }
        body_aabb
    }

    pub fn total_linear_damping(&self) -> real {
        self.state.total_linear_damping
    }

    pub fn total_angular_damping(&self) -> real {
        self.state.total_angular_damping
    }

    pub fn total_gravity(&self) -> Vector {
        self.state.total_gravity
    }

    pub fn gravity_scale(&self) -> real {
        self.gravity_scale
    }

    pub fn using_area_gravity(&self) -> bool {
        self.using_area_gravity
    }

    pub fn contact_count(&self) -> i32 {
        self.state.contact_count
    }

    pub fn contacts(&self) -> Vec<&Contact> {
        self.state
            .contacts
            .iter()
            .take(self.state.contact_count as usize)
            .collect()
    }

    fn set_space_before(&mut self, physics_spaces: &mut PhysicsSpaces, physics_ids: &PhysicsIds) {
        // remove body from previous space
        if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
            let id = self.base.get_id();
            space
                .get_mut_state()
                .body_remove_from_mass_properties_update_list(id);
            space
                .get_mut_state()
                .body_remove_from_gravity_update_list(id);
            space.get_mut_state().body_remove_from_active_list(id);
            space.get_mut_state().body_remove_from_state_query_list(id);
            space.get_mut_state().body_remove_from_area_update_list(id);
            space
                .get_mut_state()
                .body_remove_from_force_integrate_list(id);
        }
    }

    fn set_space_after(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        let id = self.base.get_id();
        if self.base.is_space_valid() && self.base.mode.ord() >= BodyMode::KINEMATIC.ord() {
            if self.get_force_integration_callable().is_some()
                && let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids))
            {
                space.get_mut_state().body_add_to_force_integrate_list(id);
            }
            if self.get_state_sync_callback().is_some()
                && let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids))
            {
                space.get_mut_state().body_add_to_state_query_list(id);
            }
            if !self.can_sleep {
                self.set_can_sleep(false, physics_engine);
            }
            if self.state.active || !self.sleep {
                self.wakeup(physics_engine);
                if let Some(space) = physics_spaces.get_mut(&self.base.get_space(physics_ids)) {
                    space.get_mut_state().body_add_to_active_list(id);
                }
            } else if self.can_sleep && self.sleep {
                self.force_sleep(physics_engine);
            }
            if self.base.mode.ord() >= BodyMode::RIGID.ord() {
                if self.omit_force_integration {
                    self.apply_gravity_scale(0.0, physics_engine);
                    self.apply_linear_damping(
                        0.0,
                        false,
                        physics_engine,
                        physics_spaces,
                        physics_ids,
                    );
                    self.apply_angular_damping(
                        0.0,
                        false,
                        physics_engine,
                        physics_spaces,
                        physics_ids,
                    );
                } else {
                    self.apply_gravity_scale(self.gravity_scale, physics_engine);
                    self.apply_linear_damping(
                        self.linear_damping,
                        self.linear_damping_mode == BodyDampMode::COMBINE,
                        physics_engine,
                        physics_spaces,
                        physics_ids,
                    );
                    self.apply_angular_damping(
                        self.angular_damping,
                        self.angular_damping_mode == BodyDampMode::COMBINE,
                        physics_engine,
                        physics_spaces,
                        physics_ids,
                    );
                }
                self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
                if self.state.linear_velocity != Vector::default() {
                    self.set_linear_velocity(self.state.linear_velocity, physics_engine);
                }
                if self.state.angular_velocity != ANGLE_ZERO {
                    self.set_angular_velocity(self.state.angular_velocity, physics_engine);
                }
                if self.state.constant_force != Vector::default() {
                    self.set_constant_force(self.state.constant_force, physics_engine);
                }
                if self.state.constant_torque != ANGLE_ZERO {
                    self.set_constant_torque(self.state.constant_torque, physics_engine);
                }
                if self.state.impulse != Vector::default() {
                    self.apply_central_impulse(self.state.impulse, physics_engine);
                }
                if self.state.torque != ANGLE_ZERO {
                    self.apply_torque_impulse(self.state.torque, physics_engine);
                }
                #[cfg(feature = "dim3")]
                self.apply_axis_lock(physics_engine);
                self.set_continuous_collision_detection_mode(self.ccd_enabled, physics_engine);
                physics_engine.body_update_material(
                    self.base.get_space_id(),
                    self.base.get_body_handle(),
                    &self.init_material(),
                );
            }
        }
    }

    pub fn get_collision_layer(&self) -> u32 {
        self.base.get_collision_layer()
    }

    pub fn get_collision_mask(&self) -> u32 {
        self.base.get_collision_mask()
    }

    pub fn set_collision_layer(&mut self, layer: u32, physics_engine: &mut PhysicsEngine) {
        if self.base.get_collision_layer() != layer {
            self.base.set_collision_layer(layer, physics_engine);
            self.update_colliders_filters(physics_engine);
        }
    }

    pub fn set_collision_mask(&mut self, mask: u32, physics_engine: &mut PhysicsEngine) {
        if self.base.get_collision_mask() != mask {
            self.base.set_collision_mask(mask, physics_engine);
            self.update_colliders_filters(physics_engine);
        }
    }
}
// We won't use the pointers between threads, so it should be safe.
unsafe impl Sync for RapierBody {}
impl IRapierCollisionObject for RapierBody {
    fn get_base(&self) -> &RapierCollisionObjectBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierCollisionObjectBase {
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
        physics_ids: &mut PhysicsIds,
    ) {
        if space == self.base.get_space(physics_ids) {
            return;
        }
        self.set_space_before(physics_spaces, physics_ids);
        self.base
            .set_space(space, physics_engine, physics_spaces, physics_ids);
        self.recreate_shapes(physics_engine, physics_spaces, physics_ids);
        self.set_space_after(physics_engine, physics_spaces, physics_ids);
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
        let handle = self
            .base
            .create_shape(shape, p_shape_index, mat, physics_engine);
        self.init_collider(handle, self.base.get_space_id(), physics_engine);
        handle
    }

    fn recreate_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if !self.base.is_valid() {
            return;
        }
        RapierCollisionObjectBase::recreate_shapes(
            self,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
    }

    fn init_material(&self) -> Material {
        Material {
            friction: Some(self.friction),
            restitution: Some(self.bounce),
            contact_skin: Some(self.contact_skin),
            collision_layer: Some(self.base.get_collision_layer()),
            collision_mask: Some(self.base.get_collision_mask()),
            dominance: Some(self.base.get_dominance()),
        }
    }

    fn shapes_changed(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        self.mass_properties_changed(physics_engine, physics_spaces, physics_ids);
        self.wakeup(physics_engine);
        // in case we have a one way shape
        self.update_colliders_filters(physics_engine);
    }

    fn shape_changed(
        &mut self,
        shape_id: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        RapierCollisionObjectBase::shape_changed(
            self,
            shape_id,
            physics_engine,
            physics_spaces,
            physics_ids,
        );
    }

    #[cfg(feature = "serde-serialize")]
    fn export_json(&self) -> String {
        let state = BodyExport {
            body_state: &self.state,
            base_state: &self.base.state,
        };
        match serde_json::to_string_pretty(&state) {
            Ok(s) => {
                return s;
            }
            Err(e) => {
                godot_error!("Failed to serialize body to json: {}", e);
            }
        }
        "{}".to_string()
    }

    #[cfg(feature = "serde-serialize")]
    fn export_binary(&self) -> PackedByteArray {
        let mut buf = PackedByteArray::new();
        let state = BodyExport {
            body_state: &self.state,
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
                godot_error!("Failed to serialize body to binary: {}", e);
            }
        }
        buf
    }

    #[cfg(feature = "serde-serialize")]
    fn import_binary(&mut self, data: PackedByteArray) {
        match bincode::deserialize::<BodyImport>(data.as_slice()) {
            Ok(import) => {
                self.state = import.body_state;
                self.base.state = import.base_state;
            }
            Err(e) => {
                godot_error!("Failed to deserialize body from binary: {}", e);
            }
        }
    }
}
impl Drop for RapierBody {
    fn drop(&mut self) {
        if let Some(direct_state) = &self.direct_state {
            direct_state.clone().free();
        }
    }
}
