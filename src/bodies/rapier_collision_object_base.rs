use godot::classes::ProjectSettings;
#[cfg(feature = "dim2")]
use godot::classes::physics_server_2d::*;
#[cfg(feature = "dim3")]
use godot::classes::physics_server_3d::*;
use godot::prelude::*;
use rapier::dynamics::RigidBodyHandle;
use rapier::geometry::ColliderHandle;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::PhysicsSpaces;
use servers::rapier_physics_singleton::RapierId;
use servers::rapier_physics_singleton::get_id_rid;
use servers::rapier_project_settings::RapierProjectSettings;
use spaces::rapier_space::RapierSpace;

use crate::rapier_wrapper::prelude::*;
use crate::types::*;
use crate::*;
#[cfg(feature = "dim2")]
const SLEEP_THRESHOLD_LINEAR: &str = "physics/2d/sleep_threshold_linear";
#[cfg(feature = "dim3")]
const SLEEP_THRESHOLD_LINEAR: &str = "physics/3d/sleep_threshold_linear";
#[cfg(feature = "dim2")]
const SLEEP_THRESHOLD_ANGULAR: &str = "physics/2d/sleep_threshold_angular";
#[cfg(feature = "dim3")]
const SLEEP_THRESHOLD_ANGULAR: &str = "physics/3d/sleep_threshold_angular";
#[cfg(feature = "dim2")]
const TIME_BEFORE_SLEEP: &str = "physics/2d/time_before_sleep";
#[cfg(feature = "dim3")]
const TIME_BEFORE_SLEEP: &str = "physics/3d/time_before_sleep";
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum CollisionObjectType {
    Area,
    Body,
}
#[derive(Debug, Clone, Copy)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct CollisionObjectShape {
    pub xform: Transform,
    pub id: RapierId,
    pub disabled: bool,
    pub one_way_collision: bool,
    pub one_way_collision_margin: real,
    #[cfg(feature = "dim2")]
    pub one_way_collision_direction: Vector,
    pub collider_handle: ColliderHandle,
}
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierCollisionObjectBaseState {
    pub(crate) id: RapierId,
    pub(crate) body_handle: RigidBodyHandle,
    pub(crate) space_id: RapierId,
    pub(crate) shapes: Vec<CollisionObjectShape>,
    pub(crate) transform: Transform,
    pub(crate) inv_transform: Transform,
}
impl Default for RapierCollisionObjectBaseState {
    fn default() -> Self {
        Self {
            id: RapierId::default(),
            body_handle: RigidBodyHandle::invalid(),
            space_id: RapierId::default(),
            shapes: Vec::new(),
            transform: Transform::IDENTITY,
            inv_transform: Transform::IDENTITY,
        }
    }
}
#[derive(Debug)]
pub struct RapierCollisionObjectBase {
    #[cfg(feature = "dim3")]
    user_flags: u32,
    collision_object_type: CollisionObjectType,
    rid: Rid,
    pub(crate) state: RapierCollisionObjectBaseState,
    instance_id: u64,
    canvas_instance_id: u64,
    pickable: bool,
    collision_mask: u32,
    collision_layer: u32,
    dominance: i8,
    pub(crate) is_debugging_contacts: bool,
    pub(crate) mode: BodyMode,
    massless: bool,
    pub(crate) activation_angular_threshold: real,
    pub(crate) activation_linear_threshold: real,
    pub(crate) activation_time_until_sleep: real,
    /// The shape holding the object's compound collider, when it has one.
    ///
    /// Shapes that are not compound parts keep colliders of their own alongside it, so the
    /// compound cannot be found by looking for the first valid handle -- this names it.
    compound_anchor: Option<usize>,
    /// The shapes gathered into that compound, in the order they were built.
    compound_members: Vec<usize>,
    /// For each part of that compound, the index of the shape it was built from.
    ///
    /// Not one part per shape: a skew is applied by decomposing the shape, and those pieces are
    /// spliced in individually.
    compound_part_shapes: Vec<usize>,
}
impl Default for RapierCollisionObjectBase {
    fn default() -> Self {
        Self::new(RapierId::default(), Rid::Invalid, CollisionObjectType::Body)
    }
}
impl RapierCollisionObjectBase {
    pub fn new(id: RapierId, rid: Rid, collision_object_type: CollisionObjectType) -> Self {
        let mut mode = BodyMode::RIGID;
        if collision_object_type == CollisionObjectType::Area {
            mode = BodyMode::STATIC;
        }
        let project_settings = ProjectSettings::singleton();
        let activation_angular_threshold = project_settings
            .get_setting_with_override(SLEEP_THRESHOLD_ANGULAR)
            .try_to()
            .unwrap_or_default();
        let length_unit = RapierProjectSettings::get_length_unit();
        let mut activation_linear_threshold = project_settings
            .get_setting_with_override(SLEEP_THRESHOLD_LINEAR)
            .try_to()
            .unwrap_or_default();
        activation_linear_threshold /= length_unit;
        let activation_time_until_sleep = project_settings
            .get_setting_with_override(TIME_BEFORE_SLEEP)
            .try_to()
            .unwrap_or_default();
        Self {
            #[cfg(feature = "dim3")]
            user_flags: 0,
            collision_object_type,
            rid,
            state: RapierCollisionObjectBaseState {
                id,
                body_handle: RigidBodyHandle::invalid(),
                space_id: RapierId::default(),
                shapes: Vec::new(),
                transform: Transform::IDENTITY,
                inv_transform: Transform::IDENTITY,
            },
            instance_id: 0,
            canvas_instance_id: 0,
            pickable: true,
            collision_mask: 1,
            collision_layer: 1,
            dominance: 0,
            is_debugging_contacts: false,
            mode,
            massless: false,
            activation_angular_threshold,
            activation_linear_threshold,
            activation_time_until_sleep,
            compound_anchor: None,
            compound_members: Vec::new(),
            compound_part_shapes: Vec::new(),
        }
    }

    pub(super) fn create_shape(
        &self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        mat: Material,
        physics_engine: &mut PhysicsEngine,
    ) -> ColliderHandle {
        if shape.collider_handle != ColliderHandle::invalid() {
            godot_error!("collider is valid");
        }
        let mut user_data = UserData::default();
        self.set_collider_user_data(&mut user_data, p_shape_index);
        match self.collision_object_type {
            CollisionObjectType::Body => physics_engine.collider_create_solid(
                self.state.space_id,
                shape.id,
                &mat,
                self.state.body_handle,
                &user_data,
            ),
            CollisionObjectType::Area => physics_engine.collider_create_sensor(
                self.state.space_id,
                shape.id,
                &mat,
                self.state.body_handle,
                &user_data,
            ),
        }
    }

    /// Whether this object's shapes should live in one compound collider rather than one each.
    ///
    /// Only static solid bodies without one-way shapes for now: a compound reports contacts by
    /// subshape rather than by collider, which per-shape one-way filtering and sensor events
    /// cannot express.
    ///
    /// Shapes the compound cannot speak for are left out of it and keep a collider each, while the
    /// rest still gather into one -- so a single such shape no longer costs every other shape the
    /// seam fix.
    pub(crate) fn wants_compound_collider(&self, physics_engine: &PhysicsEngine) -> bool {
        self.collision_object_type == CollisionObjectType::Body
            && self.mode == BodyMode::STATIC
            && self.compound_candidates(physics_engine).count() > 1
    }

    /// The shapes that would go into the object's compound, in index order.
    ///
    /// Left out are shapes that cannot legally be a compound part -- concave polygons, heightmaps,
    /// world boundaries -- and one-way shapes. One-way filtering is answered per collider, from the
    /// shape index in its user data, and a compound reports one index for every part it holds, so a
    /// one-way shape has to keep a collider of its own to stay one-way.
    ///
    /// Only meaningful together with the rest of [`Self::wants_compound_collider`]; on its own it
    /// says nothing about whether the object should have a compound at all.
    fn compound_candidates<'a>(
        &'a self,
        physics_engine: &'a PhysicsEngine,
    ) -> impl Iterator<Item = usize> + 'a {
        self.state
            .shapes
            .iter()
            .enumerate()
            .filter(|(_, shape)| !shape.disabled && !shape.one_way_collision)
            .filter(move |(_, shape)| shape_can_be_compound_part(physics_engine, shape.id))
            .map(|(index, _)| index)
    }

    /// Whether the object currently holds a compound collider.
    pub(crate) fn is_compound(&self) -> bool {
        self.compound_anchor.is_some()
    }

    /// Whether a collider reporting `shape_index` is the object's compound rather than one of the
    /// shapes keeping a collider of its own.
    pub(crate) fn hit_is_compound(&self, shape_index: usize) -> bool {
        self.compound_anchor == Some(shape_index)
    }

    /// The shapes gathered into the compound, which a hit on it stands for.
    pub(crate) fn compound_member_indices(&self) -> &[usize] {
        &self.compound_members
    }

    /// The shapes the compound took, with their indices.
    pub(crate) fn compound_shapes(&self) -> impl Iterator<Item = (usize, &CollisionObjectShape)> {
        self.compound_members
            .iter()
            .filter_map(|&index| self.state.shapes.get(index).map(|shape| (index, shape)))
    }

    /// Whether the shape is one the compound took, rather than one keeping its own collider.
    pub(crate) fn is_compound_member(&self, shape_index: usize) -> bool {
        self.compound_members.contains(&shape_index)
    }

    /// The shape holding the compound collider.
    pub(super) fn compound_anchor_index(&self) -> Option<usize> {
        self.compound_anchor
    }

    /// The member shapes as compound parts, with the object's scale baked in the way
    /// [`Self::update_shape_transform`] bakes it into a standalone collider.
    fn compound_parts(&self) -> Vec<ShapeInfo> {
        let scale = transform_scale(&self.state.transform);
        self.compound_members
            .iter()
            .filter_map(|&index| self.state.shapes.get(index))
            .map(|shape| {
                let mut info = shape_info_from_body_shape(shape.id, shape.xform);
                info.scale = vector_to_rapier(vector_to_godot(info.scale) * scale);
                info.transform.translation *= vector_to_rapier(scale);
                info
            })
            .collect()
    }

    /// Gathers the object's eligible shapes into one compound collider, leaving the rest to keep a
    /// collider each.
    ///
    /// The handle is reported so the caller can park it on the anchor shape; an invalid one means
    /// the compound was refused and every shape has to go back to a collider of its own.
    pub(super) fn create_compound_collider(
        &mut self,
        mat: Material,
        physics_engine: &mut PhysicsEngine,
    ) -> ColliderHandle {
        self.compound_members = self.compound_candidates(physics_engine).collect();
        let Some(&anchor) = self.compound_members.first() else {
            return ColliderHandle::invalid();
        };
        if !self.is_valid() {
            return ColliderHandle::invalid();
        }
        let mut user_data = UserData::default();
        self.set_collider_user_data(&mut user_data, anchor);
        let (handle, part_sources) = physics_engine.collider_create_solid_compound(
            self.state.space_id,
            &self.compound_parts(),
            &mat,
            self.state.body_handle,
            &user_data,
        );
        self.compound_anchor = (handle != ColliderHandle::invalid()).then_some(anchor);
        self.set_compound_part_shapes(part_sources);
        handle
    }

    /// Forgets the compound, so the object is back to a collider per shape.
    pub(super) fn clear_compound(&mut self) {
        self.compound_anchor = None;
        self.compound_members.clear();
        self.compound_part_shapes.clear();
    }

    /// Records which shape each compound part came from, translating the part sources -- which
    /// index the members in order -- into indices into all of the object's shapes.
    fn set_compound_part_shapes(&mut self, part_sources: Vec<usize>) {
        self.compound_part_shapes = part_sources
            .into_iter()
            .filter_map(|source| self.compound_members.get(source).copied())
            .collect();
    }

    /// Rebuilds the compound collider's shape in place, keeping the collider -- and every contact
    /// pair referencing it -- alive, so no exit and enter events fire for a geometry edit.
    pub(super) fn update_compound_collider(&mut self, physics_engine: &mut PhysicsEngine) {
        let part_sources = physics_engine.collider_update_solid_compound(
            self.state.space_id,
            self.compound_collider_handle(),
            &self.compound_parts(),
        );
        self.set_compound_part_shapes(part_sources);
    }

    /// The shape a compound part was built from.
    pub(crate) fn shape_index_for_compound_part(&self, part_index: u32) -> Option<usize> {
        self.compound_part_shapes.get(part_index as usize).copied()
    }

    /// The collider holding the whole object while it is a compound.
    fn compound_collider_handle(&self) -> ColliderHandle {
        self.compound_anchor
            .and_then(|anchor| self.state.shapes.get(anchor))
            .map(|shape| shape.collider_handle)
            .unwrap_or(ColliderHandle::invalid())
    }

    pub(super) fn update_shapes_indexes(&mut self, physics_engine: &mut PhysicsEngine) {
        if !self.is_valid() {
            return;
        }
        for (shape_index, shape) in self.state.shapes.iter().enumerate() {
            let mut user_data = UserData::default();
            self.set_collider_user_data(&mut user_data, shape_index);
            physics_engine.collider_set_user_data(
                self.state.space_id,
                shape.collider_handle,
                &user_data,
            );
        }
    }

    pub(super) fn destroy_shapes(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if let Some(space) = physics_spaces.get_mut(&self.get_space(physics_ids)) {
            for (i, shape) in self.state.shapes.iter_mut().enumerate() {
                if shape.collider_handle == ColliderHandle::invalid() {
                    // skip
                    continue;
                }
                // Keep track of body information for delayed removal
                space.get_mut_state().add_removed_collider(
                    shape.collider_handle,
                    self.state.id,
                    self.instance_id,
                    i,
                    self.collision_object_type,
                );
                physics_engine.collider_destroy(self.state.space_id, shape.collider_handle);
                shape.collider_handle = ColliderHandle::invalid();
            }
        }
    }

    pub(super) fn destroy_shape(
        &self,
        shape: CollisionObjectShape,
        p_shape_index: usize,
        physics_spaces: &mut PhysicsSpaces,
        physics_engine: &mut PhysicsEngine,
        physics_ids: &PhysicsIds,
    ) -> ColliderHandle {
        if shape.collider_handle != ColliderHandle::invalid() {
            if let Some(space) = physics_spaces.get_mut(&self.get_space(physics_ids)) {
                // Keep track of body information for delayed removal
                space.get_mut_state().add_removed_collider(
                    shape.collider_handle,
                    self.state.id,
                    self.instance_id,
                    p_shape_index,
                    self.get_type(),
                );
            }
            physics_engine.collider_destroy(self.state.space_id, shape.collider_handle);
        }
        ColliderHandle::invalid()
    }

    pub(super) fn update_shape_transform(
        &mut self,
        shape_index: usize,
        shape: CollisionObjectShape,
        physics_engine: &mut PhysicsEngine,
    ) {
        // A compound bakes every shape's transform into the one collider, so a single shape moving
        // means rebuilding it -- pushing this shape's transform onto the collider would replace
        // the whole compound with just this shape.
        if self.is_compound() && self.compound_members.contains(&shape_index) {
            self.update_compound_collider(physics_engine);
            return;
        }
        if !self.is_space_valid() || shape.collider_handle == ColliderHandle::invalid() {
            return;
        }
        let scale = transform_scale(&self.state.transform);
        let mut shape_info = shape_info_from_body_shape(shape.id, shape.xform);
        shape_info.scale = vector_to_rapier(vector_to_godot(shape_info.scale) * scale);
        let position = shape_info.transform.translation * vector_to_rapier(scale);
        shape_info.transform.translation = position;
        physics_engine.collider_set_transform(
            self.state.space_id,
            shape.collider_handle,
            shape_info,
        );
    }

    pub(super) fn update_transform(&mut self, physics_engine: &mut PhysicsEngine) {
        if !self.is_valid() {
            return;
        }
        let position =
            physics_engine.body_get_position(self.state.space_id, self.state.body_handle);
        let rotation = physics_engine.body_get_angle(self.state.space_id, self.state.body_handle);
        self.state.transform =
            transform_update(&self.state.transform, rotation, vector_to_godot(position));
        self.state.inv_transform = transform_inverse(&self.state.transform);
    }

    pub(super) fn set_space(
        &mut self,
        p_space: Rid,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &mut PhysicsIds,
    ) {
        // previous space
        if self.is_space_valid() {
            self.destroy_body(physics_engine);
            self.destroy_shapes(physics_engine, physics_spaces, physics_ids);
            if let Some(space) = physics_spaces.get_mut(&self.get_space(physics_ids)) {
                space
                    .get_mut_state()
                    .reset_space_if_empty(physics_engine, &RapierSpace::get_world_settings());
            }
        }
        if let Some(space) = physics_spaces.get_mut(&p_space) {
            self.state.space_id = space.get_state().get_id();
            self.state.space_id = space.get_state().get_id();
            self.is_debugging_contacts = space.is_debugging_contacts();
        } else {
            self.state.space_id = WorldHandle::default();
            self.state.space_id = RapierId::default();
            return;
        }
        let position = vector_to_rapier(self.state.transform.origin);
        let angle = transform_rotation_rapier(&self.state.transform);
        if self.mode == BodyMode::STATIC {
            self.state.body_handle = physics_engine.body_create(
                self.state.space_id,
                position,
                angle,
                BodyType::Static,
                self.activation_angular_threshold,
                self.activation_linear_threshold,
                self.activation_time_until_sleep,
            );
        } else if self.mode == BodyMode::KINEMATIC {
            self.state.body_handle = physics_engine.body_create(
                self.state.space_id,
                position,
                angle,
                BodyType::Kinematic,
                self.activation_angular_threshold,
                self.activation_linear_threshold,
                self.activation_time_until_sleep,
            );
        } else {
            self.state.body_handle = physics_engine.body_create(
                self.state.space_id,
                position,
                angle,
                BodyType::Dynamic,
                self.activation_angular_threshold,
                self.activation_linear_threshold,
                self.activation_time_until_sleep,
            );
        }
        let mut user_data = UserData::default();
        self.set_collider_user_data(&mut user_data, 0);
        physics_engine.body_set_user_data(self.state.space_id, self.state.body_handle, &user_data);
    }

    pub fn get_space_id(&self) -> WorldHandle {
        self.state.space_id
    }

    pub fn is_valid(&self) -> bool {
        self.is_body_valid() && self.is_space_valid()
    }

    pub fn is_body_valid(&self) -> bool {
        self.state.body_handle != RigidBodyHandle::invalid()
    }

    pub fn is_space_valid(&self) -> bool {
        self.state.space_id != WorldHandle::default()
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn get_id(&self) -> RapierId {
        self.state.id
    }

    pub fn set_instance_id(&mut self, p_instance_id: u64) {
        self.instance_id = p_instance_id;
    }

    pub fn get_instance_id(&self) -> u64 {
        self.instance_id
    }

    pub fn get_body_handle(&self) -> RigidBodyHandle {
        self.state.body_handle
    }

    #[cfg(feature = "dim2")]
    pub fn set_canvas_instance_id(&mut self, p_canvas_instance_id: u64) {
        self.canvas_instance_id = p_canvas_instance_id;
    }

    pub fn get_canvas_instance_id(&self) -> u64 {
        self.canvas_instance_id
    }

    pub fn set_collider_user_data(&self, r_user_data: &mut UserData, p_shape_index: usize) {
        r_user_data.part1 = self.get_id();
        r_user_data.part2 = p_shape_index as u64;
    }

    pub fn get_collider_user_data(
        p_user_data: &UserData,
        physics_ids: &PhysicsIds,
    ) -> (Rid, usize) {
        let id = p_user_data.part1;
        let rid = get_id_rid(id, physics_ids);
        (rid, p_user_data.shape_index() as usize)
    }

    pub fn get_type(&self) -> CollisionObjectType {
        self.collision_object_type
    }

    pub fn get_shape_count(&self) -> i32 {
        self.state.shapes.len() as i32
    }

    pub fn get_shape(&self, physics_ids: &PhysicsIds, idx: usize) -> Rid {
        if let Some(shape) = self.state.shapes.get(idx) {
            return get_id_rid(shape.id, physics_ids);
        }
        Rid::Invalid
    }

    pub fn get_shape_transform(&self, idx: usize) -> Transform {
        if let Some(shape) = self.state.shapes.get(idx) {
            return shape.xform;
        }
        Transform::IDENTITY
    }

    pub fn set_transform(
        &mut self,
        p_transform: Transform,
        wake_up: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        let teleport = self.state.transform == Transform::IDENTITY;
        let old_scale = transform_scale(&self.state.transform);
        self.state.transform = p_transform;
        self.state.inv_transform = transform_inverse(&self.state.transform);
        if !self.is_valid() {
            return;
        }
        let origin = self.state.transform.origin;
        let position = vector_to_rapier(origin);
        let rotation = transform_rotation_rapier(&self.state.transform);
        physics_engine.body_set_transform(
            self.state.space_id,
            self.state.body_handle,
            position,
            rotation,
            teleport,
            wake_up,
        );
        // Only origin and rotation reach the rigid body; the object's scale lives in the colliders'
        // own scaled shapes, so a scale change has to be pushed to each of them. Updating the shape
        // transforms rather than recreating the shapes avoids losing collisions for a step (#398).
        if transform_scale(&self.state.transform) != old_scale {
            for i in 0..self.state.shapes.len() {
                let shape = self.state.shapes[i];
                self.update_shape_transform(i, shape, physics_engine);
            }
        }
    }

    pub fn get_transform(&self) -> Transform {
        self.state.transform
    }

    pub fn get_space(&self, physics_ids: &PhysicsIds) -> Rid {
        get_id_rid(self.state.space_id, physics_ids)
    }

    pub fn is_shape_disabled(&self, idx: usize) -> bool {
        if let Some(shape) = self.state.shapes.get(idx) {
            return shape.disabled;
        }
        true
    }

    #[cfg(feature = "dim2")]
    pub fn set_shape_as_one_way_collision(
        &mut self,
        p_idx: usize,
        p_one_way_collision: bool,
        p_margin: real,
        p_direction: Vector,
    ) {
        if let Some(shape) = self.state.shapes.get_mut(p_idx) {
            shape.one_way_collision = p_one_way_collision;
            shape.one_way_collision_margin = p_margin;
            shape.one_way_collision_direction = p_direction
                .try_normalized()
                .unwrap_or(Vector::new(0.0, 1.0));
        }
    }

    pub fn is_shape_set_as_one_way_collision(&self, p_idx: usize) -> bool {
        if let Some(shape) = self.state.shapes.get(p_idx) {
            return shape.one_way_collision;
        }
        false
    }

    pub fn get_shape_one_way_collision_margin(&self, p_idx: usize) -> real {
        if let Some(shape) = self.state.shapes.get(p_idx) {
            return shape.one_way_collision_margin;
        }
        0.0
    }

    #[cfg(feature = "dim2")]
    pub fn get_shape_one_way_collision_direction(&self, p_idx: usize) -> Vector {
        if let Some(shape) = self.state.shapes.get(p_idx) {
            return shape.one_way_collision_direction;
        }
        Vector::new(0.0, 1.0)
    }

    #[cfg(feature = "dim3")]
    pub fn get_shape_one_way_collision_direction(&self, _p_idx: usize) -> Vector {
        Vector::new(0.0, 1.0, 0.0)
    }

    pub fn set_collision_mask(&mut self, p_mask: u32, physics_engine: &mut PhysicsEngine) {
        self.collision_mask = p_mask;
        if self.is_valid() {
            let material = Material::new(self.collision_layer, self.collision_mask, self.dominance);
            physics_engine.body_update_material(
                self.state.space_id,
                self.state.body_handle,
                &material,
                true,
            );
        }
    }

    pub fn get_collision_mask(&self) -> u32 {
        self.collision_mask
    }

    pub fn set_collision_layer(&mut self, p_layer: u32, physics_engine: &mut PhysicsEngine) {
        self.collision_layer = p_layer;
        if self.is_valid() {
            let material = Material::new(self.collision_layer, self.collision_mask, self.dominance);
            physics_engine.body_update_material(
                self.state.space_id,
                self.state.body_handle,
                &material,
                true,
            );
        }
    }

    pub fn get_collision_layer(&self) -> u32 {
        self.collision_layer
    }

    pub fn set_dominance(&mut self, dominance: i8, physics_engine: &mut PhysicsEngine) {
        self.dominance = dominance;
        if self.is_valid() {
            let material = Material::new(self.collision_layer, self.collision_mask, self.dominance);
            physics_engine.body_update_material(
                self.state.space_id,
                self.state.body_handle,
                &material,
                true,
            );
        }
    }

    pub fn get_dominance(&self) -> i8 {
        self.dominance
    }

    pub fn get_mode(&self) -> BodyMode {
        self.mode
    }

    pub fn set_pickable(&mut self, p_pickable: bool) {
        self.pickable = p_pickable;
    }

    pub fn get_pickable(&self) -> bool {
        self.pickable
    }

    pub fn destroy_body(&mut self, physics_engine: &mut PhysicsEngine) {
        if self.state.body_handle != RigidBodyHandle::invalid() {
            physics_engine.body_destroy(self.state.space_id, self.state.body_handle);
            self.state.body_handle = RigidBodyHandle::invalid();
        }
    }

    #[cfg(feature = "dim3")]
    pub fn set_user_flags(&mut self, p_flags: u32) {
        self.user_flags = p_flags;
    }

    #[cfg(feature = "dim3")]
    pub fn get_user_flags(&self) -> u32 {
        self.user_flags
    }

    pub fn set_massless(&mut self, p_massless: bool) {
        self.massless = p_massless;
    }

    pub fn is_massless(&self) -> bool {
        self.massless
    }
}
impl Drop for RapierCollisionObjectBase {
    fn drop(&mut self) {
        if self.state.body_handle != RigidBodyHandle::invalid() {
            godot_error!("Body leaked");
        }
    }
}
