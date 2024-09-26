use hashbrown::HashMap;
use hashbrown::HashSet;
use rapier::prelude::ColliderHandle;
use rapier::prelude::RigidBodyHandle;

use crate::bodies::rapier_collision_object_base::CollisionObjectType;
use crate::rapier_wrapper::handle::WorldHandle;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::rapier_wrapper::prelude::WorldSettings;
impl RemovedColliderInfo {
    pub fn new(
        rb_handle: RigidBodyHandle,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) -> Self {
        Self {
            rb_handle,
            instance_id,
            shape_index,
            collision_object_type,
        }
    }
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct RemovedColliderInfo {
    pub rb_handle: RigidBodyHandle,
    pub instance_id: u64,
    pub shape_index: usize,
    pub collision_object_type: CollisionObjectType,
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
#[serde(default)]
#[derive(Debug, PartialEq, Clone, Default)]
pub struct RapierSpaceState {
    #[cfg_attr(
        feature = "serde-serialize",
        serde(
            serialize_with = "rapier::utils::serde::serialize_to_vec_tuple",
            deserialize_with = "rapier::utils::serde::deserialize_from_vec_tuple"
        )
    )]
    removed_colliders: HashMap<ColliderHandle, RemovedColliderInfo>,
    active_list: HashSet<RigidBodyHandle>,
    mass_properties_update_list: HashSet<RigidBodyHandle>,
    gravity_update_list: HashSet<RigidBodyHandle>,
    state_query_list: HashSet<RigidBodyHandle>,
    force_integrate_query_list: HashSet<RigidBodyHandle>,
    monitor_query_list: HashSet<RigidBodyHandle>,
    area_update_list: HashSet<RigidBodyHandle>,
    body_area_update_list: HashSet<RigidBodyHandle>,
    time_stepped: f32,
    active_objects: i32,
    handle: WorldHandle,
}
impl RapierSpaceState {
    pub fn new(physics_engine: &mut PhysicsEngine, world_settings: &WorldSettings) -> Self {
        let handle = physics_engine.world_create(world_settings);
        Self {
            handle,
            ..Default::default()
        }
    }

    pub fn body_add_to_mass_properties_update_list(&mut self, body: RigidBodyHandle) {
        self.mass_properties_update_list.insert(body);
    }

    pub fn body_remove_from_mass_properties_update_list(&mut self, body: RigidBodyHandle) {
        self.mass_properties_update_list.remove(&body);
    }

    pub fn body_add_to_gravity_update_list(&mut self, body: RigidBodyHandle) {
        self.gravity_update_list.insert(body);
    }

    pub fn body_remove_from_gravity_update_list(&mut self, body: RigidBodyHandle) {
        self.gravity_update_list.remove(&body);
    }

    pub fn body_add_to_active_list(&mut self, body: RigidBodyHandle) {
        self.active_list.insert(body);
    }

    pub fn body_remove_from_active_list(&mut self, body: RigidBodyHandle) {
        self.active_list.remove(&body);
    }

    pub fn body_add_to_state_query_list(&mut self, body: RigidBodyHandle) {
        self.state_query_list.insert(body);
    }

    pub fn body_remove_from_state_query_list(&mut self, body: RigidBodyHandle) {
        self.state_query_list.remove(&body);
    }

    pub fn body_add_to_force_integrate_list(&mut self, body: RigidBodyHandle) {
        self.force_integrate_query_list.insert(body);
    }

    pub fn body_remove_from_force_integrate_list(&mut self, body: RigidBodyHandle) {
        self.force_integrate_query_list.remove(&body);
    }

    pub fn area_add_to_monitor_query_list(&mut self, area: RigidBodyHandle) {
        self.monitor_query_list.insert(area);
    }

    pub fn area_add_to_area_update_list(&mut self, area: RigidBodyHandle) {
        self.area_update_list.insert(area);
    }

    pub fn area_remove_from_area_update_list(&mut self, area: RigidBodyHandle) {
        self.area_update_list.remove(&area);
    }

    pub fn body_add_to_area_update_list(&mut self, body: RigidBodyHandle) {
        self.body_area_update_list.insert(body);
    }

    pub fn body_remove_from_area_update_list(&mut self, body: RigidBodyHandle) {
        self.body_area_update_list.remove(&body);
    }

    pub fn add_removed_collider(
        &mut self,
        handle: ColliderHandle,
        rb_handle: RigidBodyHandle,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) {
        self.removed_colliders.insert(
            handle,
            RemovedColliderInfo::new(rb_handle, instance_id, shape_index, collision_object_type),
        );
    }

    pub fn get_removed_collider_info(
        &self,
        handle: &ColliderHandle,
    ) -> Option<&RemovedColliderInfo> {
        self.removed_colliders.get(handle)
    }

    pub fn get_handle(&self) -> WorldHandle {
        self.handle
    }

    pub fn is_valid(&self) -> bool {
        self.handle != WorldHandle::default()
    }

    pub fn get_active_objects(&self) -> i32 {
        self.active_objects
    }

    pub fn set_active_objects(&mut self, count: i32) {
        self.active_objects = count;
    }

    pub fn get_time_stepped(&self) -> f32 {
        self.time_stepped
    }

    pub fn set_time_stepped(&mut self, time: f32) {
        self.time_stepped = time;
    }

    pub fn get_active_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.active_list
    }

    pub fn get_mass_properties_update_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.mass_properties_update_list
    }

    pub fn get_area_update_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.area_update_list
    }

    pub fn get_body_area_update_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.body_area_update_list
    }

    pub fn get_gravity_update_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.gravity_update_list
    }

    pub fn get_active_bodies(&self) -> Vec<RigidBodyHandle> {
        self.active_list.clone().into_iter().collect()
    }

    pub fn get_state_query_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.state_query_list
    }

    pub fn get_force_integrate_query_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.force_integrate_query_list
    }

    pub fn get_monitor_query_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.monitor_query_list
    }

    pub fn reset_mass_properties_update_list(&mut self) {
        self.mass_properties_update_list.clear();
    }

    pub fn reset_monitor_query_list(&mut self) {
        self.monitor_query_list.clear();
    }

    pub fn reset_removed_colliders(&mut self) {
        self.removed_colliders.clear();
    }

    pub fn destroy(&mut self, physics_engine: &mut PhysicsEngine) {
        if self.is_valid() {
            physics_engine.world_destroy(self.handle);
            self.handle = WorldHandle::default();
        }
    }

    // Reset the rapier world if it is empty. Helps maintain determinism when deleting all things but not reloading.
    pub fn reset_space_if_empty(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        world_settings: &WorldSettings,
    ) {
        if self.is_valid() {
            physics_engine.world_reset_if_empty(self.handle, world_settings);
        }
    }
}
#[cfg(test)]
mod tests {
    use rapier::prelude::ColliderHandle;
    use rapier::prelude::RigidBodyHandle;

    use super::*;
    use crate::bodies::rapier_collision_object_base::CollisionObjectType;
    fn create_test_rigid_body_handle() -> RigidBodyHandle {
        RigidBodyHandle::from_raw_parts(1, 0)
    }
    fn create_test_collider_handle() -> ColliderHandle {
        ColliderHandle::from_raw_parts(1, 0)
    }
    fn create_world_settings() -> WorldSettings {
        WorldSettings {
            particle_radius: 1.0,
            smoothing_factor: 1.0,
            counters_enabled: true,
        }
    }
    #[test]
    fn test_rapier_space_state_new() {
        let mut physics_engine = PhysicsEngine::default();
        let state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        assert!(state.is_valid());
        assert_eq!(state.get_active_objects(), 0);
        assert!(state.get_active_list().is_empty());
        assert!(state.get_mass_properties_update_list().is_empty());
        assert!(state.get_gravity_update_list().is_empty());
        assert!(state.get_state_query_list().is_empty());
        assert!(state.get_force_integrate_query_list().is_empty());
        assert!(state.get_monitor_query_list().is_empty());
        assert!(state.get_area_update_list().is_empty());
        assert!(state.get_body_area_update_list().is_empty());
        assert!(state.get_active_objects() == 0);
        assert!(state.get_time_stepped() == 0.0);
        assert!(state.get_handle() != WorldHandle::default());
        assert!(state.is_valid());
    }
    #[test]
    fn test_body_add_and_remove_from_active_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.body_add_to_active_list(rb_handle);
        assert!(state.get_active_list().contains(&rb_handle));
        state.body_remove_from_active_list(rb_handle);
        assert!(!state.get_active_list().contains(&rb_handle));
        assert!(state.get_active_bodies().is_empty());
    }
    #[test]
    fn test_body_add_and_remove_reset_from_mass_properties_update_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.body_add_to_mass_properties_update_list(rb_handle);
        assert!(state.get_mass_properties_update_list().contains(&rb_handle));
        state.body_remove_from_mass_properties_update_list(rb_handle);
        assert!(!state.get_mass_properties_update_list().contains(&rb_handle));
        state.body_add_to_mass_properties_update_list(rb_handle);
        assert!(state.get_mass_properties_update_list().contains(&rb_handle));
        state.reset_mass_properties_update_list();
        assert!(state.get_mass_properties_update_list().is_empty());
    }
    #[test]
    fn test_body_add_and_remove_from_gravity_update_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.body_add_to_gravity_update_list(rb_handle);
        assert!(state.get_gravity_update_list().contains(&rb_handle));
        state.body_remove_from_gravity_update_list(rb_handle);
        assert!(!state.get_gravity_update_list().contains(&rb_handle));
    }
    #[test]
    fn test_body_add_and_remove_from_state_query_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.body_add_to_state_query_list(rb_handle);
        assert!(state.get_state_query_list().contains(&rb_handle));
        state.body_remove_from_state_query_list(rb_handle);
        assert!(!state.get_state_query_list().contains(&rb_handle));
    }
    #[test]
    fn test_body_add_and_remove_from_force_integrate_query_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.body_add_to_force_integrate_list(rb_handle);
        assert!(state.get_force_integrate_query_list().contains(&rb_handle));
        state.body_remove_from_force_integrate_list(rb_handle);
        assert!(!state.get_force_integrate_query_list().contains(&rb_handle));
    }
    #[test]
    fn test_area_add_and_reset_from_monitor_query_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.area_add_to_monitor_query_list(rb_handle);
        assert!(state.get_monitor_query_list().contains(&rb_handle));
        state.reset_monitor_query_list();
        assert!(state.get_monitor_query_list().is_empty());
    }
    #[test]
    fn test_body_add_and_remove_from_area_update_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.body_add_to_area_update_list(rb_handle);
        assert!(state.get_body_area_update_list().contains(&rb_handle));
        state.body_remove_from_area_update_list(rb_handle);
        assert!(!state.get_body_area_update_list().contains(&rb_handle));
    }
    #[test]
    fn test_area_add_and_remove_from_area_update_list() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let rb_handle = create_test_rigid_body_handle();
        state.area_add_to_area_update_list(rb_handle);
        assert!(state.get_area_update_list().contains(&rb_handle));
        state.area_remove_from_area_update_list(rb_handle);
        assert!(!state.get_area_update_list().contains(&rb_handle));
    }
    #[test]
    fn test_add_removed_collider() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        let collider_handle = create_test_collider_handle();
        let rb_handle = create_test_rigid_body_handle();
        let instance_id = 123;
        let shape_index = 0;
        let collision_object_type = CollisionObjectType::Body;
        state.add_removed_collider(
            collider_handle,
            rb_handle,
            instance_id,
            shape_index,
            collision_object_type,
        );
        let removed_info = state.get_removed_collider_info(&collider_handle);
        assert!(removed_info.is_some());
        let removed_info = removed_info.unwrap();
        assert_eq!(removed_info.rb_handle, rb_handle);
        assert_eq!(removed_info.instance_id, instance_id);
        assert_eq!(removed_info.shape_index, shape_index);
        assert_eq!(removed_info.collision_object_type, collision_object_type);
        state.reset_removed_colliders();
        assert!(state.get_removed_collider_info(&collider_handle).is_none());
    }
    #[test]
    fn test_set_and_get_active_objects() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        state.set_active_objects(10);
        assert_eq!(state.get_active_objects(), 10);
    }
    #[test]
    fn test_set_and_get_time_stepped() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        state.set_time_stepped(0.0016);
        assert_eq!(state.get_time_stepped(), 0.0016);
    }
    #[test]
    fn test_destroy() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        state.destroy(&mut physics_engine);
        assert!(!state.is_valid());
    }
    #[test]
    fn test_reset_space_if_empty() {
        let mut physics_engine = PhysicsEngine::default();
        let mut state = RapierSpaceState::new(&mut physics_engine, &create_world_settings());
        state.reset_space_if_empty(&mut physics_engine, &create_world_settings());
        assert!(state.is_valid());
    }
}
