use godot::builtin::Rid;
use hashbrown::HashMap;
use hashbrown::HashSet;
use rapier::prelude::ColliderHandle;
use rapier::prelude::RigidBodyHandle;

use crate::bodies::rapier_collision_object_base::CollisionObjectType;
use crate::rapier_wrapper::handle::WorldHandle;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::types::*;
impl RemovedColliderInfo {
    pub fn new(
        rid: Rid,
        rb_handle: RigidBodyHandle,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) -> Self {
        Self {
            rid,
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
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "default_rid"))]
    pub rid: Rid,
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
    pub(crate) removed_colliders: HashMap<ColliderHandle, RemovedColliderInfo>,
    pub(crate) active_list: HashSet<RigidBodyHandle>,
    pub(crate) mass_properties_update_list: HashSet<RigidBodyHandle>,
    pub(crate) gravity_update_list: HashSet<RigidBodyHandle>,
    pub(crate) state_query_list: HashSet<RigidBodyHandle>,
    pub(crate) force_integrate_query_list: HashSet<RigidBodyHandle>,
    pub(crate) monitor_query_list: HashSet<RigidBodyHandle>,
    pub(crate) area_update_list: HashSet<RigidBodyHandle>,
    pub(crate) body_area_update_list: HashSet<RigidBodyHandle>,
    pub(crate) time_stepped: f32,
    pub(crate) island_count: i32,
    pub(crate) active_objects: i32,
    pub(crate) collision_pairs: i32,
    pub(crate) handle: WorldHandle,
}
impl RapierSpaceState {
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
        rid: Rid,
        rb_handle: RigidBodyHandle,
        instance_id: u64,
        shape_index: usize,
        collision_object_type: CollisionObjectType,
    ) {
        self.removed_colliders.insert(
            handle,
            RemovedColliderInfo::new(
                rid,
                rb_handle,
                instance_id,
                shape_index,
                collision_object_type,
            ),
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

    pub fn get_island_count(&self) -> i32 {
        self.island_count
    }

    pub fn get_active_objects(&self) -> i32 {
        self.active_objects
    }

    pub fn get_collision_pairs(&self) -> i32 {
        self.collision_pairs
    }

    pub fn get_active_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.active_list
    }

    pub fn get_mass_properties_update_list(&self) -> &HashSet<RigidBodyHandle> {
        &self.mass_properties_update_list
    }

    pub fn reset_mass_properties_update_list(&mut self) {
        self.mass_properties_update_list.clear();
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

    pub fn destroy(&mut self, physics_engine: &mut PhysicsEngine) {
        if self.is_valid() {
            physics_engine.world_destroy(self.handle);
            self.handle = WorldHandle::default();
        }
    }
}
#[cfg(test)]
mod tests {
    use rapier::prelude::*;

    use super::*;
    #[test]
    fn test_add_remove_rigid_body_from_mass_properties_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to mass_properties_update_list
        state.body_add_to_mass_properties_update_list(rb_handle);
        assert!(state.mass_properties_update_list.contains(&rb_handle));

        // Remove from mass_properties_update_list
        state.body_remove_from_mass_properties_update_list(rb_handle);
        assert!(!state.mass_properties_update_list.contains(&rb_handle));
    }
    #[test]
    fn test_reset_mass_properties_update_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);
        // Add an item to the list
        state.body_add_to_mass_properties_update_list(rb_handle);
        assert!(state.mass_properties_update_list.contains(&rb_handle));
        // Reset the list
        state.reset_mass_properties_update_list();
        assert!(state.mass_properties_update_list.is_empty());
    }

    #[test]
    fn test_add_remove_rigid_body_from_gravity_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to gravity_update_list
        state.body_add_to_gravity_update_list(rb_handle);
        assert!(state.gravity_update_list.contains(&rb_handle));

        // Remove from gravity_update_list
        state.body_remove_from_gravity_update_list(rb_handle);
        assert!(!state.gravity_update_list.contains(&rb_handle));
    }

    #[test]
    fn test_add_remove_rigid_body_from_active_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to active_list
        state.body_add_to_active_list(rb_handle);
        assert!(state.active_list.contains(&rb_handle));

        // Remove from active_list
        state.body_remove_from_active_list(rb_handle);
        assert!(!state.active_list.contains(&rb_handle));
    }

    #[test]
    fn test_add_remove_rigid_body_from_state_query_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to state_query_list
        state.body_add_to_state_query_list(rb_handle);
        assert!(state.state_query_list.contains(&rb_handle));

        // Remove from state_query_list
        state.body_remove_from_state_query_list(rb_handle);
        assert!(!state.state_query_list.contains(&rb_handle));
    }

    #[test]
    fn test_add_remove_rigid_body_from_force_integrate_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to force_integrate_query_list
        state.body_add_to_force_integrate_list(rb_handle);
        assert!(state.force_integrate_query_list.contains(&rb_handle));

        // Remove from force_integrate_query_list
        state.body_remove_from_force_integrate_list(rb_handle);
        assert!(!state.force_integrate_query_list.contains(&rb_handle));
    }

    #[test]
    fn test_add_remove_area_from_monitor_query_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to monitor_query_list
        state.area_add_to_monitor_query_list(rb_handle);
        assert!(state.monitor_query_list.contains(&rb_handle));

        // Remove from monitor_query_list
        state.monitor_query_list.remove(&rb_handle);
        assert!(!state.monitor_query_list.contains(&rb_handle));
    }

    #[test]
    fn test_add_remove_area_from_area_update_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to area_update_list
        state.area_add_to_area_update_list(rb_handle);
        assert!(state.area_update_list.contains(&rb_handle));

        // Remove from area_update_list
        state.area_remove_from_area_update_list(rb_handle);
        assert!(!state.area_update_list.contains(&rb_handle));
    }

    #[test]
    fn test_add_remove_body_from_area_update_list() {
        let mut state = RapierSpaceState::default();
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);

        // Add to body_area_update_list
        state.body_add_to_area_update_list(rb_handle);
        assert!(state.body_area_update_list.contains(&rb_handle));

        // Remove from body_area_update_list
        state.body_remove_from_area_update_list(rb_handle);
        assert!(!state.body_area_update_list.contains(&rb_handle));
    }
    #[test]
    fn test_add_removed_collider() {
        let mut state = RapierSpaceState::default();
        let collider_handle = ColliderHandle::from_raw_parts(1, 1);
        let rb_handle = RigidBodyHandle::from_raw_parts(1, 1);
        let rid = Rid::new(42);
        let instance_id = 123;
        let shape_index = 1;
        let collision_object_type = CollisionObjectType::Body;
        // Add removed collider
        state.add_removed_collider(
            collider_handle,
            rid,
            rb_handle,
            instance_id,
            shape_index,
            collision_object_type,
        );
        let removed_collider = state.get_removed_collider_info(&collider_handle);
        assert!(removed_collider.is_some());
        let collider_info = removed_collider.unwrap();
        assert_eq!(collider_info.rid, rid);
        assert_eq!(collider_info.rb_handle, rb_handle);
        assert_eq!(collider_info.instance_id, instance_id);
        assert_eq!(collider_info.shape_index, shape_index);
        assert_eq!(collider_info.collision_object_type, collision_object_type);
    }
    #[test]
    fn test_is_valid() {
        let mut state = RapierSpaceState::default();
        assert!(!state.is_valid());
        // Assign a non-default WorldHandle
        state.handle = WorldHandle::from_raw_parts(1, 1);
        assert!(state.is_valid());
    }
}
