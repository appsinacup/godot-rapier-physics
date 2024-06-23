use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::PhysicsCollisionObjects;
#[derive(Default)]
pub struct OneWayDirection {
    pub body1: bool,
    pub body2: bool,
    pub pixel_body1_margin: Real,
    pub pixel_body2_margin: Real,
}
pub type CollisionFilterCallback = fn(
    filter_info: &CollisionFilterInfo,
    physics_collision_objects: &PhysicsCollisionObjects,
) -> bool;
pub type CollisionModifyContactsCallback = fn(
    filter_info: &CollisionFilterInfo,
    physics_collision_objects: &PhysicsCollisionObjects,
) -> OneWayDirection;
pub struct CollisionFilterInfo {
    pub user_data1: UserData,
    pub user_data2: UserData,
}
pub struct PhysicsHooksCollisionFilter<'a> {
    pub collision_filter_body_callback: &'a CollisionFilterCallback,
    pub collision_filter_sensor_callback: &'a CollisionFilterCallback,
    pub collision_modify_contacts_callback: &'a CollisionModifyContactsCallback,
    pub physics_collision_objects: &'a PhysicsCollisionObjects,
}
impl<'a> PhysicsHooks for PhysicsHooksCollisionFilter<'a> {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let result = Some(SolverFlags::COMPUTE_IMPULSES);
        let Some(collider1) = context.colliders.get(context.collider1) else {
            return result;
        };
        let Some(collider2) = context.colliders.get(context.collider2) else {
            return result;
        };
        let filter_info = CollisionFilterInfo {
            user_data1: UserData::new(collider1.user_data),
            user_data2: UserData::new(collider2.user_data),
        };
        // Handle contact filtering for rigid bodies
        if !(self.collision_filter_body_callback)(&filter_info, self.physics_collision_objects) {
            return None;
        }
        result
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        let Some(collider1) = context.colliders.get(context.collider1) else {
            return false;
        };
        let Some(collider2) = context.colliders.get(context.collider2) else {
            return false;
        };
        let filter_info = CollisionFilterInfo {
            user_data1: UserData::new(collider1.user_data),
            user_data2: UserData::new(collider2.user_data),
        };
        // Handle intersection filtering for sensors
        (self.collision_filter_sensor_callback)(&filter_info, self.physics_collision_objects)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let Some(rididbody1_handle) = context.rigid_body1 else {
            return;
        };
        let Some(rididbody2_handle) = context.rigid_body2 else {
            return;
        };
        let Some(collider1) = context.colliders.get(context.collider1) else {
            return;
        };
        let Some(collider2) = context.colliders.get(context.collider2) else {
            return;
        };
        let Some(body1) = context.bodies.get(rididbody1_handle) else {
            return;
        };
        let Some(body2) = context.bodies.get(rididbody2_handle) else {
            return;
        };
        let filter_info = CollisionFilterInfo {
            user_data1: UserData::new(collider1.user_data),
            user_data2: UserData::new(collider2.user_data),
        };
        let allowed_local_n1 = collider1.position().rotation * Vector::y();
        let allowed_local_n2 = collider2.position().rotation * Vector::y();
        let one_way_direction =
            (self.collision_modify_contacts_callback)(&filter_info, self.physics_collision_objects);
        let mut contact_is_pass_through = false;
        let mut dist: Real = 0.0;
        if let Some(contact) = context.manifold.find_deepest_contact() {
            dist = contact.dist;
        }
        if one_way_direction.body1 {
            let motion_len = body2.linvel().magnitude();
            let body_margin1 = one_way_direction.pixel_body1_margin;
            let max_allowed = motion_len
                * Real::max(body2.linvel().normalize().dot(&allowed_local_n1), 0.0)
                + body_margin1;
            contact_is_pass_through =
                body2.linvel().dot(&allowed_local_n1) <= DEFAULT_EPSILON || dist < -max_allowed;
        } else if one_way_direction.body2 {
            let motion_len = body1.linvel().magnitude();
            let body_margin2 = one_way_direction.pixel_body2_margin;
            let max_allowed = motion_len
                * Real::max(body1.linvel().normalize().dot(&allowed_local_n2), 0.0)
                + body_margin2;
            contact_is_pass_through =
                body1.linvel().dot(&allowed_local_n2) <= DEFAULT_EPSILON || dist < -max_allowed;
        }
        if contact_is_pass_through {
            context.solver_contacts.clear();
        }
    }
}
