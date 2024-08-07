use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsCollisionObjects;
#[derive(Default)]
pub struct OneWayDirection {
    pub body1: bool,
    pub body2: bool,
    pub pixel_body1_margin: Real,
    pub pixel_body2_margin: Real,
    pub previous_linear_velocity1: Vector<Real>,
    pub previous_linear_velocity2: Vector<Real>,
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
    pub collision_modify_contacts_callback: &'a CollisionModifyContactsCallback,
    pub physics_collision_objects: &'a PhysicsCollisionObjects,
    pub last_step: Real,
    pub ghost_collision_distance: Real,
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
        let mut rigid_body_1_linvel = one_way_direction.previous_linear_velocity1;
        let mut rigid_body_2_linvel = one_way_direction.previous_linear_velocity2;
        if rigid_body_1_linvel.norm() == 0.0 {
            rigid_body_1_linvel = *body1.linvel();
        }
        if rigid_body_2_linvel.norm() == 0.0 {
            rigid_body_2_linvel = *body2.linvel();
        }
        // ghost collisions
        if body1.is_dynamic() && !body2.is_dynamic() {
            let normal = *context.normal;
            if rigid_body_1_linvel.norm() == 0.0 {
                return;
            }
            let normal_dot_velocity = normal.dot(&rigid_body_1_linvel.normalize());
            let velocity_magnitude = rigid_body_1_linvel.magnitude() * self.last_step;
            let length_along_normal = velocity_magnitude * Real::max(normal_dot_velocity, 0.0);
            if normal_dot_velocity >= -DEFAULT_EPSILON {
                context.solver_contacts.retain(|contact| {
                    let dist = -contact.dist;
                    let diff = dist - length_along_normal;
                    if diff < 0.5 && dist.abs() < self.ghost_collision_distance {
                        return false;
                    }
                    true
                });
            }
        } else if body2.is_dynamic() && !body1.is_dynamic() {
            let normal = -*context.normal;
            if rigid_body_2_linvel.norm() == 0.0 {
                return;
            }
            let normal_dot_velocity = normal.dot(&rigid_body_2_linvel.normalize());
            let velocity_magnitude = rigid_body_2_linvel.magnitude() * self.last_step;
            let length_along_normal = velocity_magnitude * Real::max(normal_dot_velocity, 0.0);
            if normal_dot_velocity >= -DEFAULT_EPSILON {
                context.solver_contacts.retain(|contact| {
                    let dist = -contact.dist;
                    let diff = dist - length_along_normal;
                    if diff < 0.5 && dist.abs() < self.ghost_collision_distance {
                        return false;
                    }
                    true
                });
            }
        }
        let mut dist = 0.0;
        if let Some(deepest) = context.manifold.find_deepest_contact() {
            dist = deepest.dist;
        }
        // one way collision
        if one_way_direction.body1 {
            let body_margin1 = one_way_direction.pixel_body1_margin;
            let motion_len = rigid_body_2_linvel.magnitude();
            let velocity_dot1 = rigid_body_2_linvel.normalize().dot(&allowed_local_n1);
            let length_along_normal = motion_len * Real::max(velocity_dot1, 0.0) * self.last_step;
            contact_is_pass_through |= velocity_dot1 <= DEFAULT_EPSILON
                && dist - length_along_normal > body_margin1
                && length_along_normal > DEFAULT_EPSILON;
        } else if one_way_direction.body2 {
            let velocity_dot2 = rigid_body_1_linvel.normalize().dot(&allowed_local_n2);
            let motion_len = rigid_body_1_linvel.magnitude();
            let body_margin2 = one_way_direction.pixel_body2_margin;
            let length_along_normal = motion_len * Real::max(velocity_dot2, 0.0) * self.last_step;
            contact_is_pass_through |= velocity_dot2 <= DEFAULT_EPSILON
                && dist - length_along_normal > body_margin2
                && length_along_normal > DEFAULT_EPSILON;
        }
        if contact_is_pass_through {
            context.solver_contacts.clear();
        }
    }
}
