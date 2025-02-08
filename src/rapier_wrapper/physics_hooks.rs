use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsCollisionObjects;
use crate::servers::rapier_physics_singleton::PhysicsIds;
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
    physics_ids: &PhysicsIds,
) -> bool;
pub type CollisionModifyContactsCallback = fn(
    filter_info: &CollisionFilterInfo,
    physics_collision_objects: &PhysicsCollisionObjects,
    physics_ids: &PhysicsIds,
) -> OneWayDirection;
pub struct CollisionFilterInfo {
    pub user_data1: UserData,
    pub user_data2: UserData,
}
pub struct PhysicsHooksCollisionFilter<'a> {
    pub collision_filter_body_callback: &'a CollisionFilterCallback,
    pub collision_modify_contacts_callback: &'a CollisionModifyContactsCallback,
    pub physics_collision_objects: &'a PhysicsCollisionObjects,
    pub physics_ids: &'a PhysicsIds,
    pub last_step: Real,
    pub ghost_collision_distance: Real,
}
pub fn update_as_oneway_platform(
    context: &mut ContactModificationContext,
    local_normal: &Vector<Real>,
    allowed_local_normal: &Vector<Real>,
) {
    const CONTACT_CONFIGURATION_UNKNOWN: u32 = 0;
    const CONTACT_CURRENTLY_ALLOWED: u32 = 1;
    const CONTACT_CURRENTLY_FORBIDDEN: u32 = 2;
    // Test the allowed normal with the local-space contact normal that
    // points towards the exterior of context.collider1.
    let contact_is_ok = local_normal.dot(allowed_local_normal) < 0.00001;
    match *context.user_data {
        CONTACT_CONFIGURATION_UNKNOWN => {
            if contact_is_ok {
                // The contact is close enough to the allowed normal.
                *context.user_data = CONTACT_CURRENTLY_ALLOWED;
            } else {
                // The contact normal isn't close enough to the allowed
                // normal, so remove all the contacts and mark further contacts
                // as forbidden.
                context.solver_contacts.clear();
                // NOTE: in some very rare cases `local_n1` will be
                // zero if the objects are exactly touching at one point.
                // So in this case we can't really conclude.
                // If the norm is non-zero, then we can tell we need to forbid
                // further contacts. Otherwise we have to wait for the next frame.
                if local_normal.norm_squared() > 0.1 {
                    *context.user_data = CONTACT_CURRENTLY_FORBIDDEN;
                }
            }
        }
        CONTACT_CURRENTLY_FORBIDDEN => {
            // Contacts are forbidden so we need to continue forbidding contacts
            // until all the contacts are non-penetrating again. In that case, if
            // the contacts are OK with respect to the contact normal, then we can
            // mark them as allowed.
            if contact_is_ok && context.solver_contacts.iter().all(|c| c.dist > 0.0) {
                *context.user_data = CONTACT_CURRENTLY_ALLOWED;
            } else {
                // Discard all the contacts.
                context.solver_contacts.clear();
            }
        }
        CONTACT_CURRENTLY_ALLOWED => {
            // We allow all the contacts right now. The configuration becomes
            // uncertain again when the contact manifold no longer contains any contact.
            if context.solver_contacts.is_empty() {
                *context.user_data = CONTACT_CONFIGURATION_UNKNOWN;
            }
        }
        _ => unreachable!(),
    }
}
impl PhysicsHooks for PhysicsHooksCollisionFilter<'_> {
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
        if !(self.collision_filter_body_callback)(
            &filter_info,
            self.physics_collision_objects,
            self.physics_ids,
        ) {
            return None;
        }
        result
    }

    fn filter_intersection_pair(&self, _context: &PairFilterContext) -> bool {
        true
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let Some(collider1) = context.colliders.get(context.collider1) else {
            return;
        };
        let Some(collider2) = context.colliders.get(context.collider2) else {
            return;
        };
        let Some(rididbody1_handle) = context.rigid_body1 else {
            return;
        };
        let Some(rididbody2_handle) = context.rigid_body2 else {
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
        let one_way_direction = (self.collision_modify_contacts_callback)(
            &filter_info,
            self.physics_collision_objects,
            self.physics_ids,
        );
        if one_way_direction.body1 {
            update_as_oneway_platform(context, &(context.normal.clone()), &allowed_local_n1);
        }
        if one_way_direction.body2 {
            update_as_oneway_platform(context, &-*context.normal, &allowed_local_n2);
        }
        let contact_is_pass_through = false;
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
        if contact_is_pass_through {
            context.solver_contacts.clear();
        }
    }
}
