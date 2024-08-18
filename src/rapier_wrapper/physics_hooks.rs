use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsCollisionObjects;
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
    pub collision_modify_contacts_callback: &'a CollisionModifyContactsCallback,
    pub physics_collision_objects: &'a PhysicsCollisionObjects,
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
        let Some(collider1) = context.colliders.get(context.collider1) else {
            return;
        };
        let Some(collider2) = context.colliders.get(context.collider2) else {
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
        if one_way_direction.body1 {
            update_as_oneway_platform(context, &(context.normal.clone()), &allowed_local_n1);
        }
        if one_way_direction.body2 {
            update_as_oneway_platform(context, &-*context.normal, &allowed_local_n2);
        }
    }
}
