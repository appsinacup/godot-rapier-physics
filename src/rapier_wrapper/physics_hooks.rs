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
    pub body1_direction: Vector,
    pub body2_direction: Vector,
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
}
// Godot's CMP_EPSILON, the tolerance its own one-way checks compare dot products against. A
// contact along a flat face is exactly perpendicular to the one-way direction, so the guard band
// around zero decides those cases and has to be the same width as Godot's.
pub const GODOT_ONE_WAY_DOT_EPSILON: Real = 1.0e-5_f32;
// `contact_dir` is the unit contact normal oriented away from the one-way shape, matching the
// normal Godot's `GodotBodyPair2D::setup` tests against the shape's one-way direction.
fn update_as_godot_one_way_platform(
    context: &mut ContactModificationContext,
    contact_dir: Vector,
    valid_dir: Vector,
) {
    const CONTACT_CONFIGURATION_UNKNOWN: u32 = 0;
    const CONTACT_CURRENTLY_ALLOWED: u32 = 1;
    const CONTACT_CURRENTLY_FORBIDDEN: u32 = 2;
    let contact_is_ok = contact_dir.dot(valid_dir) > GODOT_ONE_WAY_DOT_EPSILON;
    match *context.user_data {
        CONTACT_CONFIGURATION_UNKNOWN => {
            if contact_is_ok {
                *context.user_data = CONTACT_CURRENTLY_ALLOWED;
            } else {
                context.solver_contacts.clear();
                *context.user_data = CONTACT_CURRENTLY_FORBIDDEN;
            }
        }
        CONTACT_CURRENTLY_FORBIDDEN => {
            if contact_is_ok && context.solver_contacts.iter().all(|c| c.dist > 0.0) {
                *context.user_data = CONTACT_CURRENTLY_ALLOWED;
            } else {
                context.solver_contacts.clear();
            }
        }
        CONTACT_CURRENTLY_ALLOWED => {
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
        let filter_info = CollisionFilterInfo {
            user_data1: UserData::new(collider1.user_data),
            user_data2: UserData::new(collider2.user_data),
        };
        let one_way_direction = if filter_info.user_data1.needs_contact_callback()
            || filter_info.user_data2.needs_contact_callback()
        {
            (self.collision_modify_contacts_callback)(
                &filter_info,
                self.physics_collision_objects,
                self.physics_ids,
            )
        } else {
            OneWayDirection::default()
        };
        // `context.normal` points from collider1 towards collider2, so it has to be flipped when
        // collider1 is the one-way shape for the dot product to keep Godot's meaning.
        if one_way_direction.body1 {
            let valid_dir = collider1.position().rotation * one_way_direction.body1_direction;
            update_as_godot_one_way_platform(context, -*context.normal, valid_dir);
        } else if one_way_direction.body2 {
            let valid_dir = collider2.position().rotation * one_way_direction.body2_direction;
            update_as_godot_one_way_platform(context, *context.normal, valid_dir);
        }
    }
}
