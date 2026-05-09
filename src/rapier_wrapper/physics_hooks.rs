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
    pub previous_linear_velocity1: Vector,
    pub previous_linear_velocity2: Vector,
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
const GODOT_ONE_WAY_DOT_EPSILON: Real = 1.0e-5_f32;
fn update_as_godot_one_way_platform(
    context: &mut ContactModificationContext,
    shape_rel_dir: Vector,
    valid_dir: Vector,
) {
    const CONTACT_CONFIGURATION_UNKNOWN: u32 = 0;
    const CONTACT_CURRENTLY_ALLOWED: u32 = 1;
    const CONTACT_CURRENTLY_FORBIDDEN: u32 = 2;
    let shape_rel_length_sq = shape_rel_dir.length_squared();
    let contact_is_ok = shape_rel_length_sq > GODOT_ONE_WAY_DOT_EPSILON
        && shape_rel_dir.normalize().dot(valid_dir) > GODOT_ONE_WAY_DOT_EPSILON;
    match *context.user_data {
        CONTACT_CONFIGURATION_UNKNOWN => {
            if contact_is_ok {
                *context.user_data = CONTACT_CURRENTLY_ALLOWED;
            } else {
                context.solver_contacts.clear();
                if shape_rel_length_sq > 0.1 {
                    *context.user_data = CONTACT_CURRENTLY_FORBIDDEN;
                }
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
        let one_way_direction = (self.collision_modify_contacts_callback)(
            &filter_info,
            self.physics_collision_objects,
            self.physics_ids,
        );
        if one_way_direction.body1 {
            let valid_dir = collider1.position().rotation * -Vector::Y;
            let shape_rel_dir = collider2.position().translation - collider1.position().translation;
            update_as_godot_one_way_platform(context, shape_rel_dir, valid_dir);
        } else if one_way_direction.body2 {
            let valid_dir = collider2.position().rotation * -Vector::Y;
            let shape_rel_dir = collider1.position().translation - collider2.position().translation;
            update_as_godot_one_way_platform(context, shape_rel_dir, valid_dir);
        }
        let contact_is_pass_through = false;
        let mut rigid_body_1_linvel = one_way_direction.previous_linear_velocity1;
        let mut rigid_body_2_linvel = one_way_direction.previous_linear_velocity2;
        if rigid_body_1_linvel.length() == 0.0 {
            rigid_body_1_linvel = body1.linvel();
        }
        if rigid_body_2_linvel.length() == 0.0 {
            rigid_body_2_linvel = body2.linvel();
        }
        // ghost collisions
        if body1.is_dynamic() && !body2.is_dynamic() {
            let normal = *context.normal;
            if rigid_body_1_linvel.length() == 0.0 {
                return;
            }
            let normal_dot_velocity = normal.dot(rigid_body_1_linvel.normalize());
            let velocity_magnitude = rigid_body_1_linvel.length() * self.last_step;
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
            if rigid_body_2_linvel.length() == 0.0 {
                return;
            }
            let normal_dot_velocity = normal.dot(rigid_body_2_linvel.normalize());
            let velocity_magnitude = rigid_body_2_linvel.length() * self.last_step;
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
