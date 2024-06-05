use crate::rapier2d::convert::*;
use crate::rapier2d::user_data::*;
use rapier2d::prelude::*;

pub struct OneWayDirection {
    pub body1: bool,
    pub body2: bool,
    pub pixel_body1_margin: Real,
    pub pixel_body2_margin: Real,
    pub last_timestep: Real,
}

impl OneWayDirection {
    pub fn default() -> OneWayDirection {
        OneWayDirection {
            body1: false,
            body2: false,
            pixel_body1_margin: 0.0,
            pixel_body2_margin: 0.0,
            last_timestep: 0.001,
        }
    }
}

pub type CollisionFilterCallback = fn(filter_info: &CollisionFilterInfo) -> bool;
pub type CollisionModifyContactsCallback = fn(filter_info: &CollisionFilterInfo) -> OneWayDirection;

pub struct CollisionFilterInfo {
    pub user_data1: UserData,
    pub user_data2: UserData,
}

impl CollisionFilterInfo {
    pub fn new() -> CollisionFilterInfo {
        CollisionFilterInfo {
            user_data1: UserData::invalid_user_data(),
            user_data2: UserData::invalid_user_data(),
        }
    }
}

pub struct PhysicsHooksCollisionFilter<'a> {
    pub collision_filter_body_callback: &'a CollisionFilterCallback,
    pub collision_filter_sensor_callback: &'a CollisionFilterCallback,
    pub collision_modify_contacts_callback: &'a CollisionModifyContactsCallback,
}

impl<'a> PhysicsHooks for PhysicsHooksCollisionFilter<'a> {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let user_data1 = context.colliders[context.collider1].user_data;
        let user_data2 = context.colliders[context.collider2].user_data;

        let mut filter_info = CollisionFilterInfo::new();
        filter_info.user_data1 = UserData::new(user_data1);
        filter_info.user_data2 = UserData::new(user_data2);

        // Handle contact filtering for rigid bodies
        if !(self.collision_filter_body_callback)(&filter_info) {
            return None;
        }

        Some(SolverFlags::COMPUTE_IMPULSES)
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        let user_data1 = context.colliders[context.collider1].user_data;
        let user_data2 = context.colliders[context.collider2].user_data;

        let mut filter_info = CollisionFilterInfo::new();
        filter_info.user_data1 = UserData::new(user_data1);
        filter_info.user_data2 = UserData::new(user_data2);

        // Handle intersection filtering for sensors
        (self.collision_filter_sensor_callback)(&filter_info)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let collider_1 = &context.colliders[context.collider1];
        let collider_2 = &context.colliders[context.collider2];
        let body1 = &context.bodies[context.rigid_body1.unwrap()];
        let body2 = &context.bodies[context.rigid_body2.unwrap()];
        let mut filter_info = CollisionFilterInfo::new();
        filter_info.user_data1 = UserData::new(collider_1.user_data);
        filter_info.user_data2 = UserData::new(collider_2.user_data);
        let allowed_local_n1 = collider_1.position().rotation * Vector::y();
        let allowed_local_n2 = collider_2.position().rotation * Vector::y();
        let one_way_direction = (self.collision_modify_contacts_callback)(&filter_info);
        let mut contact_is_pass_through = false;
        let mut dist: Real = 0.0;
        if let Some(contact) = context.manifold.find_deepest_contact() {
            dist = contact.dist;
        }

        if one_way_direction.body1 {
            let motion_len = body2.linvel().magnitude();
            let body_margin1 = pixels_to_meters(one_way_direction.pixel_body1_margin);
            let max_allowed = motion_len
                * Real::max(body2.linvel().normalize().dot(&allowed_local_n1), 0.0)
                + body_margin1;
            contact_is_pass_through =
                body2.linvel().dot(&allowed_local_n1) <= DEFAULT_EPSILON || dist < -max_allowed;
        } else if one_way_direction.body2 {
            let motion_len = body1.linvel().magnitude();
            let body_margin2 = pixels_to_meters(one_way_direction.pixel_body2_margin);
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
