use rapier2d::prelude::*;
use crate::handle::*;
use crate::user_data::*;

#[repr(C)]
pub struct OneWayDirection {
    pub body1 : bool,
    pub body2 : bool,
}

pub type CollisionFilterCallback = Option<extern "C" fn(world_handle : Handle, filter_info : &CollisionFilterInfo) -> bool>;
pub type CollisionModifyContactsCallback = Option<extern "C" fn(world_handle : Handle, filter_info : &CollisionFilterInfo) -> OneWayDirection>;

#[repr(C)]
pub struct CollisionFilterInfo {
    pub user_data1: UserData,
    pub user_data2: UserData,
}

impl CollisionFilterInfo {
    pub fn new() -> CollisionFilterInfo {
        CollisionFilterInfo {
			user_data1: invalid_user_data(),
			user_data2: invalid_user_data(),
        }
    }
}

pub struct PhysicsHooksCollisionFilter<'a> {
	pub world_handle : Handle,
	pub collision_filter_body_callback : &'a CollisionFilterCallback,
	pub collision_filter_sensor_callback : &'a CollisionFilterCallback,
	pub collision_modify_contacts_callback : &'a CollisionModifyContactsCallback,
}

impl<'a> PhysicsHooks for PhysicsHooksCollisionFilter<'a> {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
		if self.collision_filter_body_callback.is_some() {
			let callback = self.collision_filter_body_callback.unwrap();

			let user_data1 = context.colliders[context.collider1].user_data;
			let user_data2 = context.colliders[context.collider2].user_data;
			
			let mut filter_info = CollisionFilterInfo::new();
			filter_info.user_data1 = UserData::new(user_data1);
			filter_info.user_data2 = UserData::new(user_data2);
			
			// Handle contact filtering for rigid bodies
			if !callback(self.world_handle, &filter_info) {
				return None;
			}
		}
		
		return Some(SolverFlags::COMPUTE_IMPULSES);
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        if self.collision_filter_sensor_callback.is_some() {
			let callback = self.collision_filter_sensor_callback.unwrap();

			let user_data1 = context.colliders[context.collider1].user_data;
			let user_data2 = context.colliders[context.collider2].user_data;

			let mut filter_info = CollisionFilterInfo::new();
			filter_info.user_data1 = UserData::new(user_data1);
			filter_info.user_data2 = UserData::new(user_data2);
			
			// Handle intersection filtering for sensors
			return callback(self.world_handle, &filter_info);
		}

		return true;
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        if self.collision_modify_contacts_callback.is_some() {
			let callback = self.collision_modify_contacts_callback.unwrap();

            let collider_1 = &context.colliders[context.collider1];
            let collider_2 = &context.colliders[context.collider2];
            let body1 = &context.bodies[context.rigid_body1.unwrap()];
            let body2 = &context.bodies[context.rigid_body2.unwrap()];

			let mut filter_info = CollisionFilterInfo::new();
			filter_info.user_data1 = UserData::new(collider_1.user_data);
			filter_info.user_data2 = UserData::new(collider_2.user_data);
            let allowed_local_n1 = collider_1.position().rotation * Vector::y();
            let allowed_local_n2 = collider_2.position().rotation * Vector::y();
			let one_way_direction = callback(self.world_handle, &filter_info);
            let mut contact_is_pass_through = false;
            if one_way_direction.body1 {
                contact_is_pass_through = body2.linvel().normalize().dot(&allowed_local_n1) <= -DEFAULT_EPSILON * 10.0;
            } else if one_way_direction.body2 {
                contact_is_pass_through = body1.linvel().normalize().dot(&allowed_local_n2) <= -DEFAULT_EPSILON * 10.0;
            }
            if contact_is_pass_through {
                context.solver_contacts.clear();
            }
        }
    }
}
