use rapier2d::prelude::*;
use crate::handle::*;
use crate::user_data::*;

pub type CollisionFilterCallback = Option<extern "C" fn(world_handle : Handle, filter_info : &CollisionFilterInfo) -> bool>;

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

    fn modify_solver_contacts(&self, _context: &mut ContactModificationContext) {
        // TODO implement conveyer belt
        // for this we need to store the static object constant speed somewhere
        /*
        if context.rigid_body1.is_none() || context.rigid_body2.is_none() {
            return;
        }
		let rigid_body1 = context.bodies.get(context.rigid_body1.unwrap());
		let rigid_body2 = context.bodies.get(context.rigid_body1.unwrap());
        if rigid_body1.is_none() || rigid_body2.is_none() {
            return;
        }
        let mut rigid_body1 = rigid_body1.unwrap();
        let mut rigid_body2 = rigid_body2.unwrap();

        if rigid_body2.is_fixed() && !rigid_body1.is_fixed() {
            (rigid_body2, rigid_body1) = (rigid_body1, rigid_body2);
        }
        */
        // static and non static
        //if rigid_body1.is_fixed() && !rigid_body2.is_fixed() {
            //for solver_contact in &mut *context.solver_contacts {
                //solver_contact.tangent_velocity.x = 100.0;
            //}
        //}
    }
}
