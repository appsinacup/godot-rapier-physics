use rapier2d::prelude::*;
use crate::handle::*;
use crate::physics_world::*;
use crate::vector::Vector;

#[no_mangle]
pub extern "C" fn joint_create_revolute(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, anchor_1 : &Vector, anchor_2 : &Vector, angular_limit_lower: Real, angular_limit_upper: Real, angular_limit_enabled: bool, motor_target_velocity: Real, motor_enabled: bool) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let mut joint = RevoluteJointBuilder::new()
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y))
    .motor_model(MotorModel::AccelerationBased);
    if angular_limit_enabled {
        joint = joint.limits([angular_limit_lower, angular_limit_upper]);
    }
    if motor_enabled {
        joint = joint.motor_velocity(motor_target_velocity, 0.0);
    }
    
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}


#[no_mangle]
pub extern "C" fn joint_change_revolute_params(world_handle : Handle, joint_handle: Handle , angular_limit_lower: Real, angular_limit_upper: Real, angular_limit_enabled: bool, motor_target_velocity: Real, motor_enabled: bool){
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = physics_world.impulse_joint_set.get_mut(handle_to_joint_handle(joint_handle));
    assert!(joint.is_some());
    let joint = joint.unwrap().data.as_revolute_mut();
    assert!(joint.is_some());
    let mut joint = joint.unwrap();
    if motor_enabled {
        joint = joint.set_motor_velocity(motor_target_velocity, 0.0);
    }
    if angular_limit_enabled {
        joint.set_limits([angular_limit_lower, angular_limit_upper]);
    }
}

#[no_mangle]
pub extern "C" fn joint_create_prismatic(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, axis : &Vector, anchor_1 : &Vector, anchor_2 : &Vector, limits : &Vector) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = PrismaticJointBuilder::new(UnitVector::new_normalize(vector![axis.x, axis.y]))
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y))
	.limits([limits.x, limits.y]);
    
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}

#[no_mangle]
pub extern "C" fn joint_destroy(world_handle : Handle, joint_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

	return physics_world.remove_joint(joint_handle);
}
