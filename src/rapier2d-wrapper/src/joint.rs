use rapier2d::prelude::*;
use crate::convert::*;
use crate::handle::*;
use crate::physics_world::*;
use crate::vector::Vector;

#[no_mangle]
pub extern "C" fn joint_create_revolute(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, pixel_anchor_1 : &Vector, pixel_anchor_2 : &Vector, angular_limit_lower: Real, angular_limit_upper: Real, angular_limit_enabled: bool, pixel_motor_target_velocity: Real, motor_enabled: bool, disable_collision: bool) -> Handle {
    let anchor_1 = &vector_pixels_to_meters(pixel_anchor_1);
    let anchor_2 = &vector_pixels_to_meters(pixel_anchor_2);

    let motor_target_velocity = pixels_to_meters(pixel_motor_target_velocity);
    
    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let mut joint = RevoluteJointBuilder::new()
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y))
    .motor_model(MotorModel::ForceBased)
    .contacts_enabled(!disable_collision);
    if angular_limit_enabled {
        joint = joint.limits([angular_limit_lower, angular_limit_upper]);
    }
    if motor_enabled {
        joint = joint.motor_velocity(motor_target_velocity, 0.0);
    }
    
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}


#[no_mangle]
pub extern "C" fn joint_change_revolute_params(world_handle : Handle, joint_handle: Handle , angular_limit_lower: Real, angular_limit_upper: Real, angular_limit_enabled: bool, pixel_motor_target_velocity: Real, motor_enabled: bool){
    let motor_target_velocity = pixels_to_meters(pixel_motor_target_velocity);

    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = physics_world.impulse_joint_set.get_mut(handle_to_joint_handle(joint_handle));
    assert!(joint.is_some());
    let joint = joint.unwrap().data.as_revolute_mut();
    assert!(joint.is_some());
    let joint = joint.unwrap();
    if motor_enabled {
        joint.set_motor_velocity(motor_target_velocity, 0.0)
        .set_motor_max_force(Real::MAX);
    } else {
        joint.set_motor_velocity(0.0, 0.0)
        .set_motor_max_force(0.0);
    }
    if angular_limit_enabled {
        joint.set_limits([angular_limit_lower, angular_limit_upper]);
    }
}

#[no_mangle]
pub extern "C" fn joint_create_prismatic(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, axis : &Vector, pixel_anchor_1 : &Vector, pixel_anchor_2 : &Vector, pixel_limits : &Vector, disable_collision: bool) -> Handle {
    let anchor_1 = &vector_pixels_to_meters(pixel_anchor_1);
    let anchor_2 = &vector_pixels_to_meters(pixel_anchor_2);
    let limits = &vector_pixels_to_meters(pixel_limits);

    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = PrismaticJointBuilder::new(UnitVector::new_normalize(vector![axis.x, axis.y]))
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y))
	.limits([limits.x, limits.y])
    .contacts_enabled(!disable_collision);
    
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}

#[no_mangle]
pub extern "C" fn joint_create_spring(world_handle : Handle, body_handle_1 : Handle, body_handle_2 : Handle, pixel_anchor_1 : &Vector, pixel_anchor_2 : &Vector, stiffness: Real, damping: Real, pixel_rest_length: Real, disable_collision: bool) -> Handle {
    let anchor_1 = &vector_pixels_to_meters(pixel_anchor_1);
    let anchor_2 = &vector_pixels_to_meters(pixel_anchor_2);
    let rest_length = pixels_to_meters(pixel_rest_length);
    
    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = SpringJointBuilder::new(rest_length, stiffness, damping)
    .local_anchor1(point!(anchor_1.x, anchor_1.y))
    .local_anchor2(point!(anchor_2.x, anchor_2.y))
    .contacts_enabled(!disable_collision);
	return physics_world.insert_joint(body_handle_1, body_handle_2, joint);
}

#[no_mangle]
pub extern "C" fn joint_change_spring_params(world_handle : Handle, joint_handle: Handle , stiffness: Real, damping: Real, pixel_rest_length: Real){
    let rest_length = pixels_to_meters(pixel_rest_length);

    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = physics_world.impulse_joint_set.get_mut(handle_to_joint_handle(joint_handle));
    assert!(joint.is_some());
    let spring_joint = joint.unwrap();
    spring_joint.data.set_motor_position(JointAxis::X, rest_length, stiffness, damping);
}

#[no_mangle]
pub extern "C" fn joint_destroy(world_handle : Handle, joint_handle : Handle) {
    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

	return physics_world.remove_joint(joint_handle);
}

#[no_mangle]
pub extern "C" fn joint_change_disable_collision(world_handle : Handle, joint_handle: Handle , disable_collision: bool){
    let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let joint = physics_world.impulse_joint_set.get_mut(handle_to_joint_handle(joint_handle));
    assert!(joint.is_some());
    joint.unwrap().data.set_contacts_enabled(!disable_collision);
}