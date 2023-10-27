use rapier2d::prelude::*;
use crate::handle::*;
use crate::vector::Vector;
use crate::user_data::UserData;
use crate::physics_world::*;
use crate::collider::*;

#[repr(C)]
pub enum BodyType {
    Dynamic,
    Kinematic,
    Static,
}

fn set_rigid_body_properties_internal(rigid_body : &mut RigidBody, pos : &Vector, rot : Real, wake_up : bool) {
    if rigid_body.is_dynamic() {
        rigid_body.set_position(Isometry::new(vector![pos.x, pos.y], rot), wake_up);
    } else {
        rigid_body.set_next_kinematic_position(Isometry::new(vector![pos.x, pos.y], rot));
    }
}

#[no_mangle]
pub extern "C" fn body_create(world_handle : Handle, pos : &Vector, rot : Real, user_data : &UserData, body_type: BodyType) -> Handle {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let mut rigid_body : RigidBody;
    match body_type {
        BodyType::Dynamic => {
            rigid_body = RigidBodyBuilder::dynamic().build();
        }
        BodyType::Kinematic => {
            rigid_body = RigidBodyBuilder::kinematic_position_based().build();
        }
        BodyType::Static => {
            rigid_body = RigidBodyBuilder::fixed().build();
        }
    }
	let activation = rigid_body.activation_mut();
	activation.time_until_sleep = physics_world.sleep_time_until_sleep;
    activation.linear_threshold = physics_world.sleep_linear_threshold;
    activation.angular_threshold = physics_world.sleep_angular_threshold;
    set_rigid_body_properties_internal(&mut rigid_body, pos, rot, true);
	rigid_body.user_data = user_data.get_data();
    let body_handle = physics_world.rigid_body_set.insert(rigid_body);
    return rigid_body_handle_to_handle(body_handle);
}

#[no_mangle]
pub extern "C" fn body_change_mode(world_handle : Handle, body_handle : Handle, body_type: BodyType, wakeup: bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let body: &mut RigidBody = body.unwrap();
    match body_type {
        BodyType::Dynamic => {
            body.set_body_type(RigidBodyType::Dynamic, wakeup);
        }
        BodyType::Kinematic => {
            body.set_body_type(RigidBodyType::KinematicPositionBased, wakeup);
        }
        BodyType::Static => {
            body.set_body_type(RigidBodyType::Fixed, wakeup);
        }
    }
}

#[no_mangle]
pub extern "C" fn body_destroy(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    physics_world.remove_rigid_body(body_handle);
}

#[no_mangle]
pub extern "C" fn body_get_position(world_handle : Handle, body_handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    let body_vector = body.unwrap().translation();
    return Vector { x : body_vector.x, y : body_vector.y };
}

#[no_mangle]
pub extern "C" fn body_get_angle(world_handle : Handle, body_handle : Handle) -> Real {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    return body.unwrap().rotation().angle();
}

#[no_mangle]
pub extern "C" fn body_set_transform(world_handle : Handle, body_handle : Handle, pos : &Vector, rot : Real, wake_up : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let body = body.unwrap();
    set_rigid_body_properties_internal(body, pos, rot, wake_up);
}

#[no_mangle]
pub extern "C" fn body_get_linear_velocity(world_handle : Handle, body_handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    let body = body.unwrap();
    let body_vel = body.linvel();
    return Vector { x : body_vel.x, y : body_vel.y };
}

#[no_mangle]
pub extern "C" fn body_set_linear_velocity(world_handle : Handle, body_handle : Handle, vel : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_linvel(vector![vel.x, vel.y], true);
}

#[no_mangle]
pub extern "C" fn body_update_material(world_handle : Handle, body_handle : Handle, mat : &Material) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    for collider in body.unwrap().colliders() {
        if let Some(col) = physics_world.collider_set.get_mut(*collider) {
            col.set_friction(mat.friction);
            col.set_restitution(mat.restitution)
        }
    }
}

#[no_mangle]
pub extern "C" fn body_get_angular_velocity(world_handle : Handle, body_handle : Handle) -> Real {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    return body.unwrap().angvel();
}

#[no_mangle]
pub extern "C" fn body_set_angular_velocity(world_handle : Handle, body_handle : Handle, vel : Real) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_angvel(vel, true);
}

#[no_mangle]
pub extern "C" fn body_set_linear_damping(world_handle : Handle, body_handle : Handle, linear_damping : Real) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_linear_damping(linear_damping);
}

#[no_mangle]
pub extern "C" fn body_set_angular_damping(world_handle : Handle, body_handle : Handle, angular_damping : Real) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_angular_damping(angular_damping);
}

#[no_mangle]
pub extern "C" fn body_set_gravity_scale(world_handle : Handle, body_handle : Handle, gravity_scale : Real, wake_up : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().set_gravity_scale(gravity_scale, wake_up);
}

#[no_mangle]
pub extern "C" fn body_set_can_sleep(world_handle : Handle, body_handle : Handle, can_sleep:  bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let body = body.unwrap();
    
    if can_sleep && body.activation().angular_threshold == -1.0 {
		let activation = body.activation_mut();
        activation.angular_threshold = physics_world.sleep_angular_threshold;
        activation.linear_threshold = physics_world.sleep_linear_threshold;
    } else if !can_sleep && body.activation().angular_threshold != -1.0 {
		let activation = body.activation_mut();
        activation.angular_threshold = -1.0;
        activation.linear_threshold = -1.0;
    }

    // TODO: Check if is requiered
    if !can_sleep && body.is_sleeping() {
        body.wake_up(true);
    }
}

#[no_mangle]
pub extern "C" fn body_is_ccd_enabled(world_handle : Handle, body_handle : Handle) -> bool {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().is_ccd_enabled()
}

#[no_mangle]
pub extern "C" fn body_set_ccd_enabled(world_handle : Handle, body_handle : Handle, enable: bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().enable_ccd(enable);
}

#[no_mangle]
pub extern "C" fn body_set_mass_properties(world_handle : Handle, body_handle : Handle, mass : Real, inertia : Real, local_com : &Vector, wake_up : bool, force_update : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
	let body_ref = body.unwrap();
	body_ref.set_additional_mass_properties(MassProperties::new(point![local_com.x, local_com.y], mass, inertia), wake_up);
    if force_update {
        body_ref.recompute_mass_properties_from_colliders(&physics_world.collider_set);
    }
}

#[no_mangle]
pub extern "C" fn body_add_force(world_handle : Handle, body_handle : Handle, force : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().add_force(vector!(force.x, force.y), true);
}

#[no_mangle]
pub extern "C" fn body_add_force_at_point(world_handle : Handle, body_handle : Handle, force : &Vector, point : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let mut local_point = point![point.x, point.y];
    let body = body.unwrap();
    local_point += body.center_of_mass().coords;
    body.add_force_at_point(vector!(force.x, force.y), local_point, true);
}

#[no_mangle]
pub extern "C" fn body_add_torque(world_handle : Handle, body_handle : Handle, torque: Real) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().add_torque(torque, true);
}

#[no_mangle]
pub extern "C" fn body_apply_impulse(world_handle : Handle, body_handle : Handle, impulse : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().apply_impulse(vector!(impulse.x, impulse.y), true);
}

#[no_mangle]
pub extern "C" fn body_apply_impulse_at_point(world_handle : Handle, body_handle : Handle, impulse : &Vector, point : &Vector) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let mut local_point = point![point.x, point.y];
    let body = body.unwrap();
    local_point += body.center_of_mass().coords;
    body.apply_impulse_at_point(vector!(impulse.x, impulse.y), local_point, true);
}

#[no_mangle]
pub extern "C" fn body_get_constant_force(world_handle : Handle, body_handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    let constant_force = body.unwrap().user_force();
    return Vector { x : constant_force.x, y : constant_force.y };
}

#[no_mangle]
pub extern "C" fn body_get_constant_torque(world_handle : Handle, body_handle : Handle) -> Real {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().user_torque()
}

#[no_mangle]
pub extern "C" fn body_apply_torque_impulse(world_handle : Handle, body_handle : Handle, torque_impulse : Real) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().apply_torque_impulse(torque_impulse, true);
}

#[no_mangle]
pub extern "C" fn body_reset_torques(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().reset_torques(false);
}

#[no_mangle]
pub extern "C" fn body_reset_forces(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().reset_forces(false);
}

#[no_mangle]
pub extern "C" fn body_wake_up(world_handle : Handle, body_handle : Handle, strong : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    // TODO: check if this is OK, at the startup call to RapierPhysicsServer2D::free where body is not some
    assert!(body.is_some());
    let body = body.unwrap();
    if body.is_sleeping() {
        body.wake_up(strong);
    }
}

#[no_mangle]
pub extern "C" fn body_force_sleep(world_handle : Handle, body_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world.rigid_body_set.get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().sleep();
}
