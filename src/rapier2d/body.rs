use crate::rapier2d::collider::*;
use crate::rapier2d::convert::*;
use crate::rapier2d::handle::*;
use crate::rapier2d::physics_world::*;
use crate::rapier2d::user_data::UserData;
use crate::rapier2d::vector::Vector;
use rapier2d::prelude::*;

pub enum BodyType {
    Dynamic,
    Kinematic,
    Static,
}

fn set_rigid_body_properties_internal(
    rigid_body: &mut RigidBody,
    pixel_pos: &Vector,
    rot: Real,
    wake_up: bool,
) {
    let pos = &vector_pixels_to_meters(pixel_pos);

    if !rigid_body.is_kinematic() {
        rigid_body.set_position(Isometry::new(vector![pos.x, pos.y], rot), wake_up);
    } else {
        rigid_body.set_next_kinematic_position(Isometry::new(vector![pos.x, pos.y], rot));
    }
}

pub fn body_create(
    world_handle: Handle,
    pixel_pos: &Vector,
    rot: Real,
    user_data: &UserData,
    body_type: BodyType,
) -> Handle {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let mut rigid_body: RigidBody;
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
    // let default values better
    set_rigid_body_properties_internal(&mut rigid_body, pixel_pos, rot, true);
    rigid_body.user_data = user_data.get_data();
    let body_handle = physics_world
        .physics_objects
        .rigid_body_set
        .insert(rigid_body);
    rigid_body_handle_to_handle(body_handle)
}

pub fn body_change_mode(
    world_handle: Handle,
    body_handle: Handle,
    body_type: BodyType,
    wakeup: bool,
) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
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
}

pub fn body_destroy(world_handle: Handle, body_handle: Handle) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    physics_world.remove_rigid_body(body_handle);
}

pub fn body_get_position(world_handle: Handle, body_handle: Handle) -> Vector {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get(rigid_body_handle) {
        let body_vector = body.translation();
        return vector_meters_to_pixels(&Vector {
            x: body_vector.x,
            y: body_vector.y,
        })
    }
    vector_meters_to_pixels(&Vector::default())
}

pub fn body_get_angle(world_handle: Handle, body_handle: Handle) -> Real {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get(rigid_body_handle) {
        return body.rotation().angle();
    }
    0.0
}

pub fn body_set_transform(
    world_handle: Handle,
    body_handle: Handle,
    pixel_pos: &Vector,
    rot: Real,
    wake_up: bool,
) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        set_rigid_body_properties_internal(body, pixel_pos, rot, wake_up);
    }
}

pub fn body_get_linear_velocity(world_handle: Handle, body_handle: Handle) -> Vector {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get(rigid_body_handle) {
        let body_vel = body.linvel();
        return vector_meters_to_pixels(&Vector {
            x: body_vel.x,
            y: body_vel.y,
        });
    }
    vector_meters_to_pixels(&Vector::default())
}

pub fn body_set_linear_velocity(world_handle: Handle, body_handle: Handle, pixel_vel: &Vector) {
    let vel = &vector_pixels_to_meters(pixel_vel);

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.set_linvel(vector![vel.x, vel.y], true);
    }
}

pub fn body_update_material(world_handle: Handle, body_handle: Handle, mat: &Material) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        for collider in body.colliders() {
            if let Some(col) = physics_world.physics_objects.collider_set.get_mut(*collider)
            {
                // TODO update when https://github.com/dimforge/rapier/issues/622 is fixed
                if mat.friction >= 0.0 {
                    col.set_friction(mat.friction);
                }
                if mat.restitution >= 0.0 {
                    col.set_restitution(mat.restitution);
                }
                if mat.contact_skin >= 0.0 {
                    col.set_contact_skin(mat.contact_skin);
                }
            }
        }
    }
}

pub fn body_get_angular_velocity(world_handle: Handle, body_handle: Handle) -> Real {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get(rigid_body_handle) {
        return body.angvel();
    }
    0.0
}

pub fn body_set_angular_velocity(world_handle: Handle, body_handle: Handle, vel: Real) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.set_angvel(vel, true);
    }
}

pub fn body_set_linear_damping(world_handle: Handle, body_handle: Handle, linear_damping: Real) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.set_linear_damping(linear_damping);
    }
}

pub fn body_set_angular_damping(world_handle: Handle, body_handle: Handle, angular_damping: Real) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.set_angular_damping(angular_damping);
    }
}

pub fn body_set_gravity_scale(
    world_handle: Handle,
    body_handle: Handle,
    gravity_scale: Real,
    wake_up: bool,
) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.set_gravity_scale(gravity_scale, wake_up);
    }
}

pub fn body_set_can_sleep(world_handle: Handle, body_handle: Handle, can_sleep: bool) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        if !can_sleep && body.activation().angular_threshold != -1.0 {
            let activation = body.activation_mut();
            activation.angular_threshold = -1.0;
            activation.normalized_linear_threshold = -1.0;
        }
        // TODO: Check if is requiered
        if !can_sleep && body.is_sleeping() {
            body.wake_up(true);
        }
    }
}

pub fn body_set_ccd_enabled(world_handle: Handle, body_handle: Handle, enable: bool) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.enable_ccd(enable);
    }
}

pub fn body_set_mass_properties(
    world_handle: Handle,
    body_handle: Handle,
    mass: Real,
    pixel_inertia: Real,
    pixel_local_com: &Vector,
    wake_up: bool,
    force_update: bool,
) {
    let local_com = &vector_pixels_to_meters(pixel_local_com);
    let inertia = pixels_to_meters(pixels_to_meters(pixel_inertia));

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.set_additional_mass_properties(
            MassProperties::new(point![local_com.x, local_com.y], mass, inertia),
            wake_up,
        );
        if force_update {
            body.recompute_mass_properties_from_colliders(&physics_world.physics_objects.collider_set);
        }
    }
}

pub fn body_add_force(world_handle: Handle, body_handle: Handle, pixel_force: &Vector) {
    let force = &vector_pixels_to_meters(pixel_force);

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        body.add_force(vector!(force.x, force.y), true);
    }
}

pub fn body_add_force_at_point(
    world_handle: Handle,
    body_handle: Handle,
    pixel_force: &Vector,
    pixel_point: &Vector,
) {
    let force = &vector_pixels_to_meters(pixel_force);
    let point = &vector_pixels_to_meters(pixel_point);

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    if let Some(body) = physics_world.physics_objects.rigid_body_set.get_mut(rigid_body_handle) {
        let local_point = point![point.x, point.y] + body.center_of_mass().coords;
        body.add_force_at_point(vector!(force.x, force.y), local_point, true);
    }
}

pub fn body_add_torque(world_handle: Handle, body_handle: Handle, pixel_torque: Real) {
    let torque = pixels_to_meters(pixels_to_meters(pixel_torque));
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().add_torque(torque, true);
}

pub fn body_apply_impulse(world_handle: Handle, body_handle: Handle, pixel_impulse: &Vector) {
    let impulse = &vector_pixels_to_meters(pixel_impulse);

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    let body = body.unwrap();
    let impulse = vector!(impulse.x, impulse.y);
    body.apply_impulse(impulse, true);
}

pub fn body_apply_impulse_at_point(
    world_handle: Handle,
    body_handle: Handle,
    pixel_impulse: &Vector,
    pixel_point: &Vector,
) {
    let impulse = &vector_pixels_to_meters(pixel_impulse);
    let point = &vector_pixels_to_meters(pixel_point);

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    let mut local_point = point![point.x, point.y];
    let body = body.unwrap();
    let impulse = vector!(impulse.x, impulse.y);
    local_point += body.center_of_mass().coords;
    body.apply_impulse_at_point(impulse, local_point, true);
}

pub fn body_get_constant_force(world_handle: Handle, body_handle: Handle) -> Vector {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    let constant_force = body.unwrap().user_force();
    vector_meters_to_pixels(&Vector {
        x: constant_force.x,
        y: constant_force.y,
    })
}

pub fn body_get_constant_torque(world_handle: Handle, body_handle: Handle) -> Real {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    meters_to_pixels(meters_to_pixels(body.unwrap().user_torque()))
}

pub fn body_apply_torque_impulse(
    world_handle: Handle,
    body_handle: Handle,
    pixel_torque_impulse: Real,
) {
    let torque_impulse = pixels_to_meters(pixels_to_meters(pixel_torque_impulse));

    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().apply_torque_impulse(torque_impulse, true);
}

pub fn body_reset_torques(world_handle: Handle, body_handle: Handle) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().reset_torques(false);
}

pub fn body_reset_forces(world_handle: Handle, body_handle: Handle) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().reset_forces(false);
}

pub fn body_wake_up(world_handle: Handle, body_handle: Handle, strong: bool) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    // TODO: check if this is OK, at the startup call to RapierPhysicsServer2D::free where body is not some
    assert!(body.is_some());
    let body = body.unwrap();
    if body.is_sleeping() {
        body.wake_up(strong);
    }
}

pub fn body_force_sleep(world_handle: Handle, body_handle: Handle) {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let rigid_body_handle = handle_to_rigid_body_handle(body_handle);
    let body = physics_world
        .physics_objects
        .rigid_body_set
        .get_mut(rigid_body_handle);
    assert!(body.is_some());
    body.unwrap().sleep();
}
