use rapier2d::na::Vector2;
use rapier2d::prelude::*;
use crate::handle::*;
use crate::user_data::*;
use crate::vector::Vector;
use crate::physics_world::*;

pub fn scale_shape(shape: &SharedShape, scale: &Vector) -> Option<SharedShape> {
    let shape_type = shape.shape_type();
    if shape_type == ShapeType::Ball {
        let new_shape = shape.as_ball().unwrap().scaled(&Vector2::<Real>::new(scale.x, scale.y), 20).unwrap();
        if new_shape.is_left() {
            let shape = new_shape.unwrap_left();
            return Some(SharedShape::new(shape));
        } else {
            let shape = new_shape.unwrap_right();
            return Some(SharedShape::new(shape));
        }
    }
    else if shape_type == ShapeType::Cuboid {
        let new_shape = shape.as_cuboid().unwrap().scaled(&Vector2::<Real>::new(scale.x, scale.y));
        return Some(SharedShape::new(new_shape));
    }
    else if shape_type == ShapeType::HalfSpace {
        let new_shape = shape.as_halfspace().unwrap().scaled(&Vector2::<Real>::new(scale.x, scale.y)).unwrap();
        return Some(SharedShape::new(new_shape));
    }
    else if shape_type == ShapeType::Polyline {
        let new_shape = shape.as_polyline().unwrap().clone().scaled(&Vector2::<Real>::new(scale.x, scale.y));
        return Some(SharedShape::new(new_shape));
    }
    else if shape_type == ShapeType::ConvexPolygon {
        let new_shape = shape.as_convex_polygon().unwrap().clone().scaled(&Vector2::<Real>::new(scale.x, scale.y)).unwrap();
        return Some(SharedShape::new(new_shape));
    }
    else if shape_type == ShapeType::Compound {
        let new_shapes = shape.as_compound().unwrap().shapes();
        let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
        for shape in new_shapes {
            let new_shape = scale_shape(&shape.1, scale);
            if let Some(shape_extracted) = new_shape {
                shapes_vec.push((shape.0, shape_extracted));
            }
        }
        return Some(SharedShape::compound(shapes_vec));
    }
    return None;
}

#[repr(C)]
pub struct Material {
    pub friction : Real,
    pub restitution : Real,
}

#[no_mangle]
pub extern "C" fn default_material() -> Material {
    Material {
        friction : 1.0,
        restitution : 0.0,
    }
}

#[no_mangle]
pub extern "C" fn collider_create_solid(world_handle : Handle, shape_handle : Handle, mat : &Material, body_handle : Handle, user_data : &UserData) -> Handle {
	let mut physics_engine = SINGLETON.lock().unwrap();
    let shape = physics_engine.get_shape(shape_handle);
	let mut collider = ColliderBuilder::new(shape.clone()).build();
    collider.set_friction(mat.friction);
    collider.set_restitution(mat.restitution);
    collider.set_friction_combine_rule(CoefficientCombineRule::Multiply);
    collider.set_restitution_combine_rule(CoefficientCombineRule::Max);
    collider.set_density(0.0);
	collider.user_data = user_data.get_data();
	collider.set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS);
	let physics_world = physics_engine.get_world(world_handle);
    return physics_world.insert_collider(collider, body_handle);
}

#[no_mangle]
pub extern "C" fn collider_create_sensor(world_handle : Handle, shape_handle : Handle, body_handle : Handle, user_data : &UserData) -> Handle {
	let mut physics_engine = SINGLETON.lock().unwrap();
    let shape = physics_engine.get_shape(shape_handle);
	let mut collider = ColliderBuilder::new(shape.clone()).build();
    collider.set_sensor(true);
	collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
	let mut collision_types = collider.active_collision_types();
	collision_types |= ActiveCollisionTypes::KINEMATIC_KINEMATIC;
	collider.set_active_collision_types(collision_types);
	collider.user_data = user_data.get_data();
	collider.set_active_hooks(ActiveHooks::FILTER_INTERSECTION_PAIR);
	let physics_world = physics_engine.get_world(world_handle);
    return physics_world.insert_collider(collider, body_handle);
}

#[no_mangle]
pub extern "C" fn collider_destroy(world_handle : Handle, handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    return physics_world.remove_collider(handle);
}

#[no_mangle]
pub extern "C" fn collider_get_position(world_handle : Handle, handle : Handle) -> Vector {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get(collider_handle);
    assert!(collider.is_some());
    let collider_vector = collider.unwrap().translation();
    return Vector { x : collider_vector.x, y : collider_vector.y };
}

#[no_mangle]
pub extern "C" fn collider_get_angle(world_handle : Handle, handle : Handle) -> Real {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get(collider_handle);
    assert!(collider.is_some());
    return collider.unwrap().rotation().angle();
}

#[no_mangle]
pub extern "C" fn collider_set_transform(world_handle : Handle, handle : Handle, shape_handle : Handle, pos : &Vector, rot : Real, scale: &Vector) {
    {
        let mut physics_engine = SINGLETON.lock().unwrap();
        let physics_world = physics_engine.get_world(world_handle);
        let collider_handle = handle_to_collider_handle(handle);
        let collider = physics_world.collider_set.get_mut(collider_handle);
        assert!(collider.is_some());
        let collider = collider.unwrap();
        collider.set_position_wrt_parent(Isometry::new(vector![pos.x, pos.y], rot));
    }
    {
        let new_shape:SharedShape;
        {
            let mut physics_engine = SINGLETON.lock().unwrap();
            let shape = physics_engine.get_shape(shape_handle);
            if let Some(extracted_shape) = scale_shape(shape, scale) {
                new_shape = extracted_shape;
            } else {
                return;
            }
        }
        {
            let mut physics_engine = SINGLETON.lock().unwrap();
            let physics_world = physics_engine.get_world(world_handle);
            let collider_handle = handle_to_collider_handle(handle);
            let collider = physics_world.collider_set.get_mut(collider_handle);
            assert!(collider.is_some());
            collider.unwrap().set_shape(new_shape);
        }
    }
}

#[no_mangle]
pub extern "C" fn collider_set_collision_events_enabled(world_handle : Handle, handle : Handle, enable : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get_mut(collider_handle);
    assert!(collider.is_some());
	let collider_access = collider.unwrap();
	let mut active_events = collider_access.active_events();
	if enable {
		active_events |= ActiveEvents::COLLISION_EVENTS;
	} else {
		active_events &= !ActiveEvents::COLLISION_EVENTS;
	}
	collider_access.set_active_events(active_events);
}

#[no_mangle]
pub extern "C" fn collider_set_contact_force_events_enabled(world_handle : Handle, handle : Handle, enable : bool) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let collider_handle = handle_to_collider_handle(handle);
    let collider = physics_world.collider_set.get_mut(collider_handle);
    assert!(collider.is_some());
	let collider_access = collider.unwrap();
	let mut active_events = collider_access.active_events();
	if enable {
		active_events |= ActiveEvents::CONTACT_FORCE_EVENTS;
	} else {
		active_events &= !ActiveEvents::CONTACT_FORCE_EVENTS;
	}
	collider_access.set_active_events(active_events);
}
