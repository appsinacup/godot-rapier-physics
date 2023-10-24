use rapier2d::prelude::*;
use crate::handle::*;
use crate::vector::Vector;
use crate::physics_world::*;

pub fn point_array_to_vec(data : &Vector, data_count : usize) -> Vec::<Point::<Real>> {
    let mut vec = Vec::<Point::<Real>>::with_capacity(data_count);
    unsafe {
        let data_raw = std::slice::from_raw_parts(data, data_count);
        for point in data_raw {
            vec.push(Point::<Real> { coords : vector![point.x, point.y] });
        }
    }
    return vec;
}

#[repr(C)]
pub struct ShapeInfo {
    handle: Handle,
    position : Vector,
    rotation : Real,
}
#[no_mangle]
pub extern "C" fn shape_create_box(size : &Vector) -> Handle {
	let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_halfspace(normal : &Vector) -> Handle {
	let shape = SharedShape::halfspace(UnitVector::new_normalize(vector![normal.x, normal.y]));
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_circle(radius : Real) -> Handle {
	let shape = SharedShape::ball(radius);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_capsule(half_height : Real, radius : Real) -> Handle {
	let top_circle = SharedShape::ball(radius);
    let top_circle_position = Isometry::new(vector![0.0, -half_height], 0.0);
	let bottom_circle = SharedShape::ball(radius);
    let bottom_circle_position = Isometry::new(vector![0.0, half_height], 0.0);
	let square = SharedShape::cuboid(0.5 * radius, 0.5 * (half_height - radius));
    let square_pos = Isometry::new(vector![0.0, 0.0], 0.0);
    let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
    shapes_vec.push((top_circle_position, top_circle));
    shapes_vec.push((bottom_circle_position, bottom_circle));
    shapes_vec.push((square_pos, square));
    let shape = SharedShape::compound(shapes_vec);
    // For now create the shape using circles and squares as the default capsule is buggy
    // in case of distance checking(returns invalid distances when close to the ends)
    // overall results in a 1.33x decrease in performance
    // TODO only do this in case of static objects?
	//let shape = SharedShape::capsule_y(half_height, radius);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_convex_polyline(points : &Vector, point_count : usize) -> Handle {
    let points_vec = point_array_to_vec(points, point_count);
    let shape_data = SharedShape::convex_polyline(points_vec);
	if shape_data.is_none() {
		return Handle::default();
	}
    let shape = shape_data.unwrap();
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_convave_polyline(points : &Vector, point_count : usize) -> Handle {
    let points_vec = point_array_to_vec(points, point_count);
    let shape = SharedShape::polyline(points_vec, None);
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_compound(shapes : &ShapeInfo, shape_count : usize) -> Handle {
    let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::with_capacity(shape_count);
    let mut physics_engine = SINGLETON.lock().unwrap();
    unsafe {
        let data_raw = std::slice::from_raw_parts(shapes, shape_count);
        for shape_info in data_raw {
            let shape = physics_engine.get_shape(shape_info.handle);
            let pos = vector![shape_info.position.x, shape_info.position.y];
            shapes_vec.push((Isometry::new(pos, shape_info.rotation), shape.clone()));
        }
    }

    let shape = SharedShape::compound(shapes_vec);
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_destroy(shape_handle : Handle) {
    let mut physics_engine = SINGLETON.lock().unwrap();
	return physics_engine.remove_shape(shape_handle);
}
