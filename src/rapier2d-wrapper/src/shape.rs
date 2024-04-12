use rapier2d::prelude::*;
use crate::convert::*;
use crate::handle::*;
use crate::vector::Vector;
use crate::physics_world::*;

pub fn pixel_point_array_to_vec(pixel_data : &Vector, data_count : usize) -> Vec::<Point::<Real>> {
    let mut vec = Vec::<Point::<Real>>::with_capacity(data_count);
    unsafe {
        let data_raw = std::slice::from_raw_parts(pixel_data, data_count);
        for pixel_point in data_raw {
            let point = &vector_pixels_to_meters(pixel_point);
            vec.push(Point::<Real> { coords : vector![point.x, point.y] });
        }
    }
    return vec;
}

#[repr(C)]
pub struct ShapeInfo {
    pub handle: Handle,
    pub pixel_position : Vector,
    pub rotation : Real,
    pub skew: Real,
    pub scale: Vector,
}

#[no_mangle]
pub extern "C" fn shape_create_box(pixel_size : &Vector) -> Handle {
    let size = &vector_pixels_to_meters(pixel_size);
	let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y);
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_halfspace(normal : &Vector, pixel_distance: Real) -> Handle {
    let distance = pixels_to_meters(pixel_distance);

	let shape = SharedShape::halfspace(UnitVector::new_normalize(vector![normal.x, -normal.y]));
    let shape_position = Isometry::new(vector![normal.x * distance, normal.y * distance], 0.0);
    let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
    shapes_vec.push((shape_position, shape));
    let shape_compound = SharedShape::compound(shapes_vec);
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.insert_shape(shape_compound);
}

#[no_mangle]
pub extern "C" fn shape_create_circle(pixel_radius : Real) -> Handle {
    let radius = pixels_to_meters(pixel_radius);
	let shape = SharedShape::ball(radius);
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_capsule(pixel_half_height : Real, pixel_radius : Real) -> Handle {
    let half_height = pixels_to_meters(pixel_half_height);
    let radius = pixels_to_meters(pixel_radius);

	//let top_circle = SharedShape::ball(radius);
    //let top_circle_position = Isometry::new(vector![0.0, -half_height], 0.0);
	//let bottom_circle = SharedShape::ball(radius);
    //let bottom_circle_position = Isometry::new(vector![0.0, half_height], 0.0);
	//let square = SharedShape::cuboid(0.5 * radius, 0.5 * (half_height - radius));
    //let square_pos = Isometry::default();
    //let mut shapes_vec = Vec::<(Isometry<Real>, SharedShape)>::new();
    //shapes_vec.push((top_circle_position, top_circle));
    //shapes_vec.push((bottom_circle_position, bottom_circle));
    //shapes_vec.push((square_pos, square));
    //let shape = SharedShape::compound(shapes_vec);
    // For now create the shape using circles and squares as the default capsule is buggy
    // in case of distance checking(returns invalid distances when close to the ends)
    // overall results in a 1.33x decrease in performance
    // TODO only do this in case of static objects?
	let shape = SharedShape::capsule_y(half_height, radius);
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_convex_polyline(pixel_points : &Vector, point_count : usize) -> Handle {

    let points_vec = pixel_point_array_to_vec(pixel_points, point_count);
    let shape_data = SharedShape::convex_polyline(points_vec);
	if shape_data.is_none() {
		return Handle::default();
	}
    let shape = shape_data.unwrap();
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_create_convave_polyline(pixel_points : &Vector, point_count : usize) -> Handle {
    let points_vec = pixel_point_array_to_vec(pixel_points, point_count);
    let shape = SharedShape::polyline(points_vec, None);
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.insert_shape(shape);
}

#[no_mangle]
pub extern "C" fn shape_destroy(shape_handle : Handle) {
    let mut physics_engine = singleton().lock().unwrap();
	return physics_engine.remove_shape(shape_handle);
}
