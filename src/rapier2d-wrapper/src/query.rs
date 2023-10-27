use rapier2d::parry;
use rapier2d::parry::query::NonlinearRigidMotion;
use parry::query::{DefaultQueryDispatcher};
use rapier2d::parry::query::QueryDispatcher;
use rapier2d::prelude::*;
use crate::handle::*;
use crate::user_data::*;
use crate::physics_world::*;
use crate::vector::Vector;
use crate::collider::*;
use crate::shape::*;

#[repr(C)]
pub struct RayHitInfo {
    position: Vector,
    normal: Vector,
    collider: Handle,
    user_data: UserData,
}

#[repr(C)]
pub struct PointHitInfo {
    collider: Handle,
    user_data: UserData,
}

#[repr(C)]
pub struct ShapeCastResult {
    collided: bool,
    toi : Real,
    witness1 : Vector,
    witness2 : Vector,
    normal1 : Vector,
    normal2 : Vector,
    collider: Handle,
    user_data: UserData
}

impl ShapeCastResult {
    fn new() -> ShapeCastResult {
        ShapeCastResult {
            collided : false,
            toi : 1.0, 
			collider : invalid_handle(),
            witness1 : Vector{ x: 0.0, y: 0.0 },
            witness2 : Vector{ x: 0.0, y: 0.0 },
            normal1 : Vector{ x: 0.0, y: 0.0 },
            normal2 : Vector{ x: 0.0, y: 0.0 },
            user_data: UserData { part1: 0, part2: 0 }
        }
    }
}

#[repr(C)]
pub struct ContactResult {
    collided: bool,
    within_margin: bool,
    distance: Real,
    point1 : Vector,
    point2 : Vector,
    normal1 : Vector,
    normal2 : Vector
}

impl ContactResult {
    fn new() -> ContactResult {
        ContactResult {
            collided : false,
            within_margin: false,
            distance : 0.0, 
            point1 : Vector{ x: 0.0, y: 0.0 },
            point2 : Vector{ x: 0.0, y: 0.0 },
            normal1 : Vector{ x: 0.0, y: 0.0 },
            normal2 : Vector{ x: 0.0, y: 0.0 }
        }
    }
}

#[repr(C)]
pub struct QueryExcludedInfo {
    query_collision_layer_mask: u32,
    query_canvas_instance_id: u64,
    // Pointer to array of objects
    query_exclude: * mut Handle,
    // Size of query_exclude array
    query_exclude_size: u32,
    query_exclude_body: i64,
}

#[no_mangle]
pub extern "C" fn default_query_excluded_info() -> QueryExcludedInfo {
    QueryExcludedInfo {
        query_collision_layer_mask: 0,
        query_canvas_instance_id: 0,
        query_exclude: &mut Handle::default(),
        query_exclude_size: 0,
        query_exclude_body: 0,
    }
}

type QueryHandleExcludedCallback = Option<extern "C" fn(world_handle : Handle, collider_handle : Handle, user_data : &UserData, handle_excluded_info: &QueryExcludedInfo) -> bool>;

#[no_mangle]
pub extern "C" fn intersect_ray(world_handle : Handle, from : &Vector, dir : &Vector, length: Real, collide_with_body: bool, collide_with_area: bool, hit_from_inside: bool, hit_info : &mut RayHitInfo, handle_excluded_callback: QueryHandleExcludedCallback, handle_excluded_info: &QueryExcludedInfo) -> bool {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let ray = Ray::new(point![from.x, from.y], vector![dir.x, dir.y]);
    let solid = true;
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle), &handle_excluded_info);
        }
        return true;
    };

    filter.predicate = Some(&predicate);
     
    let mut result = false;
    physics_world.query_pipeline.intersections_with_ray(&physics_world.rigid_body_set, &physics_world.collider_set, &ray, length, solid, filter,
        |handle, intersection| {
            // Callback called on each collider hit by the ray.

            if hit_from_inside || intersection.toi != 0.0 {
                result = true;

                let hit_point = ray.point_at(intersection.toi);
                let hit_normal = intersection.normal;
                hit_info.position = Vector {
                    x: hit_point.x,
                    y: hit_point.y,
                };
                hit_info.normal = Vector {
                    x: hit_normal.x,
                    y: hit_normal.y,
                };
                hit_info.collider = collider_handle_to_handle(handle);
				hit_info.user_data = physics_world.get_collider_user_data(handle);
                return false; // We found a collision hit.
            }
            true // Continue to search.
        },
    );

    return result;
}

#[no_mangle]
pub extern "C" fn intersect_point(world_handle : Handle, position : &Vector,  collide_with_body: bool, collide_with_area: bool, hit_info_array : *mut PointHitInfo, hit_info_length : usize, handle_excluded_callback: QueryHandleExcludedCallback, handle_excluded_info: &QueryExcludedInfo) -> usize {
    let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);

    let point = Point::new(position.x, position.y);
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle), &handle_excluded_info);
        }
        return true;
    };

    filter.predicate = Some(&predicate);
  
    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(hit_info_array, hit_info_length));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.intersections_with_point( &physics_world.rigid_body_set, &physics_world.collider_set, &point, filter,
        |handle| {
        // Callback called on each collider hit by the ray.
        hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
        hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
        cpt_hit += 1;
        let keep_searching = cpt_hit < hit_info_length;
        keep_searching // Continue to search collisions if we still have space for results.
    });

    return cpt_hit;
}

#[no_mangle]
pub extern "C" fn shape_collide(motion1 : &Vector, shape_info1: ShapeInfo, motion2 : &Vector, shape_info2: ShapeInfo) -> ShapeCastResult {
    let mut physics_engine = SINGLETON.lock().unwrap();

    let mut shared_shape1 = physics_engine.get_shape(shape_info1.handle).clone();
    if let Some(new_shape) = scale_shape(&shared_shape1, &shape_info1.scale) {
        shared_shape1 = new_shape;
    }
    let mut shared_shape2 = physics_engine.get_shape(shape_info1.handle).clone();
    if let Some(new_shape) = scale_shape(&shared_shape2, &shape_info1.scale) {
        shared_shape2 = new_shape;
    }
    
    let shape_vel1 = vector![motion1.x, motion1.y];
    let shape_vel2 = vector![motion2.x, motion2.y];
    let shape_transform1 = Isometry::new(vector![shape_info1.position.x, shape_info1.position.y], shape_info1.rotation);
    let shape_transform2 = Isometry::new(vector![shape_info2.position.x, shape_info2.position.y], shape_info2.rotation);
    let shape_nonlin1 = NonlinearRigidMotion::new(shape_transform1, Point::default(), shape_vel1, 0.0);
    let shape_nonlin2 = NonlinearRigidMotion::new(shape_transform2, Point::default(), shape_vel2, 0.0);
    let mut result = ShapeCastResult::new();
    let dispatcher = DefaultQueryDispatcher;
    let toi_result = dispatcher.nonlinear_time_of_impact(&shape_nonlin1, shared_shape1.as_ref(), &shape_nonlin2, shared_shape2.as_ref(), 0.0, 1.0, true);
    assert!(toi_result.is_ok());
    if let Some(hit) = toi_result.unwrap() {
        result.collided = true;
        result.toi = hit.toi;
        result.witness1 = Vector{ x: hit.witness1.x, y: hit.witness1.y };
        result.witness2 = Vector{ x: hit.witness2.x, y: hit.witness2.y };
        result.normal1 = Vector{ x: hit.normal1.x, y: hit.normal1.y };
        result.normal2 = Vector{ x: hit.normal2.x, y: hit.normal2.y };
        return result;
    }
    return result;
}

#[no_mangle]
pub extern "C" fn shape_casting(world_handle : Handle, motion : &Vector, shape_info: ShapeInfo, collide_with_body: bool, collide_with_area: bool, handle_excluded_callback: QueryHandleExcludedCallback, handle_excluded_info: &QueryExcludedInfo) -> ShapeCastResult {
    let mut physics_engine = SINGLETON.lock().unwrap();

    let mut shared_shape = physics_engine.get_shape(shape_info.handle).clone();
    if let Some(new_shape) = scale_shape(&shared_shape, &shape_info.scale) {
        shared_shape = new_shape;
    }

	let physics_world = physics_engine.get_world(world_handle);
    
    let shape_vel = vector![motion.x, motion.y];
    let shape_transform = Isometry::new(vector![shape_info.position.x, shape_info.position.y], shape_info.rotation);
    
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle), &handle_excluded_info);
        }
        return true;
    };

    filter.predicate = Some(&predicate);

    let mut result = ShapeCastResult::new();
    
    if let Some((collider_handle, hit)) = physics_world.query_pipeline.cast_shape(
        &physics_world.rigid_body_set, &physics_world.collider_set, &shape_transform, &shape_vel, shared_shape.as_ref(), 1.0, false, filter
    ) {
        result.collided = true;
        result.toi = hit.toi;
        result.witness1 = Vector{ x: hit.witness1.x, y: hit.witness1.y };
        result.witness2 = Vector{ x: hit.witness2.x, y: hit.witness2.y };
        result.normal1 = Vector{ x: hit.normal1.x, y: hit.normal1.y };
        result.normal2 = Vector{ x: hit.normal2.x, y: hit.normal2.y };
        result.collider = collider_handle_to_handle(collider_handle);
        result.user_data = physics_world.get_collider_user_data(collider_handle);
        return result;
    }
    return result;
}

#[no_mangle]
pub extern "C" fn intersect_shape(world_handle : Handle, shape_info: ShapeInfo, collide_with_body: bool, collide_with_area: bool, hit_info_array : *mut PointHitInfo, hit_info_length : usize, handle_excluded_callback: QueryHandleExcludedCallback, handle_excluded_info: &QueryExcludedInfo) -> usize {
    let mut physics_engine = SINGLETON.lock().unwrap();

    let mut shared_shape = physics_engine.get_shape(shape_info.handle).clone();
    if let Some(new_shape) = scale_shape(&shared_shape, &shape_info.scale) {
        shared_shape = new_shape;
    }

	let physics_world = physics_engine.get_world(world_handle);
    
    let shape_transform = Isometry::new(vector![shape_info.position.x, shape_info.position.y], shape_info.rotation);
    
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        if handle_excluded_callback.is_some() {
            return !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(handle), &physics_world.get_collider_user_data(handle), &handle_excluded_info);
        }
        return true;
    };

    filter.predicate = Some(&predicate);

    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(hit_info_array, hit_info_length));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.intersections_with_shape (
        &physics_world.rigid_body_set, &physics_world.collider_set, &shape_transform, shared_shape.as_ref(), filter, 
        |handle| {
            // Callback called on each collider hit by the ray.
            hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
            hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
            cpt_hit += 1;
            let keep_searching = cpt_hit < hit_info_length;
            keep_searching // Continue to search collisions if we still have space for results.
        }
    );

    return cpt_hit;
}

#[no_mangle]
pub extern "C" fn intersect_aabb(world_handle : Handle, aabb_min : &Vector, aabb_max : &Vector, collide_with_body: bool, collide_with_area: bool, hit_info_array : *mut PointHitInfo, hit_info_length : usize, handle_excluded_callback: QueryHandleExcludedCallback, handle_excluded_info: &QueryExcludedInfo) -> usize {
    let mut physics_engine = SINGLETON.lock().unwrap();

	let physics_world = physics_engine.get_world(world_handle);
    
    // let aabb_transform = Isometry::new(vector![position.x, position.y], rotation);
    let aabb_min_point = Point::new(aabb_min.x, aabb_min.y);
    let aabb_max_point = Point::new(aabb_max.x, aabb_max.y);
    
    // let transformed_aabb_min = aabb_transform * aabb_min_point;
    // let transformed_aabb_max = aabb_transform * aabb_max_point;
    
    let aabb = Aabb {
        mins: aabb_min_point,
        maxs: aabb_max_point,
    };

    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(hit_info_array, hit_info_length));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.colliders_with_aabb_intersecting_aabb(&aabb,         
        |handle| {

        let mut valid_hit = false;
        if let Some(collider) = physics_world.collider_set.get(*handle) {

            // type filder
            if collider.is_sensor() && collide_with_area {
                valid_hit = true;
            } else if !collider.is_sensor() && collide_with_body {
                valid_hit = true;
            }

            if valid_hit && handle_excluded_callback.is_some(){
                valid_hit = !handle_excluded_callback.unwrap()(world_handle, collider_handle_to_handle(*handle), &physics_world.get_collider_user_data(*handle), &handle_excluded_info);
            }
        }
        
        if !valid_hit {
            return true; // continue
        }

        // Callback called on each collider hit by the ray.
        hit_info_slice[cpt_hit].collider = collider_handle_to_handle(*handle);
        hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(*handle);
        cpt_hit += 1;
        let keep_searching = cpt_hit < hit_info_length;
        keep_searching // Continue to search collisions if we still have space for results.
    });

    return cpt_hit;
}

#[no_mangle]
pub extern "C" fn shapes_contact(world_handle : Handle, shape_info1 : ShapeInfo, shape_info2 : ShapeInfo, margin: Real) -> ContactResult {
    let mut physics_engine = SINGLETON.lock().unwrap();
    
    let physics_world = physics_engine.get_world(world_handle);

    let prediction = Real::max(physics_world.solver_prediction_distance, margin);

    let mut shared_shape1 = physics_engine.get_shape(shape_info1.handle).clone();
    if let Some(new_shape) = scale_shape(&shared_shape1, &shape_info1.scale) {
        shared_shape1 = new_shape;
    }
    let mut shared_shape2 = physics_engine.get_shape(shape_info2.handle).clone();
    if let Some(new_shape) = scale_shape(&shared_shape2, &shape_info2.scale) {
        shared_shape2 = new_shape;
    }
    
    let shape_transform1 = Isometry::new(vector![shape_info1.position.x, shape_info1.position.y], shape_info1.rotation);
    let shape_transform2 = Isometry::new(vector![shape_info2.position.x, shape_info2.position.y], shape_info2.rotation);
    
    let mut result = ContactResult::new();
    if let Ok(Some(contact)) = parry::query::contact(
        &shape_transform1, shared_shape1.as_ref(), &shape_transform2, shared_shape2.as_ref(), prediction
    ) {
        // the distance is negative if there is intersection
        // and positive if the objects are separated by distance less than margin
        result.distance = contact.dist;
        if contact.dist <= 0.0 {
            result.within_margin = false;
        } else {
            result.within_margin = true;
        }
        result.collided = true;
        result.point1 = Vector{ x: contact.point1.x + prediction * contact.normal1.x, y: contact.point1.y + prediction * contact.normal1.y };
        result.point2 = Vector{ x: contact.point2.x, y: contact.point2.y };
        result.normal1 = Vector{ x: contact.normal1.x, y: contact.normal1.y };
        result.normal2 = Vector{ x: contact.normal2.x, y: contact.normal2.y };
        return result;
    }
    return result;
}
