use crate::rapier2d::collider::*;
use crate::rapier2d::convert::meters_to_pixels;
use crate::rapier2d::convert::pixels_to_meters;
use crate::rapier2d::convert::vector_meters_to_pixels;
use crate::rapier2d::convert::vector_pixels_to_meters;
use crate::rapier2d::handle::*;
use crate::rapier2d::physics_world::*;
use crate::rapier2d::shape::*;
use crate::rapier2d::user_data::*;
use crate::rapier2d::vector::Vector;
use rapier2d::na::Vector2;
use rapier2d::parry;
use rapier2d::prelude::*;

pub struct RayHitInfo {
    pub pixel_position: Vector,
    pub normal: Vector,
    pub collider: Handle,
    pub user_data: UserData,
}

impl RayHitInfo {
    pub fn default() -> RayHitInfo {
        RayHitInfo {
            pixel_position: Vector { x: 0.0, y: 0.0 },
            normal: Vector { x: 0.0, y: 0.0 },
            collider: invalid_handle(),
            user_data: UserData { part1: 0, part2: 0 },
        }
    }
}

pub struct PointHitInfo {
    pub collider: Handle,
    pub user_data: UserData,
}

pub struct ShapeCastResult {
    pub collided: bool,
    pub toi: Real,
    pub pixel_witness1: Vector,
    pub pixel_witness2: Vector,
    pub normal1: Vector,
    pub normal2: Vector,
    pub collider: Handle,
    pub user_data: UserData,
}

impl ShapeCastResult {
    fn new() -> ShapeCastResult {
        ShapeCastResult {
            collided: false,
            toi: 1.0,
            collider: invalid_handle(),
            pixel_witness1: Vector { x: 0.0, y: 0.0 },
            pixel_witness2: Vector { x: 0.0, y: 0.0 },
            normal1: Vector { x: 0.0, y: 0.0 },
            normal2: Vector { x: 0.0, y: 0.0 },
            user_data: UserData { part1: 0, part2: 0 },
        }
    }
}

pub struct ContactResult {
    pub collided: bool,
    pub within_margin: bool,
    pub pixel_distance: Real,
    pub pixel_point1: Vector,
    pub pixel_point2: Vector,
    pub normal1: Vector,
    pub normal2: Vector,
}

impl ContactResult {
    fn new() -> ContactResult {
        ContactResult {
            collided: false,
            within_margin: false,
            pixel_distance: 0.0,
            pixel_point1: Vector { x: 0.0, y: 0.0 },
            pixel_point2: Vector { x: 0.0, y: 0.0 },
            normal1: Vector { x: 0.0, y: 0.0 },
            normal2: Vector { x: 0.0, y: 0.0 },
        }
    }
}

pub struct QueryExcludedInfo {
    pub query_collision_layer_mask: u32,
    pub query_canvas_instance_id: u64,
    // Pointer to array of objects
    pub query_exclude: *mut Handle,
    // Size of query_exclude array
    pub query_exclude_size: u32,
    pub query_exclude_body: i64,
}

pub fn default_query_excluded_info() -> QueryExcludedInfo {
    QueryExcludedInfo {
        query_collision_layer_mask: 0,
        query_canvas_instance_id: 0,
        query_exclude: &mut Handle::default(),
        query_exclude_size: 0,
        query_exclude_body: 0,
    }
}

type QueryHandleExcludedCallback = fn(
    world_handle: Handle,
    collider_handle: Handle,
    user_data: &UserData,
    handle_excluded_info: &QueryExcludedInfo,
) -> bool;

pub fn intersect_ray(
    world_handle: Handle,
    pixel_from: &Vector,
    dir: &Vector,
    pixel_length: Real,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_from_inside: bool,
    hit_info: &mut RayHitInfo,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> bool {
    let from = vector_pixels_to_meters(&pixel_from);
    let length = pixels_to_meters(pixel_length);

    let mut physics_engine = singleton().lock().unwrap();
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
        return !handle_excluded_callback(
            world_handle,
            collider_handle_to_handle(handle),
            &physics_world.get_collider_user_data(handle),
            &handle_excluded_info,
        );
    };

    filter.predicate = Some(&predicate);

    let mut result = false;
    let mut length_current = Real::MAX;
    physics_world.query_pipeline.intersections_with_ray(
        &physics_world.rigid_body_set,
        &physics_world.collider_set,
        &ray,
        length,
        solid,
        filter,
        |handle, intersection| {
            // Find closest intersection
            if intersection.toi > length_current {
                return true;
            }
            // Callback called on each collider hit by the ray.
            if hit_from_inside || intersection.toi != 0.0 {
                length_current = intersection.toi;
                result = true;

                let hit_point = ray.point_at(intersection.toi);
                let hit_normal = intersection.normal;
                hit_info.pixel_position = vector_meters_to_pixels(&Vector {
                    x: hit_point.x,
                    y: hit_point.y,
                });
                hit_info.normal = Vector {
                    x: hit_normal.x,
                    y: hit_normal.y,
                };
                hit_info.collider = collider_handle_to_handle(handle);
                hit_info.user_data = physics_world.get_collider_user_data(handle);
                //return false; // We found a collision hit.
            }
            true // Continue to search.
        },
    );

    return result;
}

pub fn intersect_point(
    world_handle: Handle,
    pixel_position: &Vector,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_info_array: *mut PointHitInfo,
    hit_info_length: usize,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> usize {
    let mut physics_engine = singleton().lock().unwrap();
    let physics_world = physics_engine.get_world(world_handle);
    let position = vector_pixels_to_meters(&pixel_position);
    let point = Point::new(position.x, position.y);
    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        return !handle_excluded_callback(
            world_handle,
            collider_handle_to_handle(handle),
            &physics_world.get_collider_user_data(handle),
            &handle_excluded_info,
        );
    };

    filter.predicate = Some(&predicate);

    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(
            hit_info_array,
            hit_info_length,
        ));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.intersections_with_point(
        &physics_world.rigid_body_set,
        &physics_world.collider_set,
        &point,
        filter,
        |handle| {
            // Callback called on each collider hit by the ray.
            hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
            hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
            cpt_hit += 1;
            let keep_searching = cpt_hit < hit_info_length;
            keep_searching // Continue to search collisions if we still have space for results.
        },
    );

    return cpt_hit;
}

pub fn shape_collide(
    pixel_motion1: &Vector,
    shape_info1: ShapeInfo,
    pixel_motion2: &Vector,
    shape_info2: ShapeInfo,
) -> ShapeCastResult {
    let mut motion1 = vector_pixels_to_meters(&pixel_motion1);
    let mut motion2 = vector_pixels_to_meters(&pixel_motion2);
    if motion1.x == 0.0 && motion1.y == 0.0 {
        motion1.x = 1e-5;
    }
    if motion2.x == 0.0 && motion2.y == 0.0 {
        motion2.x = 1e-5;
    }
    let position1 = vector_pixels_to_meters(&shape_info1.pixel_position);
    let position2 = vector_pixels_to_meters(&shape_info2.pixel_position);

    let mut physics_engine = singleton().lock().unwrap();

    let raw_shared_shape1 = physics_engine.get_shape(shape_info1.handle).clone();
    let skewed_shape1 = skew_shape(&raw_shared_shape1, shape_info1.skew);
    let shared_shape1 = scale_shape(
        &skewed_shape1,
        &Vector2::<Real>::new(shape_info1.scale.x, shape_info1.scale.y),
    );
    let raw_shared_shape2 = physics_engine.get_shape(shape_info2.handle).clone();
    let skewed_shape2 = skew_shape(&raw_shared_shape2, shape_info2.skew);
    let shared_shape2 = scale_shape(
        &skewed_shape2,
        &Vector2::<Real>::new(shape_info2.scale.x, shape_info2.scale.y),
    );

    let shape_vel1 = vector![motion1.x, motion1.y];
    let shape_vel2 = vector![motion2.x, motion2.y];
    let shape_transform1 = Isometry::new(vector![position1.x, position1.y], shape_info1.rotation);
    let shape_transform2 = Isometry::new(vector![position2.x, position2.y], shape_info2.rotation);
    let mut result = ShapeCastResult::new();
    let toi_result = parry::query::time_of_impact(
        &shape_transform1,
        &shape_vel1,
        shared_shape1.as_ref(),
        &shape_transform2,
        &shape_vel2,
        shared_shape2.as_ref(),
        1.0,
        false,
    );
    assert!(toi_result.is_ok());
    if let Some(hit) = toi_result.unwrap() {
        result.collided = true;
        result.toi = hit.toi;
        result.normal1 = Vector {
            x: hit.normal1.x,
            y: hit.normal1.y,
        };
        result.normal2 = Vector {
            x: hit.normal2.x,
            y: hit.normal2.y,
        };
        result.pixel_witness1 = vector_meters_to_pixels(&Vector {
            x: hit.witness1.x,
            y: hit.witness1.y,
        });
        result.pixel_witness2 = vector_meters_to_pixels(&Vector {
            x: hit.witness2.x,
            y: hit.witness2.y,
        });
        return result;
    }
    return result;
}

pub fn shape_casting(
    world_handle: Handle,
    pixel_motion: &Vector,
    shape_info: ShapeInfo,
    collide_with_body: bool,
    collide_with_area: bool,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
    ignore_intersecting: bool,
) -> ShapeCastResult {
    let motion = vector_pixels_to_meters(&pixel_motion);
    let position = vector_pixels_to_meters(&shape_info.pixel_position);

    let mut physics_engine = singleton().lock().unwrap();

    let raw_shared_shape = physics_engine.get_shape(shape_info.handle).clone();
    let skewed_shape = skew_shape(&raw_shared_shape, shape_info.skew);
    let shared_shape = scale_shape(
        &skewed_shape,
        &Vector2::<Real>::new(shape_info.scale.x, shape_info.scale.y),
    );

    let physics_world = physics_engine.get_world(world_handle);
    let mut shape_vel = vector![motion.x, motion.y];

    let shape_transform = Isometry::new(vector![position.x, position.y], shape_info.rotation);

    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        return !handle_excluded_callback(
            world_handle,
            collider_handle_to_handle(handle),
            &physics_world.get_collider_user_data(handle),
            &handle_excluded_info,
        );
    };

    filter.predicate = Some(&predicate);

    let mut result = ShapeCastResult::new();
    if shape_vel.x == 0.0 && shape_vel.y == 0.0 {
        shape_vel.x = 1e-5;
    }
    if let Some((collider_handle, hit)) = physics_world.query_pipeline.cast_shape(
        &physics_world.rigid_body_set,
        &physics_world.collider_set,
        &shape_transform,
        &shape_vel,
        shared_shape.as_ref(),
        1.0,
        ignore_intersecting,
        filter,
    ) {
        result.collided = true;
        result.toi = hit.toi;
        result.normal1 = Vector {
            x: hit.normal1.x,
            y: hit.normal1.y,
        };
        result.normal2 = Vector {
            x: hit.normal2.x,
            y: hit.normal2.y,
        };
        result.collider = collider_handle_to_handle(collider_handle);
        result.user_data = physics_world.get_collider_user_data(collider_handle);
        // first is world space
        let witness1 = hit.witness1;
        // second is local space
        let witness2 = shape_transform.transform_point(&hit.witness2);
        result.pixel_witness1 = vector_meters_to_pixels(&Vector {
            x: witness1.x,
            y: witness1.y,
        });
        result.pixel_witness2 = vector_meters_to_pixels(&Vector {
            x: witness2.x,
            y: witness2.y,
        });
    }
    return result;
}

pub fn intersect_shape(
    world_handle: Handle,
    shape_info: ShapeInfo,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_info_array: *mut PointHitInfo,
    hit_info_length: usize,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> usize {
    let position = vector_pixels_to_meters(&shape_info.pixel_position);
    let mut physics_engine = singleton().lock().unwrap();

    let raw_shared_shape = physics_engine.get_shape(shape_info.handle).clone();
    let skewed_shape = skew_shape(&raw_shared_shape, shape_info.skew);
    let shared_shape = scale_shape(
        &skewed_shape,
        &Vector2::<Real>::new(shape_info.scale.x, shape_info.scale.y),
    );

    let physics_world = physics_engine.get_world(world_handle);
    let shape_transform = Isometry::new(vector![position.x, position.y], shape_info.rotation);

    let mut filter = QueryFilter::new();

    if !collide_with_body {
        filter = filter.exclude_solids();
    }
    if !collide_with_area {
        filter = filter.exclude_sensors();
    }

    let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
        return !handle_excluded_callback(
            world_handle,
            collider_handle_to_handle(handle),
            &physics_world.get_collider_user_data(handle),
            &handle_excluded_info,
        );
    };

    filter.predicate = Some(&predicate);

    assert!(hit_info_length > 0);
    let hit_info_slice_opt;
    unsafe {
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(
            hit_info_array,
            hit_info_length,
        ));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world.query_pipeline.intersections_with_shape(
        &physics_world.rigid_body_set,
        &physics_world.collider_set,
        &shape_transform,
        shared_shape.as_ref(),
        filter,
        |handle| {
            // Callback called on each collider hit by the ray.
            hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
            hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
            cpt_hit += 1;
            let keep_searching = cpt_hit < hit_info_length;
            keep_searching // Continue to search collisions if we still have space for results.
        },
    );

    return cpt_hit;
}

pub fn intersect_aabb(
    world_handle: Handle,
    pixel_aabb_min: &Vector,
    pixel_aabb_max: &Vector,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_info_array: *mut PointHitInfo,
    hit_info_length: usize,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> usize {
    let aabb_min = vector_pixels_to_meters(&pixel_aabb_min);
    let aabb_max = vector_pixels_to_meters(&pixel_aabb_max);

    let mut physics_engine = singleton().lock().unwrap();

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
        hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(
            hit_info_array,
            hit_info_length,
        ));
    }
    assert!(hit_info_slice_opt.is_some());
    let hit_info_slice = hit_info_slice_opt.unwrap();

    let mut cpt_hit = 0;
    physics_world
        .query_pipeline
        .colliders_with_aabb_intersecting_aabb(&aabb, |handle| {
            let mut valid_hit = false;
            if let Some(collider) = physics_world.collider_set.get(*handle) {
                // type filder
                if collider.is_sensor() && collide_with_area {
                    valid_hit = true;
                } else if !collider.is_sensor() && collide_with_body {
                    valid_hit = true;
                }

                if valid_hit {
                    valid_hit = !handle_excluded_callback(
                        world_handle,
                        collider_handle_to_handle(*handle),
                        &physics_world.get_collider_user_data(*handle),
                        &handle_excluded_info,
                    );
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

pub fn shapes_contact(
    shape_info1: ShapeInfo,
    shape_info2: ShapeInfo,
    pixel_margin: Real,
) -> ContactResult {
    let position1 = vector_pixels_to_meters(&shape_info1.pixel_position);
    let position2 = vector_pixels_to_meters(&shape_info2.pixel_position);
    let margin = pixels_to_meters(pixel_margin);

    let mut physics_engine = singleton().lock().unwrap();

    //let prediction = Real::max(0.002, margin);
    let prediction = margin;

    let raw_shared_shape1 = physics_engine.get_shape(shape_info1.handle).clone();
    let skewed_shape1 = skew_shape(&raw_shared_shape1, shape_info1.skew);
    let shared_shape1 = scale_shape(
        &skewed_shape1,
        &Vector2::<Real>::new(shape_info1.scale.x, shape_info1.scale.y),
    );
    let raw_shared_shape2 = physics_engine.get_shape(shape_info2.handle).clone();
    let skewed_shape2 = skew_shape(&raw_shared_shape2, shape_info2.skew);
    let shared_shape2 = scale_shape(
        &skewed_shape2,
        &Vector2::<Real>::new(shape_info2.scale.x, shape_info2.scale.y),
    );

    let shape_transform1 = Isometry::new(vector![position1.x, position1.y], shape_info1.rotation);
    let shape_transform2 = Isometry::new(vector![position2.x, position2.y], shape_info2.rotation);

    let mut result = ContactResult::new();
    if let Ok(Some(contact)) = parry::query::contact(
        &shape_transform1,
        shared_shape1.as_ref(),
        &shape_transform2,
        shared_shape2.as_ref(),
        prediction,
    ) {
        // the distance is negative if there is intersection
        // and positive if the objects are separated by distance less than margin
        result.pixel_distance = meters_to_pixels(contact.dist);
        if contact.dist <= 0.0 {
            result.within_margin = false;
        } else {
            result.within_margin = true;
        }
        result.collided = true;
        result.normal1 = Vector {
            x: contact.normal1.x,
            y: contact.normal1.y,
        };
        result.normal2 = Vector {
            x: contact.normal2.x,
            y: contact.normal2.y,
        };
        result.pixel_point1 = vector_meters_to_pixels(&Vector {
            x: contact.point1.x + prediction * contact.normal1.x,
            y: contact.point1.y + prediction * contact.normal1.y,
        });
        result.pixel_point2 = vector_meters_to_pixels(&Vector {
            x: contact.point2.x,
            y: contact.point2.y,
        });
        return result;
    }
    return result;
}
