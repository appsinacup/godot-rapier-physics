use crate::rapier_wrapper::prelude::*;
use godot::log::godot_error;
use nalgebra::zero;
use rapier::parry;
use rapier::parry::query::ShapeCastOptions;
use rapier::prelude::*;
use std::ops::Mul;

pub struct RayHitInfo {
    pub pixel_position: Vector<Real>,
    pub normal: Vector<Real>,
    pub collider: Handle,
    pub user_data: UserData,
}

impl RayHitInfo {
    pub fn default() -> RayHitInfo {
        RayHitInfo {
            pixel_position: zero(),
            normal: zero(),
            collider: invalid_handle(),
            user_data: UserData::invalid_user_data(),
        }
    }
}

#[derive(Copy, Clone, Default)]
pub struct PointHitInfo {
    pub collider: Handle,
    pub user_data: UserData,
}

#[derive(Default)]
pub struct ShapeCastResult {
    pub collided: bool,
    pub toi: Real,
    pub pixel_witness1: Vector<Real>,
    pub pixel_witness2: Vector<Real>,
    pub normal1: Vector<Real>,
    pub normal2: Vector<Real>,
    pub collider: Handle,
    pub user_data: UserData,
}

impl ShapeCastResult {
    fn new() -> ShapeCastResult {
        ShapeCastResult {
            collided: false,
            toi: 1.0,
            collider: invalid_handle(),
            pixel_witness1: zero(),
            pixel_witness2: zero(),
            normal1: zero(),
            normal2: zero(),
            user_data: UserData::invalid_user_data(),
        }
    }
}

#[derive(Default)]
pub struct ContactResult {
    pub collided: bool,
    pub within_margin: bool,
    pub pixel_distance: Real,
    pub pixel_point1: Vector<Real>,
    pub pixel_point2: Vector<Real>,
    pub normal1: Vector<Real>,
    pub normal2: Vector<Real>,
}

#[derive(Default)]
pub struct QueryExcludedInfo {
    pub query_collision_layer_mask: u32,
    pub query_canvas_instance_id: u64,
    // Pointer to array of objects
    pub query_exclude: Vec<Handle>,
    pub query_exclude_size: usize,
    pub query_exclude_body: i64,
}

type QueryHandleExcludedCallback = fn(
    world_handle: Handle,
    collider_handle: Handle,
    user_data: &UserData,
    handle_excluded_info: &QueryExcludedInfo,
) -> bool;

pub fn intersect_ray(
    world_handle: Handle,
    pixel_from: Vector<Real>,
    dir: Vector<Real>,
    pixel_length: Real,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_from_inside: bool,
    hit_info: &mut RayHitInfo,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> bool {
    let mut result = false;

    if let Some(physics_world) = physics_engine().get_world(world_handle) {
        let from = vector_pixels_to_meters(pixel_from);
        let length = pixels_to_meters(pixel_length);

        let ray = Ray::new(Point { coords: from }, dir);
        let mut filter = QueryFilter::new();

        if !collide_with_body {
            filter = filter.exclude_solids();
        }
        if !collide_with_area {
            filter = filter.exclude_sensors();
        }

        let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
            !handle_excluded_callback(
                world_handle,
                collider_handle_to_handle(handle),
                &physics_world.get_collider_user_data(handle),
                handle_excluded_info,
            )
        };

        filter.predicate = Some(&predicate);

        let mut length_current = Real::MAX;
        physics_world
            .physics_objects
            .query_pipeline
            .intersections_with_ray(
                &physics_world.physics_objects.rigid_body_set,
                &physics_world.physics_objects.collider_set,
                &ray,
                length,
                true,
                filter,
                |handle, intersection| {
                    // Find closest intersection
                    if intersection.time_of_impact > length_current {
                        return true;
                    }
                    // Callback called on each collider hit by the ray.
                    if hit_from_inside || intersection.time_of_impact != 0.0 {
                        length_current = intersection.time_of_impact;
                        result = true;

                        let hit_point = ray.point_at(intersection.time_of_impact);
                        let hit_normal = intersection.normal;
                        hit_info.pixel_position = vector_meters_to_pixels(hit_point.coords);
                        hit_info.normal = hit_normal;
                        hit_info.collider = collider_handle_to_handle(handle);
                        hit_info.user_data = physics_world.get_collider_user_data(handle);
                        //return false; // We found a collision hit.
                    }
                    true // Continue to search.
                },
            );
    }
    result
}

pub fn intersect_point(
    world_handle: Handle,
    pixel_position: Vector<Real>,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_info_array: *mut PointHitInfo,
    hit_info_length: usize,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> usize {
    let mut cpt_hit = 0;
    if hit_info_length <= 0 {
        return cpt_hit;
    }
    if let Some(physics_world) = physics_engine().get_world(world_handle) {
        let position = vector_pixels_to_meters(pixel_position);
        let point = Point { coords: position };
        let mut filter = QueryFilter::new();

        if !collide_with_body {
            filter = filter.exclude_solids();
        }
        if !collide_with_area {
            filter = filter.exclude_sensors();
        }

        let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
            !handle_excluded_callback(
                world_handle,
                collider_handle_to_handle(handle),
                &physics_world.get_collider_user_data(handle),
                handle_excluded_info,
            )
        };

        filter.predicate = Some(&predicate);

        let hit_info_slice_opt;
        unsafe {
            hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(
                hit_info_array,
                hit_info_length,
            ));
        }
        assert!(hit_info_slice_opt.is_some());
        let hit_info_slice = hit_info_slice_opt.unwrap();

        physics_world
            .physics_objects
            .query_pipeline
            .intersections_with_point(
                &physics_world.physics_objects.rigid_body_set,
                &physics_world.physics_objects.collider_set,
                &point,
                filter,
                |handle| {
                    // Callback called on each collider hit by the ray.
                    hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
                    hit_info_slice[cpt_hit].user_data =
                        physics_world.get_collider_user_data(handle);
                    cpt_hit += 1;

                    cpt_hit < hit_info_length // Continue to search collisions if we still have space for results.
                },
            );
    }
    cpt_hit
}

pub fn shape_collide(
    pixel_motion1: Vector<Real>,
    shape_info1: ShapeInfo,
    pixel_motion2: Vector<Real>,
    shape_info2: ShapeInfo,
) -> ShapeCastResult {
    let mut result = ShapeCastResult::new();
    let physics_engine = physics_engine();
    let shape_vel1 = vector_pixels_to_meters(pixel_motion1);
    let shape_vel2 = vector_pixels_to_meters(pixel_motion2);
    let position1 = vector_pixels_to_meters(shape_info1.pixel_position);
    let position2 = vector_pixels_to_meters(shape_info2.pixel_position);

    if let Some(raw_shared_shape1) = physics_engine.get_shape(shape_info1.handle) {
        let skewed_shape1 = skew_shape(raw_shared_shape1, shape_info1.skew);
        let shared_shape1 = scale_shape(&skewed_shape1, shape_info1.scale);
        if let Some(raw_shared_shape2) = physics_engine.get_shape(shape_info2.handle) {
            let skewed_shape2 = skew_shape(raw_shared_shape2, shape_info2.skew);
            let shared_shape2 = scale_shape(&skewed_shape2, shape_info2.scale);

            let shape_transform1 = Isometry::new(position1, shape_info1.rotation);
            let shape_transform2 = Isometry::new(position2, shape_info2.rotation);

            let mut shape_cast_options = ShapeCastOptions::default();
            shape_cast_options.max_time_of_impact = 1.0;
            shape_cast_options.compute_impact_geometry_on_penetration = true;
            shape_cast_options.stop_at_penetration = true;
            let toi_result = parry::query::cast_shapes(
                &shape_transform1,
                &shape_vel1,
                shared_shape1.as_ref(),
                &shape_transform2,
                &shape_vel2,
                shared_shape2.as_ref(),
                shape_cast_options,
            );
            if let Ok(hit) = toi_result {
                if let Some(hit) = hit {
                    result.collided = true;
                    result.toi = hit.time_of_impact;
                    result.normal1 = hit.normal1.into_inner();
                    result.normal2 = hit.normal2.into_inner();
                    result.pixel_witness1 = vector_meters_to_pixels(hit.witness1.coords);
                    result.pixel_witness2 = vector_meters_to_pixels(hit.witness2.coords);
                }
                // can we get a hit without a result?
                godot_error!("hit without a result");
            }
        }
    }
    result
}

pub fn shape_casting(
    world_handle: Handle,
    pixel_motion: Vector<Real>,
    shape_info: ShapeInfo,
    collide_with_body: bool,
    collide_with_area: bool,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> ShapeCastResult {
    let mut result = ShapeCastResult::new();

    let physics_engine = physics_engine();

    if let Some(raw_shared_shape) = physics_engine.get_shape(shape_info.handle) {
        let shape_vel = vector_pixels_to_meters(pixel_motion);
        let position = vector_pixels_to_meters(shape_info.pixel_position);
        let skewed_shape = skew_shape(raw_shared_shape, shape_info.skew);
        let shared_shape = scale_shape(&skewed_shape, shape_info.scale);

        if let Some(physics_world) = physics_engine.get_world(world_handle) {
            let shape_transform = Isometry::new(position, shape_info.rotation);

            let mut filter = QueryFilter::new();

            if !collide_with_body {
                filter = filter.exclude_solids();
            }
            if !collide_with_area {
                filter = filter.exclude_sensors();
            }

            let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
                !handle_excluded_callback(
                    world_handle,
                    collider_handle_to_handle(handle),
                    &physics_world.get_collider_user_data(handle),
                    handle_excluded_info,
                )
            };

            filter.predicate = Some(&predicate);

            let mut shape_cast_options = ShapeCastOptions::default();
            shape_cast_options.max_time_of_impact = 1.0;
            shape_cast_options.compute_impact_geometry_on_penetration = true;
            shape_cast_options.stop_at_penetration = true;
            if let Some((collider_handle, hit)) =
                physics_world.physics_objects.query_pipeline.cast_shape(
                    &physics_world.physics_objects.rigid_body_set,
                    &physics_world.physics_objects.collider_set,
                    &shape_transform,
                    &shape_vel,
                    shared_shape.as_ref(),
                    shape_cast_options,
                    filter,
                )
            {
                result.collided = true;
                result.toi = hit.time_of_impact;
                result.normal1 = hit.normal1.into_inner();
                result.normal2 = hit.normal2.into_inner();
                result.collider = collider_handle_to_handle(collider_handle);
                result.user_data = physics_world.get_collider_user_data(collider_handle);
                // first is world space
                let witness1 = hit.witness1;
                // second is local space
                let witness2 = shape_transform.transform_point(&hit.witness2);
                result.pixel_witness1 = vector_meters_to_pixels(witness1.coords);
                result.pixel_witness2 = vector_meters_to_pixels(witness2.coords);
            }
        }
    }
    result
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
    if hit_info_length <= 0 {
        return 0;
    }
    let physics_engine = physics_engine();
    let mut cpt_hit = 0;

    if let Some(raw_shared_shape) = physics_engine.get_shape(shape_info.handle) {
        let skewed_shape = skew_shape(raw_shared_shape, shape_info.skew);
        let shared_shape = scale_shape(&skewed_shape, shape_info.scale);

        if let Some(physics_world) = physics_engine.get_world(world_handle) {
            let position = vector_pixels_to_meters(shape_info.pixel_position);
            let shape_transform = Isometry::new(position, shape_info.rotation);

            let mut filter = QueryFilter::new();

            if !collide_with_body {
                filter = filter.exclude_solids();
            }
            if !collide_with_area {
                filter = filter.exclude_sensors();
            }

            let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
                !handle_excluded_callback(
                    world_handle,
                    collider_handle_to_handle(handle),
                    &physics_world.get_collider_user_data(handle),
                    handle_excluded_info,
                )
            };

            filter.predicate = Some(&predicate);

            let hit_info_slice_opt;
            unsafe {
                hit_info_slice_opt = Some(std::slice::from_raw_parts_mut(
                    hit_info_array,
                    hit_info_length,
                ));
            }
            assert!(hit_info_slice_opt.is_some());
            let hit_info_slice = hit_info_slice_opt.unwrap();

            physics_world
                .physics_objects
                .query_pipeline
                .intersections_with_shape(
                    &physics_world.physics_objects.rigid_body_set,
                    &physics_world.physics_objects.collider_set,
                    &shape_transform,
                    shared_shape.as_ref(),
                    filter,
                    |handle| {
                        // Callback called on each collider hit by the ray.
                        hit_info_slice[cpt_hit].collider = collider_handle_to_handle(handle);
                        hit_info_slice[cpt_hit].user_data =
                            physics_world.get_collider_user_data(handle);
                        cpt_hit += 1;

                        cpt_hit < hit_info_length // Continue to search collisions if we still have space for results.
                    },
                );
        }
    }
    cpt_hit
}

pub fn intersect_aabb(
    world_handle: Handle,
    pixel_aabb_min: Vector<Real>,
    pixel_aabb_max: Vector<Real>,
    collide_with_body: bool,
    collide_with_area: bool,
    hit_info_slice: &mut [PointHitInfo],
    max_results: usize,
    handle_excluded_callback: QueryHandleExcludedCallback,
    handle_excluded_info: &QueryExcludedInfo,
) -> usize {
    let mut cpt_hit = 0;

    if let Some(physics_world) = physics_engine().get_world(world_handle) {
        let aabb_min = vector_pixels_to_meters(pixel_aabb_min);
        let aabb_max = vector_pixels_to_meters(pixel_aabb_max);

        // let aabb_transform = Isometry::new(vector![position.x, position.y], rotation);
        let aabb_min_point = Point { coords: aabb_min };
        let aabb_max_point = Point { coords: aabb_max };

        // let transformed_aabb_min = aabb_transform * aabb_min_point;
        // let transformed_aabb_max = aabb_transform * aabb_max_point;

        let aabb = Aabb {
            mins: aabb_min_point,
            maxs: aabb_max_point,
        };

        physics_world
            .physics_objects
            .query_pipeline
            .colliders_with_aabb_intersecting_aabb(&aabb, |handle| {
                let mut valid_hit = false;
                if let Some(collider) = physics_world.physics_objects.collider_set.get(*handle) {
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
                            handle_excluded_info,
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

                cpt_hit < max_results // Continue to search collisions if we still have space for results.
            });
    }
    cpt_hit
}

pub fn shapes_contact(
    shape_info1: ShapeInfo,
    shape_info2: ShapeInfo,
    pixel_margin: Real,
) -> ContactResult {
    let position1 = vector_pixels_to_meters(shape_info1.pixel_position);
    let position2 = vector_pixels_to_meters(shape_info2.pixel_position);
    let margin = pixels_to_meters(pixel_margin);
    let mut result = ContactResult::default();

    let physics_engine = physics_engine();

    //let prediction = Real::max(0.002, margin);
    let prediction = margin;

    if let Some(raw_shared_shape1) = physics_engine.get_shape(shape_info1.handle) {
        let skewed_shape1 = skew_shape(raw_shared_shape1, shape_info1.skew);
        let shared_shape1 = scale_shape(&skewed_shape1, shape_info1.scale);
        if let Some(raw_shared_shape2) = physics_engine.get_shape(shape_info2.handle) {
            let skewed_shape2 = skew_shape(raw_shared_shape2, shape_info2.skew);
            let shared_shape2 = scale_shape(&skewed_shape2, shape_info2.scale);

            let shape_transform1 = Isometry::new(position1, shape_info1.rotation);
            let shape_transform2 = Isometry::new(position2, shape_info2.rotation);

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
                result.within_margin = contact.dist > 0.0;
                result.collided = true;
                result.normal1 = contact.normal1.into_inner();
                result.normal2 = contact.normal2.into_inner();
                result.pixel_point1 = vector_meters_to_pixels(
                    (contact.point1 + contact.normal1.mul(prediction)).coords,
                );
                result.pixel_point2 = vector_meters_to_pixels(contact.point2.coords);
                return result;
            }
        }
    }
    result
}
