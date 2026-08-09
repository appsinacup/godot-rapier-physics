use std::ops::Mul;

use godot::global::godot_error;
use godot::global::godot_warn;
use rapier::parry::query::QueryDispatcher;
use rapier::parry::query::ShapeCastOptions;
use rapier::parry::query::ShapeCastStatus;
use rapier::prelude::*;

use crate::rapier_wrapper::physics_world::PhysicsWorld as RapierWrapperPhysicsWorld;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsCollisionObjects;
use crate::servers::rapier_physics_singleton::PhysicsIds;
use crate::spaces::rapier_space::RapierSpace;
pub struct RayHitInfo {
    pub pixel_position: Vector,
    pub normal: Vector,
    pub collider: ColliderHandle,
    pub user_data: UserData,
    pub feature: FeatureId,
}
impl RayHitInfo {
    pub fn default() -> RayHitInfo {
        RayHitInfo {
            pixel_position: Vector::ZERO,
            normal: Vector::ZERO,
            collider: ColliderHandle::invalid(),
            user_data: UserData::invalid_user_data(),
            feature: FeatureId::default(),
        }
    }
}
#[derive(Copy, Clone, Default)]
pub struct PointHitInfo {
    pub collider: ColliderHandle,
    pub user_data: UserData,
}
#[derive(Copy, Clone, Default, Debug)]
pub struct ShapeCastResult {
    pub collided: bool,
    pub toi: Real,
    pub toi_unsafe: Real,
    pub pixel_witness1: Vector,
    pub pixel_witness2: Vector,
    pub normal1: Vector,
    pub normal2: Vector,
    pub collider: ColliderHandle,
    pub user_data: UserData,
}
impl ShapeCastResult {
    fn new() -> ShapeCastResult {
        ShapeCastResult {
            collided: false,
            toi: 1.0,
            toi_unsafe: 1.0,
            collider: ColliderHandle::invalid(),
            pixel_witness1: Vector::ZERO,
            pixel_witness2: Vector::ZERO,
            normal1: Vector::ZERO,
            normal2: Vector::ZERO,
            user_data: UserData::invalid_user_data(),
        }
    }
}
#[derive(Copy, Clone, Default, Debug)]
pub struct WitnessPair {
    pub pixel_witness1: Vector,
    pub pixel_witness2: Vector,
}
impl WitnessPair {
    fn new() -> WitnessPair {
        WitnessPair {
            pixel_witness1: Vector::ZERO,
            pixel_witness2: Vector::ZERO,
        }
    }
}
#[derive(Default)]
pub struct ContactResult {
    pub collided: bool,
    pub within_margin: bool,
    pub pixel_distance: Real,
    pub pixel_point1: Vector,
    pub pixel_point2: Vector,
    pub normal1: Vector,
    pub normal2: Vector,
}
/// Floor for the distance parry is asked to look ahead by. Parry reports a contact only
/// strictly closer than this, so it has to stay above zero for shapes that rest exactly
/// touching to be seen at all. Callers still compare against their own margin.
const MIN_CONTACT_PREDICTION: Real = 0.002;

fn contact_prediction(margin: Real) -> Real {
    Real::max(MIN_CONTACT_PREDICTION, margin)
}
#[derive(Default)]
pub struct QueryExcludedInfo {
    pub query_collision_layer_mask: u32,
    pub query_canvas_instance_id: Option<u64>,
    pub query_pickable: bool,
    // Pointer to array of objects
    pub query_exclude: Vec<ColliderHandle>,
    pub query_exclude_size: usize,
    pub query_exclude_body: i64,
}
fn update_ray_hit_info(
    physics_world: &RapierWrapperPhysicsWorld,
    ray: &Ray,
    hit_from_inside: bool,
    handle: ColliderHandle,
    intersection: RayIntersection,
    length_current: &mut Real,
    hit_info: &mut RayHitInfo,
) -> bool {
    if intersection.time_of_impact > *length_current {
        return false;
    }
    if !hit_from_inside && intersection.time_of_impact == 0.0 {
        return false;
    }
    *length_current = intersection.time_of_impact;
    hit_info.pixel_position = ray.point_at(intersection.time_of_impact);
    if hit_from_inside && intersection.time_of_impact == 0.0 {
        hit_info.normal = Vector::ZERO;
    } else {
        hit_info.normal = intersection.normal;
    }
    hit_info.collider = handle;
    hit_info.user_data = physics_world.get_collider_user_data(handle);
    hit_info.feature = intersection.feature;
    true
}
#[derive(Default)]
pub struct ContactImpulseInfo {
    pub total_impulse: Vector,
    /// Sum of magnitudes, so opposing contacts do not cancel out.
    pub total_magnitude: Real,
    pub max_impulse: Real,
    pub max_normal: Vector,
    pub total_tangent_impulse: Real,
}
fn accumulate_contact_impulse(
    pair: &ContactPair,
    collider_handle: ColliderHandle,
    info: &mut ContactImpulseInfo,
) {
    // Manifold normals point from collider1 towards collider2.
    let sign = if pair.collider1 == collider_handle {
        -1.0
    } else {
        1.0
    };
    for manifold in pair.solver_manifolds() {
        let mut manifold_impulse = 0.0;
        for point in &manifold.points {
            manifold_impulse += point.data.impulse;
            info.total_tangent_impulse += tangent_impulse_magnitude(point.data.tangent_impulse);
        }
        info.total_impulse += manifold.data.normal * (manifold_impulse * sign);
        info.total_magnitude += manifold_impulse;
        if manifold_impulse > info.max_impulse {
            info.max_impulse = manifold_impulse;
            info.max_normal = manifold.data.normal * sign;
        }
    }
}
impl PhysicsEngine {
    pub fn body_get_contact_impulse(
        &self,
        world_handle: WorldHandle,
        body_handle: RigidBodyHandle,
        other: Option<RigidBodyHandle>,
    ) -> ContactImpulseInfo {
        let mut info = ContactImpulseInfo::default();
        let Some(physics_world) = self.get_world(world_handle) else {
            return info;
        };
        let rigid_body_set = &physics_world.physics_objects.rigid_body_set;
        let Some(body) = rigid_body_set.get(body_handle) else {
            return info;
        };
        let narrow_phase = &physics_world.physics_objects.narrow_phase;
        if let Some(other_handle) = other {
            let Some(other_body) = rigid_body_set.get(other_handle) else {
                return info;
            };
            // Indexing the contact graph directly beats walking every pair this body is in and
            // discarding the misses, which for ground or platform colliders can be hundreds.
            for &collider_handle in body.colliders() {
                for &other_collider in other_body.colliders() {
                    if let Some(pair) = narrow_phase.contact_pair(collider_handle, other_collider) {
                        accumulate_contact_impulse(pair, collider_handle, &mut info);
                    }
                }
            }
        } else {
            for &collider_handle in body.colliders() {
                for pair in narrow_phase.contact_pairs_with(collider_handle) {
                    accumulate_contact_impulse(pair, collider_handle, &mut info);
                }
            }
        }
        info
    }

    #[allow(clippy::too_many_arguments)]
    pub fn intersect_ray(
        &self,
        world_handle: WorldHandle,
        from: Vector,
        dir: Vector,
        length: Real,
        collide_with_body: bool,
        collide_with_area: bool,
        hit_from_inside: bool,
        hit_info: &mut RayHitInfo,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
        space: &RapierSpace,
    ) -> bool {
        let mut result = false;
        let Some(physics_world) = self.get_world(world_handle) else {
            return false;
        };
        let ray = Ray::new(from, dir);
        let mut filter = QueryFilter::new();
        if !collide_with_body {
            filter = filter.exclude_solids();
        }
        if !collide_with_area {
            filter = filter.exclude_sensors();
        }
        let predicate = |handle: ColliderHandle, collider: &Collider| -> bool {
            if space.is_handle_excluded_callback(
                handle,
                &physics_world.get_collider_user_data(handle),
                handle_excluded_info,
                physics_collision_objects,
                physics_ids,
            ) {
                return false;
            }
            hit_from_inside
                || !collider
                    .shape()
                    .contains_point(collider.position(), ray.origin)
        };
        filter.predicate = Some(&predicate);
        let mut length_current = Real::MAX;
        let query_pipeline = physics_world.physics_objects.broad_phase.as_query_pipeline(
            physics_world
                .physics_objects
                .narrow_phase
                .query_dispatcher(),
            &physics_world.physics_objects.rigid_body_set,
            &physics_world.physics_objects.collider_set,
            filter,
        );
        let broad_phase_empty = query_pipeline.bvh.is_empty();
        if let Some((handle, intersection)) =
            query_pipeline.cast_ray_and_get_normal(&ray, length, true)
            && update_ray_hit_info(
                physics_world,
                &ray,
                hit_from_inside,
                handle,
                intersection,
                &mut length_current,
                hit_info,
            )
        {
            result = true;
        }
        // Before the first physics step, Rapier's broad phase can still be empty.
        // Scan colliders read-only so early direct-space raycasts can hit loaded shapes.
        if !result && broad_phase_empty && !physics_world.physics_objects.collider_set.is_empty() {
            for (handle, collider) in physics_world.physics_objects.collider_set.iter() {
                if !filter.test(
                    &physics_world.physics_objects.rigid_body_set,
                    handle,
                    collider,
                ) {
                    continue;
                }
                if let Some(intersection) = collider.shape().cast_ray_and_get_normal(
                    collider.position(),
                    &ray,
                    length,
                    true,
                ) && update_ray_hit_info(
                    physics_world,
                    &ray,
                    hit_from_inside,
                    handle,
                    intersection,
                    &mut length_current,
                    hit_info,
                ) {
                    result = true;
                    if intersection.time_of_impact == 0.0 {
                        break;
                    }
                }
            }
        }
        result
    }

    #[allow(clippy::too_many_arguments)]
    pub fn intersect_point(
        &self,
        world_handle: WorldHandle,
        position: Vector,
        collide_with_body: bool,
        collide_with_area: bool,
        hit_info_array: *mut PointHitInfo,
        hit_info_length: usize,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
        space: &RapierSpace,
    ) -> usize {
        let mut cpt_hit = 0;
        if hit_info_length == 0 {
            return cpt_hit;
        }
        if let Some(physics_world) = self.get_world(world_handle) {
            let point = position;
            let mut filter = QueryFilter::new();
            if !collide_with_body {
                filter = filter.exclude_solids();
            }
            if !collide_with_area {
                filter = filter.exclude_sensors();
            }
            let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
                !space.is_handle_excluded_callback(
                    handle,
                    &physics_world.get_collider_user_data(handle),
                    handle_excluded_info,
                    physics_collision_objects,
                    physics_ids,
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
            if let Some(hit_info_slice) = hit_info_slice_opt {
                for (handle, _) in physics_world
                    .physics_objects
                    .broad_phase
                    .as_query_pipeline(
                        physics_world
                            .physics_objects
                            .narrow_phase
                            .query_dispatcher(),
                        &physics_world.physics_objects.rigid_body_set,
                        &physics_world.physics_objects.collider_set,
                        filter,
                    )
                    .intersect_point(point)
                {
                    // Callback called on each collider hit by the ray.
                    hit_info_slice[cpt_hit].collider = handle;
                    hit_info_slice[cpt_hit].user_data =
                        physics_world.get_collider_user_data(handle);
                    cpt_hit += 1;
                    // Stop if we filled the hit info array.
                    if cpt_hit >= hit_info_length {
                        break;
                    }
                }
            }
        }
        cpt_hit
    }

    #[cfg(feature = "dim2")]
    pub fn shape_collide(
        &self,
        shape_vel1: Vector,
        shape_info1: ShapeInfo,
        shape_vel2: Vector,
        shape_info2: ShapeInfo,
    ) -> ShapeCastResult {
        let mut result = ShapeCastResult::new();
        if let Some(raw_shared_shape1) = self.get_shape(shape_info1.handle) {
            let shared_shape1 = scale_shape(raw_shared_shape1, shape_info1);
            if let Some(raw_shared_shape2) = self.get_shape(shape_info2.handle) {
                let shared_shape2 = scale_shape(raw_shared_shape2, shape_info2);
                let shape_transform1 = shape_info1.transform;
                let shape_transform2 = shape_info2.transform;
                let shape_cast_options = ShapeCastOptions {
                    max_time_of_impact: 1.0,
                    stop_at_penetration: true,
                    compute_impact_geometry_on_penetration: true,
                    ..Default::default()
                };
                // Stationary shapes
                if shape_vel1.length_squared() < DEFAULT_EPSILON
                    && shape_vel2.length_squared() < DEFAULT_EPSILON
                {
                    let pos12 = shape_transform1.inv_mul(&shape_transform2);
                    // Parry only reports a contact strictly closer than the prediction
                    // distance, so querying with zero misses shapes that rest exactly
                    // touching, which is where the solver leaves them. The depth test below
                    // is what actually decides whether they collide.
                    let contact_result = separation_ray_query_dispatcher()
                        .contact(
                            &pos12,
                            shared_shape1.as_ref(),
                            shared_shape2.as_ref(),
                            contact_prediction(0.0),
                        )
                        .map(|contact| {
                            contact.map(|mut contact| {
                                contact.transform_by_mut(&shape_transform1, &shape_transform2);
                                contact
                            })
                        });
                    match contact_result {
                        Ok(None) => {}
                        Ok(Some(contact)) => {
                            // the distance is negative if there is intersection
                            if contact.dist <= 0.0 {
                                result.toi = 0.0;
                                result.collided = true;
                                // parry::query::contact() returns results in world space
                                result.normal1 = contact.normal1;
                                result.normal2 = contact.normal2;
                                result.pixel_witness1 = contact.point1;
                                result.pixel_witness2 = contact.point2;
                            }
                        }
                        Err(err) => {
                            godot_error!("contact error: {:?}", err);
                        }
                    }
                    return result;
                }
                let pos12 = shape_transform1.inv_mul(&shape_transform2);
                let vel12 = shape_transform1.rotation.inverse() * (shape_vel2 - shape_vel1);
                let toi_result = separation_ray_query_dispatcher().cast_shapes(
                    &pos12,
                    vel12,
                    shared_shape1.as_ref(),
                    shared_shape2.as_ref(),
                    shape_cast_options,
                );
                match toi_result {
                    Ok(None) => {}
                    Ok(Some(hit)) => {
                        if hit.status == ShapeCastStatus::Failed
                            || hit.status == ShapeCastStatus::OutOfIterations
                        {
                            godot_warn!("shape collide status warn: {:?}", hit.status);
                        }
                        result.collided = true;
                        result.toi = hit.time_of_impact;
                        // parry::query::cast_shapes() returns results in each shape's local space
                        result.normal1 = shape_transform1.rotation * hit.normal1;
                        result.normal2 = shape_transform2.rotation * hit.normal2;
                        result.pixel_witness1 =
                            shape_transform1 * hit.witness1 + shape_vel1 * hit.time_of_impact;
                        result.pixel_witness2 =
                            shape_transform2 * hit.witness2 + shape_vel2 * hit.time_of_impact;
                    }
                    Err(err) => {
                        godot_error!("toi error: {:?}", err);
                    }
                }
            }
        }
        result
    }

    #[allow(clippy::too_many_arguments)]
    pub fn shape_find_intersections(
        &self,
        world_handle: WorldHandle,
        shape_vel: Vector,
        shape_info: ShapeInfo,
        margin: Real,
        collide_with_body: bool,
        collide_with_area: bool,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
        space: &RapierSpace,
        results: &mut Vec<WitnessPair>,
        max_results: usize,
    ) -> usize {
        let mut result_count = 0;
        if max_results == 0 {
            return result_count;
        }
        let Some(raw_shared_shape) = self.get_shape(shape_info.handle) else {
            return result_count;
        };
        let Some(physics_world) = self.get_world(world_handle) else {
            return result_count;
        };
        let shared_shape = scale_shape(raw_shared_shape, shape_info);
        let shape_transform = shape_info.transform;
        let mut filter = QueryFilter::new();
        if !collide_with_body {
            filter = filter.exclude_solids();
        }
        if !collide_with_area {
            filter = filter.exclude_sensors();
        }
        let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
            !space.is_handle_excluded_callback(
                handle,
                &physics_world.get_collider_user_data(handle),
                handle_excluded_info,
                physics_collision_objects,
                physics_ids,
            )
        };
        filter.predicate = Some(&predicate);
        let velocity_size = shape_vel.length();
        let mut shape_transform_with_motion = shape_transform;
        // If we have velocity, then we cast our shape forward; shape_transform_with_motion will be updated
        // to match the position it will occupy at the end of the tick, or its hit location if it hits something this tick.
        if velocity_size > DEFAULT_EPSILON {
            let shape_cast_options = ShapeCastOptions {
                max_time_of_impact: 1.0,
                stop_at_penetration: true,
                compute_impact_geometry_on_penetration: true,
                target_distance: margin,
            };
            if let Some((_, hit)) = physics_world
                .physics_objects
                .broad_phase
                .as_query_pipeline(
                    physics_world
                        .physics_objects
                        .narrow_phase
                        .query_dispatcher(),
                    &physics_world.physics_objects.rigid_body_set,
                    &physics_world.physics_objects.collider_set,
                    filter,
                )
                .cast_shape(
                    &shape_transform,
                    shape_vel,
                    shared_shape.as_ref(),
                    shape_cast_options,
                )
            {
                if hit.status == ShapeCastStatus::Failed
                    || hit.status == ShapeCastStatus::OutOfIterations
                {
                    godot_warn!("shape casting status warn: {:?}", hit.status);
                }
                shape_transform_with_motion.translation += shape_vel * hit.time_of_impact;
            }
        }
        let mut manifolds: Vec<ContactManifold> = Vec::new();
        // Candidates come from a loosened AABB rather than a strict overlap test, which would
        // reject shapes that sit inside the margin without touching and so never let the
        // manifold step below see them.
        let query_aabb = shared_shape
            .compute_aabb(&shape_transform_with_motion)
            .loosened(contact_prediction(margin));
        for (_collider_handle, collider) in physics_world
            .physics_objects
            .broad_phase
            .as_query_pipeline(
                physics_world
                    .physics_objects
                    .narrow_phase
                    .query_dispatcher(),
                &physics_world.physics_objects.rigid_body_set,
                &physics_world.physics_objects.collider_set,
                filter,
            )
            .intersect_aabb_conservative(query_aabb)
        {
            manifolds.clear();
            let pos12 = shape_transform_with_motion.inv_mul(collider.position());
            let _ = physics_world
                .physics_objects
                .narrow_phase
                .query_dispatcher()
                .contact_manifolds(
                    &pos12,
                    shared_shape.as_ref(),
                    collider.shape(),
                    contact_prediction(margin),
                    &mut manifolds,
                    &mut None,
                );
            for m in &manifolds {
                if result_count >= max_results {
                    break;
                }
                for contact in &m.points {
                    // Parry only reports contacts strictly closer than the prediction
                    // distance, so the manifold is generated with a floor and the margin is
                    // applied here instead.
                    if contact.dist > margin {
                        continue;
                    }
                    let contact_p1 = shape_transform_with_motion * contact.local_p1;
                    let contact_p2 = collider.position() * contact.local_p2;
                    let mut this_contact: WitnessPair = WitnessPair::new();
                    this_contact.pixel_witness1 = contact_p1;
                    this_contact.pixel_witness2 = contact_p2;
                    results.push(this_contact);
                    result_count += 1;
                    if result_count >= max_results {
                        break;
                    }
                }
            }
        }
        result_count
    }

    #[allow(clippy::too_many_arguments)]
    pub fn shape_casting(
        &self,
        world_handle: WorldHandle,
        shape_vel: Vector,
        shape_info: ShapeInfo,
        margin: Real,
        collide_with_body: bool,
        collide_with_area: bool,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
        space: &RapierSpace,
        needs_exact: bool,
    ) -> Vec<ShapeCastResult> {
        let mut results: Vec<ShapeCastResult> = Vec::new();
        if let Some(raw_shared_shape) = self.get_shape(shape_info.handle) {
            let shared_shape = scale_shape(raw_shared_shape, shape_info);
            if let Some(physics_world) = self.get_world(world_handle) {
                let shape_transform = shape_info.transform;
                let mut filter = QueryFilter::new();
                if !collide_with_body {
                    filter = filter.exclude_solids();
                }
                if !collide_with_area {
                    filter = filter.exclude_sensors();
                }
                let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
                    !space.is_handle_excluded_callback(
                        handle,
                        &physics_world.get_collider_user_data(handle),
                        handle_excluded_info,
                        physics_collision_objects,
                        physics_ids,
                    )
                };
                filter.predicate = Some(&predicate);
                let velocity_size = shape_vel.length();
                if velocity_size < DEFAULT_EPSILON {
                    // Candidates come from an AABB loosened by the margin, not from a strict
                    // overlap test: a shape resting just outside the other still counts as a hit
                    // when it is within the margin, which is what Godot's queries report.
                    let prediction = contact_prediction(margin);
                    let query_aabb = shared_shape
                        .compute_aabb(&shape_transform)
                        .loosened(prediction);
                    for (collider_handle, collider) in physics_world
                        .physics_objects
                        .broad_phase
                        .as_query_pipeline(
                            physics_world
                                .physics_objects
                                .narrow_phase
                                .query_dispatcher(),
                            &physics_world.physics_objects.rigid_body_set,
                            &physics_world.physics_objects.collider_set,
                            filter,
                        )
                        .intersect_aabb_conservative(query_aabb)
                    {
                        let pos12 = shape_transform.inv_mul(collider.position());
                        match physics_world
                            .physics_objects
                            .narrow_phase
                            .query_dispatcher()
                            .contact(&pos12, shared_shape.as_ref(), collider.shape(), prediction)
                        {
                            // Parry only reports a contact strictly closer than the prediction
                            // distance, so querying with `margin` alone misses shapes that rest
                            // exactly touching, which is where rapier's solver leaves them.
                            Ok(Some(contact)) if contact.dist <= margin => {
                                let mut result = ShapeCastResult::new();
                                result.collided = true;
                                result.collider = collider_handle;
                                result.user_data =
                                    physics_world.get_collider_user_data(collider_handle);
                                result.toi = 0.0;
                                // parry returns contacts in each shape's local space.
                                result.normal1 = shape_transform.rotation * contact.normal1;
                                result.normal2 = collider.rotation() * contact.normal2;
                                result.pixel_witness1 = shape_transform * contact.point1;
                                result.pixel_witness2 = collider.position() * contact.point2;
                                results.push(result);
                                if results.len()
                                    >= crate::servers::rapier_project_settings::motion_settings()
                                        .max_shape_cast_results
                                {
                                    break;
                                }
                            }
                            Ok(_) => {}
                            Err(err) => godot_error!("contact error: {:?}", err),
                        }
                    }
                } else {
                    let shape_cast_options = ShapeCastOptions {
                        max_time_of_impact: 1.0,
                        stop_at_penetration: true,
                        compute_impact_geometry_on_penetration: true,
                        target_distance: margin,
                    };
                    let mut cast_excludes: std::collections::HashSet<ColliderHandle> =
                        std::collections::HashSet::new();
                    loop {
                        let predicate = |handle: ColliderHandle, _collider: &Collider| -> bool {
                            !cast_excludes.contains(&handle)
                                && !space.is_handle_excluded_callback(
                                    handle,
                                    &physics_world.get_collider_user_data(handle),
                                    handle_excluded_info,
                                    physics_collision_objects,
                                    physics_ids,
                                )
                        };
                        let mut cast_filter = QueryFilter::new();
                        if !collide_with_body {
                            cast_filter = cast_filter.exclude_solids();
                        }
                        if !collide_with_area {
                            cast_filter = cast_filter.exclude_sensors();
                        }
                        cast_filter.predicate = Some(&predicate);
                        let Some((collider_handle, hit)) = physics_world
                            .physics_objects
                            .broad_phase
                            .as_query_pipeline(
                                physics_world
                                    .physics_objects
                                    .narrow_phase
                                    .query_dispatcher(),
                                &physics_world.physics_objects.rigid_body_set,
                                &physics_world.physics_objects.collider_set,
                                cast_filter,
                            )
                            .cast_shape(
                                &shape_transform,
                                shape_vel,
                                shared_shape.as_ref(),
                                shape_cast_options,
                            )
                        else {
                            break;
                        };
                        if hit.status == ShapeCastStatus::Failed
                            || hit.status == ShapeCastStatus::OutOfIterations
                        {
                            godot_warn!("shape casting status warn: {:?}", hit.status);
                        }
                        if needs_exact && hit.time_of_impact == 0.0 {
                            cast_excludes.insert(collider_handle);
                            continue;
                        }
                        if let Some(collider) = physics_world
                            .physics_objects
                            .collider_set
                            .get(collider_handle)
                        {
                            let mut result = ShapeCastResult::new();
                            result.collided = true;
                            result.collider = collider_handle;
                            result.user_data =
                                physics_world.get_collider_user_data(collider_handle);
                            result.toi = hit.time_of_impact;
                            result.toi_unsafe = hit.time_of_impact;

                            // In QueryPipeline::cast_shapes(),
                            // world shapes is the hidden first parameter, and the scanner shape is the second,
                            // so the order of normals and witnesses needs to be swapped
                            // result.pixel_witness1 <- hit.witness2 transformed from scanner shape's local space
                            // result.pixel_witness2 <- hit.witness1 (world shape's local space, no need to transform)
                            result.normal1 = shape_transform.rotation * hit.normal2;
                            result.normal2 = hit.normal1;
                            result.pixel_witness1 =
                                shape_transform * hit.witness2 + shape_vel * hit.time_of_impact;
                            result.pixel_witness2 = hit.witness1;
                            // `toi` is the furthest the shape can advance without colliding and
                            // `toi_unsafe` the first fraction where it does, so the two must
                            // not be equal. The cast stops on contact, leaving no separation
                            // to close, so advancing by one contact prediction is enough to
                            // overlap at any approach angle.
                            if needs_exact {
                                let mut hit_transform = shape_transform;
                                hit_transform.translation += shape_vel * hit.time_of_impact;
                                let pos12 = hit_transform.inv_mul(collider.position());
                                let separation = physics_world
                                    .physics_objects
                                    .narrow_phase
                                    .query_dispatcher()
                                    .distance(&pos12, shared_shape.as_ref(), collider.shape())
                                    .unwrap_or(0.0)
                                    .max(0.0);
                                result.toi_unsafe = (result.toi
                                    + (separation + MIN_CONTACT_PREDICTION) / velocity_size)
                                    .min(1.0);
                            }
                            results.push(result);
                        } else {
                            godot_error!("collider not found");
                        }
                        cast_excludes.insert(collider_handle);
                        if needs_exact
                            || results.len()
                                >= crate::servers::rapier_project_settings::motion_settings()
                                    .max_shape_cast_results
                        {
                            break;
                        }
                    }
                }
            }
        }
        results
    }

    #[allow(clippy::too_many_arguments)]
    pub fn intersect_aabb(
        &self,
        world_handle: WorldHandle,
        aabb_min: Vector,
        aabb_max: Vector,
        collide_with_body: bool,
        collide_with_area: bool,
        hit_info_slice: &mut [PointHitInfo],
        max_results: usize,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
        space: &RapierSpace,
    ) -> usize {
        let mut cpt_hit = 0;
        if let Some(physics_world) = self.get_world(world_handle) {
            let aabb = Aabb {
                mins: aabb_min,
                maxs: aabb_max,
            };
            for (handle, _) in physics_world
                .physics_objects
                .broad_phase
                .as_query_pipeline(
                    physics_world
                        .physics_objects
                        .narrow_phase
                        .query_dispatcher(),
                    &physics_world.physics_objects.rigid_body_set,
                    &physics_world.physics_objects.collider_set,
                    QueryFilter::default(),
                )
                .intersect_aabb_conservative(aabb)
            {
                let mut valid_hit = false;
                if let Some(collider) = physics_world.physics_objects.collider_set.get(handle) {
                    // type filter
                    if (collider.is_sensor() && collide_with_area)
                        || (!collider.is_sensor() && collide_with_body)
                    {
                        valid_hit = true;
                    }
                    if valid_hit {
                        valid_hit = !space.is_handle_excluded_callback(
                            handle,
                            &physics_world.get_collider_user_data(handle),
                            handle_excluded_info,
                            physics_collision_objects,
                            physics_ids,
                        );
                    }
                }
                if !valid_hit {
                    continue;
                }
                // Callback called on each collider hit by the ray.
                hit_info_slice[cpt_hit].collider = handle;
                hit_info_slice[cpt_hit].user_data = physics_world.get_collider_user_data(handle);
                cpt_hit += 1;
                if cpt_hit >= max_results {
                    break;
                }
            }
        }
        cpt_hit
    }

    pub fn shapes_contact(
        &self,
        shape_info1: ShapeInfo,
        shape_info2: ShapeInfo,
        margin: Real,
    ) -> ContactResult {
        let mut result = ContactResult::default();
        let prediction = contact_prediction(margin);
        if let Some(raw_shared_shape1) = self.get_shape(shape_info1.handle) {
            let shared_shape1 = scale_shape(raw_shared_shape1, shape_info1);
            if let Some(raw_shared_shape2) = self.get_shape(shape_info2.handle) {
                let shared_shape2 = scale_shape(raw_shared_shape2, shape_info2);
                let shape_transform1 = shape_info1.transform;
                let shape_transform2 = shape_info2.transform;
                let pos12 = shape_transform1.inv_mul(&shape_transform2);
                match separation_ray_query_dispatcher()
                    .contact(
                        &pos12,
                        shared_shape1.as_ref(),
                        shared_shape2.as_ref(),
                        prediction,
                    )
                    .map(|contact| {
                        contact.map(|mut contact| {
                            contact.transform_by_mut(&shape_transform1, &shape_transform2);
                            contact
                        })
                    }) {
                    Ok(None) => {}
                    Ok(Some(contact)) => {
                        // the distance is negative if there is intersection
                        // and positive if the objects are separated by distance less than margin
                        result.pixel_distance = contact.dist;
                        result.within_margin = contact.dist > 0.0;
                        result.collided = true;
                        result.normal1 = contact.normal1;
                        result.normal2 = contact.normal2;
                        result.pixel_point1 = contact.point1 + contact.normal1.mul(prediction);
                        result.pixel_point2 = contact.point2;
                    }
                    Err(err) => {
                        godot_error!("Shape Contact Error: {:?}", err);
                    }
                }
            }
        }
        result
    }
}
