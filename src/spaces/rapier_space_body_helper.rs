use std::ops::Deref;

use bodies::rapier_collision_object_base::RapierCollisionObjectBase;
use godot::classes::native::ObjectId;
use godot::classes::physics_server_2d::BodyMode;
use godot::prelude::*;
use rapier::geometry::ColliderHandle;
use rapier::math::DEFAULT_EPSILON;
use rapier::math::Real;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;
use servers::rapier_physics_singleton::PhysicsIds;
use servers::rapier_physics_singleton::PhysicsShapes;
use shapes::rapier_shape::IRapierShape;
use shapes::rapier_shape::RapierShape;

use super::RapierDirectSpaceState;
use super::rapier_space::RapierSpace;
use crate::bodies::rapier_body::RapierBody;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::types::*;
use crate::*;
const TEST_MOTION_MARGIN: Real = 1e-4;
const TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR: Real = 0.05;
const BODY_MOTION_RECOVER_ATTEMPTS: i32 = 4;
#[cfg(feature = "dim2")]
const BODY_MOTION_RECOVER_RATIO: Real = 0.4;
#[cfg(feature = "dim3")]
const BODY_MOTION_RECOVER_RATIO: Real = 0.5;
const MAX_EXCLUDED_SHAPE_PAIRS: usize = 32;
#[cfg(feature = "dim2")]
const BODY_MOTION_CAST_ITERATIONS: i32 = 8;
#[cfg(feature = "dim3")]
const BODY_MOTION_CAST_ITERATIONS: i32 = 8;
#[cfg(feature = "dim2")]
const MIN_MOTION_THRESHOLD: Real = 1e-3;
#[cfg(feature = "dim3")]
const MIN_MOTION_THRESHOLD: Real = 1e-4;
#[cfg(feature = "dim2")]
const MOTION_EPSILON: Real = 1e-3;
#[cfg(feature = "dim3")]
const MOTION_EPSILON: Real = 1e-4;
#[cfg(feature = "dim2")]
const BLOCKED_MOTION_EPSILON: Real = 5e-3;
#[cfg(feature = "dim3")]
const BLOCKED_MOTION_EPSILON: Real = 1e-4;
#[cfg(feature = "dim2")]
const MIN_RECOVERY_THRESHOLD: Real = 1e-3;
#[cfg(feature = "dim3")]
const MIN_RECOVERY_THRESHOLD: Real = 1e-4;
#[cfg(feature = "dim2")]
const STUCK_PENETRATION_THRESHOLD: Real = 0.1;
#[cfg(feature = "dim3")]
const STUCK_PENETRATION_THRESHOLD: Real = 0.01;
const NORMAL_EPSILON: Real = 0.01;
#[derive(Clone, Copy)]
struct ExcludedShapePair {
    local_shape_index: usize,
    collision_object_rid: Rid,
    collision_shape_index: usize,
}
impl Default for ExcludedShapePair {
    fn default() -> Self {
        Self {
            local_shape_index: 0,
            collision_object_rid: Rid::Invalid,
            collision_shape_index: 0,
        }
    }
}
fn blocked_motion_tolerance(margin: Real) -> Real {
    MOTION_EPSILON
        .max(BLOCKED_MOTION_EPSILON)
        .max(margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR)
}
fn clamp_near_zero_safe_motion(motion: Vector, margin: Real, safe_fraction: &mut Real) {
    let tolerance = blocked_motion_tolerance(margin);
    if *safe_fraction < 1.0 && motion.length() * *safe_fraction < tolerance {
        *safe_fraction = 0.0;
    }
}
fn clamp_near_zero_blocked_travel(motion: Vector, margin: Real, travel: &mut Vector) {
    if motion.dot(*travel) > 0.0 && travel.length() < blocked_motion_tolerance(margin) {
        *travel = Vector::default();
    }
}
fn recover_motion_from_contacts(
    contacts: &[Vector; 64],
    priorities: &[Real; 32],
    contact_count: usize,
    min_contact_depth: Real,
) -> Vector {
    let total_priority: Real = priorities.iter().take(contact_count).sum();
    let inv_total_weight = if total_priority.abs() <= DEFAULT_EPSILON {
        1.0
    } else {
        contact_count as Real / total_priority
    };
    let mut recover_motion = Vector::default();
    for i in 0..contact_count {
        let a = contacts[i * 2];
        let b = contacts[i * 2 + 1];
        if let Some(n) = (a - b).try_normalized() {
            let d = n.dot(b);
            let depth = n.dot(a + recover_motion) - d;
            if depth > min_contact_depth + DEFAULT_EPSILON {
                recover_motion -= n
                    * (depth - min_contact_depth)
                    * BODY_MOTION_RECOVER_RATIO
                    * priorities[i]
                    * inv_total_weight;
            }
        }
    }
    recover_motion
}
#[cfg(feature = "dim2")]
fn rect_contains_point(rect: Rect, point: Vector, tolerance: Real) -> bool {
    let end = rect.position + rect.size;
    let min_x = rect.position.x.min(end.x) - tolerance;
    let max_x = rect.position.x.max(end.x) + tolerance;
    let min_y = rect.position.y.min(end.y) - tolerance;
    let max_y = rect.position.y.max(end.y) + tolerance;
    point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y
}
#[cfg(feature = "dim3")]
fn rect_contains_point(rect: Rect, point: Vector, tolerance: Real) -> bool {
    let end = rect.position + rect.size;
    let min_x = rect.position.x.min(end.x) - tolerance;
    let max_x = rect.position.x.max(end.x) + tolerance;
    let min_y = rect.position.y.min(end.y) - tolerance;
    let max_y = rect.position.y.max(end.y) + tolerance;
    let min_z = rect.position.z.min(end.z) - tolerance;
    let max_z = rect.position.z.max(end.z) + tolerance;
    point.x >= min_x
        && point.x <= max_x
        && point.y >= min_y
        && point.y <= max_y
        && point.z >= min_z
        && point.z <= max_z
}
#[cfg(feature = "dim2")]
fn shape_contact_aabb(shape: &RapierShape, transform: Transform) -> Rect {
    let mut aabb = shape.get_base().get_aabb(transform.origin);
    let mut local_transform = transform;
    local_transform.origin = Vector::default();
    aabb.size = transform_scale(&local_transform) * aabb.size;
    aabb
}
#[cfg(feature = "dim3")]
fn shape_contact_aabb(shape: &RapierShape, transform: Transform) -> Rect {
    let local_aabb = shape.get_base().get_aabb(Vector::default());
    let local_end = local_aabb.end();
    let mut min = Vector::new(Real::INFINITY, Real::INFINITY, Real::INFINITY);
    let mut max = Vector::new(Real::NEG_INFINITY, Real::NEG_INFINITY, Real::NEG_INFINITY);
    for x in [local_aabb.position.x, local_end.x] {
        for y in [local_aabb.position.y, local_end.y] {
            for z in [local_aabb.position.z, local_end.z] {
                let point = transform * Vector::new(x, y, z);
                min = min.coord_min(point);
                max = max.coord_max(point);
            }
        }
    }
    Rect::from_corners(min, max)
}
#[cfg(feature = "dim2")]
fn is_valid_recovery_contact(
    moving_shape: &RapierShape,
    moving_shape_transform: Transform,
    _collision_shape: &RapierShape,
    contact: &ContactResult,
    margin: Real,
) -> bool {
    let contact_point = vector_to_godot(contact.pixel_point2);
    let contact_aabb = shape_contact_aabb(moving_shape, moving_shape_transform).grow(margin + 1.0);
    rect_contains_point(contact_aabb, contact_point, DEFAULT_EPSILON)
}
#[cfg(feature = "dim3")]
fn is_valid_recovery_contact(
    moving_shape: &RapierShape,
    moving_shape_transform: Transform,
    _collision_shape: &RapierShape,
    contact: &ContactResult,
    margin: Real,
) -> bool {
    let contact_point = vector_to_godot(contact.pixel_point2);
    let contact_aabb = shape_contact_aabb(moving_shape, moving_shape_transform).grow(margin + 0.01);
    rect_contains_point(contact_aabb, contact_point, DEFAULT_EPSILON)
}
#[cfg(feature = "dim2")]
fn reset_body_motion_result(result: &mut PhysicsServerExtensionMotionResult) {
    result.travel = Vector::default();
    result.remainder = Vector::default();
    result.collision_point = Vector::default();
    result.collision_normal = Vector::default();
    result.collider_velocity = Vector::default();
    result.collision_depth = 0.0;
    result.collision_safe_fraction = 0.0;
    result.collision_unsafe_fraction = 0.0;
    result.collision_local_shape = 0;
    result.collider_id = ObjectId { id: 0 };
    result.collider = Rid::Invalid;
    result.collider_shape = 0;
}
#[cfg(feature = "dim3")]
fn reset_body_motion_result(result: &mut PhysicsServerExtensionMotionResult) {
    use godot::classes::native::PhysicsServer3DExtensionMotionCollision;
    result.travel = Vector::default();
    result.remainder = Vector::default();
    result.collision_depth = 0.0;
    result.collision_safe_fraction = 0.0;
    result.collision_unsafe_fraction = 0.0;
    result.collisions = core::array::from_fn(|_| PhysicsServer3DExtensionMotionCollision {
        position: Vector::default(),
        normal: Vector::default(),
        collider_velocity: Vector::default(),
        collider_angular_velocity: Vector::default(),
        depth: 0.0,
        local_shape: 0,
        collider_id: ObjectId { id: 0 },
        collider: Rid::Invalid,
        collider_shape: 0,
    });
    result.collision_count = 0;
}
fn finish_small_body_motion(
    motion: Vector,
    result: &mut PhysicsServerExtensionMotionResult,
) -> bool {
    if motion.length() >= MIN_MOTION_THRESHOLD {
        return false;
    }
    result.travel = Vector::default();
    result.remainder = Vector::default();
    result.collision_safe_fraction = 1.0;
    result.collision_unsafe_fraction = 1.0;
    true
}
fn should_collect_body_motion_collision(
    recovery_as_collision: bool,
    recovered: bool,
    safe_fraction: Real,
) -> bool {
    (recovery_as_collision && recovered) || safe_fraction < 1.0
}
fn finish_body_motion_result(
    result: &mut PhysicsServerExtensionMotionResult,
    recover_motion: Vector,
    motion: Vector,
    margin: Real,
    safe_fraction: Real,
    unsafe_fraction: Real,
    collided: bool,
) {
    if collided {
        let mut travel = recover_motion + motion * safe_fraction;
        clamp_near_zero_blocked_travel(motion, margin, &mut travel);
        result.travel += travel;
        result.remainder = motion - motion * safe_fraction;
        result.collision_safe_fraction = safe_fraction;
        result.collision_unsafe_fraction = unsafe_fraction;
    } else {
        result.travel += recover_motion + motion;
        result.remainder = Vector::default();
        result.collision_depth = 0.0;
        result.collision_safe_fraction = 1.0;
        result.collision_unsafe_fraction = 1.0;
    }
}
fn contact_depth(distance: Real, margin: Real) -> Real {
    margin - distance
}
fn is_contact_depth_allowed(distance: Real, margin: Real, min_allowed_depth: Real) -> bool {
    contact_depth(distance, margin) >= min_allowed_depth
}
fn is_motion_blocked_by_contact(contact: &ContactResult, motion: Vector) -> bool {
    let Some(motion_normal) = motion.try_normalized() else {
        return true;
    };
    let contact_normal = vector_to_godot(contact.normal2);
    contact_normal.dot(motion_normal) < -NORMAL_EPSILON
}
#[cfg(feature = "dim2")]
fn is_end_contact_relevant(contact: &ContactResult, motion: Vector) -> bool {
    contact.collided && is_motion_blocked_by_contact(contact, motion)
}
#[cfg(feature = "dim3")]
fn is_end_contact_relevant(contact: &ContactResult, _motion: Vector) -> bool {
    contact.collided
}
fn contact_collision_point(contact: &ContactResult) -> rapier::prelude::Vector {
    contact.pixel_point2
}
impl RapierSpace {
    pub fn is_handle_excluded_callback(
        &self,
        collider_handle: ColliderHandle,
        user_data: &UserData,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> bool {
        for exclude_index in 0..handle_excluded_info.query_exclude_size {
            if handle_excluded_info.query_exclude[exclude_index] == collider_handle {
                return true;
            }
        }
        let (collision_object_2d, _) =
            RapierCollisionObjectBase::get_collider_user_data(user_data, physics_ids);
        let Some(collision_object_2d) = physics_collision_objects.get(&collision_object_2d) else {
            return false;
        };
        let collision_object_base = collision_object_2d.get_base();
        let canvas_excluded = match handle_excluded_info.query_canvas_instance_id {
            Some(query_id) => collision_object_base.get_canvas_instance_id() != query_id,
            None => false,
        };
        let layer_excluded = collision_object_base.get_collision_layer()
            & handle_excluded_info.query_collision_layer_mask
            == 0;
        let rid_excluded = handle_excluded_info.query_exclude_body
            == collision_object_base.get_rid().to_u64() as i64;
        let pickable_excluded =
            handle_excluded_info.query_pickable && !collision_object_base.get_pickable();
        if canvas_excluded || layer_excluded || rid_excluded || pickable_excluded {
            return true;
        }
        let Some(direct_space) = self.get_direct_state() else {
            return false;
        };
        let Ok(direct_state) = direct_space.clone().try_cast::<RapierDirectSpaceState>() else {
            return false;
        };
        let direct_space = direct_state.deref();
        direct_space.is_body_excluded_from_query(collision_object_base.get_rid())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn test_body_motion(
        &self,
        body: &RapierBody,
        from: Transform,
        motion: Vector,
        margin: Real,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: &mut PhysicsServerExtensionMotionResult,
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_ids: &PhysicsIds,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        reset_body_motion_result(result);
        // Skip processing if motion is too small (prevents infinite micro-adjustments)
        if finish_small_body_motion(motion, result) {
            return false;
        }
        let mut body_transform = from; // Because body_transform needs to be modified during recovery
        // Step 1: recover motion.
        // Expand the body colliders by the margin (grow) and check if now it collides with a collider,
        // if yes, "recover" / "push" out of this collider
        let mut recover_motion = Vector::default();
        let margin = Real::max(margin, TEST_MOTION_MARGIN);
        let min_allowed_depth = motion
            .length()
            .min(margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR);
        let mut excluded_shape_pairs = [ExcludedShapePair::default(); MAX_EXCLUDED_SHAPE_PAIRS];
        let mut excluded_shape_pair_count = 0;
        let recovered = self.body_motion_recover(
            body,
            &mut body_transform,
            motion,
            margin,
            &mut recover_motion,
            &mut excluded_shape_pairs,
            &mut excluded_shape_pair_count,
            physics_engine,
            physics_shapes,
            physics_ids,
            physics_collision_objects,
        );
        // Step 2: Cast motion.
        // Try to to find what is the possible motion (how far it can move, it's a shapecast, when you try to find the safe point (max you can move without collision ))
        let mut best_safe = 1.0;
        let mut best_unsafe = 1.0;
        let mut best_body_shape = -1;
        self.cast_motion(
            body,
            &body_transform,
            motion,
            collide_separation_ray,
            self.get_contact_max_allowed_penetration(),
            margin,
            &mut best_safe,
            &mut best_unsafe,
            &mut best_body_shape,
            &excluded_shape_pairs,
            excluded_shape_pair_count,
            physics_engine,
            physics_shapes,
            physics_ids,
            physics_collision_objects,
        );
        // Far from the origin, parry can report a tiny safe fraction for perpendicular wall pushes.
        // Godot treats this as no travel, so keep the collision and clamp only the safe motion.
        clamp_near_zero_safe_motion(motion, margin, &mut best_safe);
        // Step 3: Rest Info
        // Apply the motion and fill the collision information
        let mut collided = false;
        if should_collect_body_motion_collision(recovery_as_collision, recovered, best_safe) {
            if best_safe >= 1.0 {
                best_body_shape = -1; //no best shape with cast, reset to -1
            }
            // Get the rest info in unsafe advance
            let unsafe_motion = motion * best_unsafe;
            body_transform.origin += unsafe_motion;
            collided = self.body_motion_collide(
                body,
                &body_transform,
                motion,
                best_body_shape,
                margin,
                min_allowed_depth,
                result,
                &excluded_shape_pairs,
                excluded_shape_pair_count,
                physics_engine,
                physics_shapes,
                physics_ids,
                physics_collision_objects,
            );
        }
        finish_body_motion_result(
            result,
            recover_motion,
            motion,
            margin,
            best_safe,
            best_unsafe,
            collided,
        );
        collided
    }

    #[allow(clippy::too_many_arguments)]
    pub fn rapier_intersect_aabb(
        &self,
        aabb: Rect,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: &mut [PointHitInfo],
        max_results: usize,
        exclude_body: Rid,
        physics_engine: &PhysicsEngine,
        physics_collision_objects: &PhysicsCollisionObjects,
        physics_ids: &PhysicsIds,
    ) -> i32 {
        if max_results < 1 {
            return 0;
        }
        let rect_begin = aabb.position - aabb.size;
        let rect_end = aabb.end();
        let mut handle_excluded_info = QueryExcludedInfo::default();
        let mut query_exclude = Vec::new();
        query_exclude.resize_with(max_results, Default::default);
        handle_excluded_info.query_exclude = query_exclude;
        handle_excluded_info.query_collision_layer_mask = collision_mask;
        handle_excluded_info.query_exclude_size = 0;
        handle_excluded_info.query_exclude_body = exclude_body.to_u64() as i64;
        physics_engine.intersect_aabb(
            self.get_state().get_id(),
            vector_to_rapier(rect_begin),
            vector_to_rapier(rect_end),
            collide_with_bodies,
            collide_with_areas,
            results,
            max_results,
            &handle_excluded_info,
            physics_collision_objects,
            physics_ids,
            self,
        ) as i32
    }

    #[allow(clippy::too_many_arguments)]
    fn body_motion_recover(
        &self,
        p_body: &RapierBody,
        p_transform: &mut Transform,
        p_motion: Vector,
        p_margin: f32,
        p_recover_motion: &mut Vector,
        excluded_shape_pairs: &mut [ExcludedShapePair; MAX_EXCLUDED_SHAPE_PAIRS],
        excluded_shape_pair_count: &mut usize,
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_ids: &PhysicsIds,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        let shape_count = p_body.get_base().get_shape_count();
        if shape_count < 1 {
            return false;
        }
        let min_contact_depth = p_margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;
        let mut recovered = false;
        let mut recover_attempts = BODY_MOTION_RECOVER_ATTEMPTS;
        let body_aabb = p_body.get_aabb(physics_shapes, physics_ids);
        loop {
            let mut results = [PointHitInfo::default(); 32];
            let mut sr = [Vector::default(); 64]; // Store contact points (2 per contact, max 32 contacts)
            let mut priorities = [0.0; 32];
            let mut contact_count = 0;
            *excluded_shape_pair_count = 0; // Reset for this iteration
            // Undo the currently transform the physics server is aware of and apply the provided one
            let margin_aabb = *p_transform * body_aabb;
            let margin_aabb = margin_aabb.grow(p_margin);
            let result_count = self.rapier_intersect_aabb(
                margin_aabb,
                p_body.get_base().get_collision_mask(),
                true,
                false,
                &mut results,
                32,
                p_body.get_base().get_rid(),
                physics_engine,
                physics_collision_objects,
                physics_ids,
            );
            // Optimization
            if result_count == 0 {
                break;
            }
            let mut collided = false;
            for body_shape_idx in 0..p_body.get_base().get_shape_count() {
                let body_shape_idx = body_shape_idx as usize;
                if p_body.get_base().is_shape_disabled(body_shape_idx) {
                    continue;
                }
                if let Some(body_shape) =
                    physics_shapes.get(&p_body.get_base().get_shape(physics_ids, body_shape_idx))
                {
                    let body_shape_transform =
                        *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
                    let body_shape_info = shape_info_from_body_shape(
                        body_shape.get_base().get_id(),
                        body_shape_transform,
                    );
                    for result_idx in 0..result_count {
                        let result_idx = result_idx as usize;
                        let result = &mut results[result_idx];
                        if !result.user_data.is_valid() {
                            continue;
                        }
                        let (shape_col_object, shape_index) =
                            RapierCollisionObjectBase::get_collider_user_data(
                                &result.user_data,
                                physics_ids,
                            );
                        if let Some(shape_col_object) =
                            physics_collision_objects.get(&shape_col_object)
                            && let Some(collision_body) = shape_col_object.get_body()
                        {
                            let moving_rid = p_body.get_base().get_rid();
                            let col_rid = collision_body.get_base().get_rid();
                            if collision_body.has_exception(moving_rid)
                                || p_body.has_exception(col_rid)
                            {
                                continue;
                            }
                            if let Some(col_shape) = physics_shapes.get(
                                &collision_body
                                    .get_base()
                                    .get_shape(physics_ids, shape_index),
                            ) {
                                let col_shape_transform = collision_body.get_base().get_transform()
                                    * collision_body.get_base().get_shape_transform(shape_index);
                                let col_shape_info = shape_info_from_body_shape(
                                    col_shape.get_base().get_id(),
                                    col_shape_transform,
                                );
                                let contact = physics_engine.shapes_contact(
                                    body_shape_info,
                                    col_shape_info,
                                    p_margin,
                                );
                                if !contact.collided {
                                    continue;
                                }
                                if !is_valid_recovery_contact(
                                    body_shape,
                                    body_shape_transform,
                                    col_shape,
                                    &contact,
                                    p_margin,
                                ) {
                                    continue;
                                }
                                let mut did_collide = true;
                                let skip_collision = physics_engine.should_skip_collision_one_dir(
                                    &contact,
                                    body_shape,
                                    shape_col_object,
                                    shape_index,
                                    &col_shape_transform,
                                    body_shape_transform.origin,
                                    p_margin,
                                    RapierSpace::get_last_step(),
                                    p_motion,
                                );
                                if skip_collision {
                                    // Add to excluded shapes - this shape should be skipped in later steps
                                    if *excluded_shape_pair_count < MAX_EXCLUDED_SHAPE_PAIRS {
                                        excluded_shape_pairs[*excluded_shape_pair_count] =
                                            ExcludedShapePair {
                                                local_shape_index: body_shape_idx,
                                                collision_object_rid: shape_col_object
                                                    .get_base()
                                                    .get_rid(),
                                                collision_shape_index: shape_index,
                                            };
                                        *excluded_shape_pair_count += 1;
                                    }
                                    did_collide = false;
                                }
                                if did_collide && contact_count < 32 {
                                    let a = vector_to_godot(contact.pixel_point1);
                                    let b = vector_to_godot(contact.pixel_point2);
                                    sr[contact_count * 2] = a;
                                    sr[contact_count * 2 + 1] = b;
                                    priorities[contact_count] = 1.0;
                                    contact_count += 1;
                                    collided = true;
                                }
                            }
                        }
                    }
                }
            }
            if !collided {
                break;
            }
            recovered = true;
            let recover_motion =
                recover_motion_from_contacts(&sr, &priorities, contact_count, min_contact_depth);
            // Break if recovery motion is too small to be meaningful
            if recover_motion.length() < MIN_RECOVERY_THRESHOLD {
                recovered = false;
                break;
            }
            *p_recover_motion += recover_motion;
            p_transform.origin += recover_motion;
            recover_attempts -= 1;
            if recover_attempts == 0 {
                break;
            }
        }
        recovered
    }

    #[allow(clippy::too_many_arguments)]
    fn cast_motion(
        &self,
        p_body: &RapierBody,
        p_transform: &Transform,
        p_motion: Vector,
        _p_collide_separation_ray: bool,
        _contact_max_allowed_penetration: f32,
        p_margin: f32,
        p_closest_safe: &mut f32,
        p_closest_unsafe: &mut f32,
        p_best_body_shape: &mut i32,
        excluded_shape_pairs: &[ExcludedShapePair; MAX_EXCLUDED_SHAPE_PAIRS],
        excluded_shape_pair_count: usize,
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_ids: &PhysicsIds,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) {
        let body_aabb = p_body.get_aabb(physics_shapes, physics_ids);
        let margin_aabb = *p_transform * body_aabb;
        let margin_aabb = margin_aabb.grow(p_margin);
        let mut motion_aabb = margin_aabb;
        motion_aabb.position += p_motion;
        motion_aabb = motion_aabb.merge(margin_aabb);
        let mut results = [PointHitInfo::default(); 32];
        let result_count = self.rapier_intersect_aabb(
            motion_aabb,
            p_body.get_base().get_collision_mask(),
            true,
            false,
            &mut results,
            32,
            p_body.get_base().get_rid(),
            physics_engine,
            physics_collision_objects,
            physics_ids,
        );
        if result_count == 0 {
            return;
        }
        for body_shape_idx in 0..p_body.get_base().get_shape_count() {
            let body_shape_idx = body_shape_idx as usize;
            if p_body.get_base().is_shape_disabled(body_shape_idx) {
                continue;
            }
            if let Some(body_shape) =
                physics_shapes.get(&p_body.get_base().get_shape(physics_ids, body_shape_idx))
            {
                let body_shape_transform =
                    *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
                let mut body_shape_info = shape_info_from_body_shape(
                    body_shape.get_base().get_id(),
                    body_shape_transform,
                );
                let mut best_safe = 1.0;
                let mut best_unsafe = 1.0;
                let mut stuck = false;
                for result_idx in 0..result_count {
                    let result_idx = result_idx as usize;
                    let result = &mut results[result_idx];
                    if !result.user_data.is_valid() {
                        continue;
                    }
                    let (shape_col_object, shape_index) =
                        RapierCollisionObjectBase::get_collider_user_data(
                            &result.user_data,
                            physics_ids,
                        );
                    // Check if this shape pair is excluded
                    let mut is_excluded = false;
                    for excluded in excluded_shape_pairs.iter().take(excluded_shape_pair_count) {
                        if excluded.local_shape_index == body_shape_idx
                            && excluded.collision_object_rid == shape_col_object
                            && excluded.collision_shape_index == shape_index
                        {
                            is_excluded = true;
                            break;
                        }
                    }
                    if is_excluded {
                        continue;
                    }
                    if let Some(shape_col_object) = physics_collision_objects.get(&shape_col_object)
                        && let Some(collision_body) = shape_col_object.get_body()
                    {
                        let moving_rid = p_body.get_base().get_rid();
                        let col_rid = collision_body.get_base().get_rid();
                        if collision_body.has_exception(moving_rid) || p_body.has_exception(col_rid)
                        {
                            continue;
                        }
                        if let Some(col_shape) = physics_shapes.get(
                            &collision_body
                                .get_base()
                                .get_shape(physics_ids, shape_index),
                        ) {
                            let col_shape_transform = collision_body.get_base().get_transform()
                                * collision_body.get_base().get_shape_transform(shape_index);
                            let col_shape_info = shape_info_from_body_shape(
                                col_shape.get_base().get_id(),
                                col_shape_transform,
                            );
                            // Test if going all the way collides
                            body_shape_info.transform.translation =
                                vector_to_rapier(body_shape_transform.origin + p_motion);
                            let end_contact =
                                physics_engine.shapes_contact(body_shape_info, col_shape_info, 0.0);
                            if !is_end_contact_relevant(&end_contact, p_motion) {
                                // Doesn't collide at end, skip
                                continue;
                            }
                            // Test initial overlap - if colliding at start position, body might be stuck
                            body_shape_info.transform.translation =
                                vector_to_rapier(body_shape_transform.origin);
                            let initial_contact = physics_engine.shapes_contact(
                                body_shape_info,
                                col_shape_info,
                                p_margin,
                            );
                            let penetration_depth = -initial_contact.pixel_distance;
                            if initial_contact.collided
                                && is_motion_blocked_by_contact(&initial_contact, p_motion)
                                && !initial_contact.within_margin
                                && penetration_depth > STUCK_PENETRATION_THRESHOLD
                            {
                                // Check one-way collision - allow passage if motion opposes one-way direction
                                if body_shape.allows_one_way_collision()
                                    && shape_col_object
                                        .get_base()
                                        .is_shape_set_as_one_way_collision(shape_index)
                                {
                                    let direction = -get_transform_forward(&col_shape_transform);
                                    if let Some(motion_normal) = p_motion.try_normalized()
                                        && motion_normal.dot(direction) < 0.0
                                    {
                                        continue; // Motion opposes one-way direction, allow passage
                                    }
                                }
                                // Body is stuck - set both safe and unsafe to 0 and mark this shape
                                stuck = true;
                                best_safe = 0.0;
                                best_unsafe = 0.0;
                                break; // Break out of result loop - this shape is stuck
                            }
                            //just do kinematic solving
                            let mut low = 0.0;
                            let mut hi = 1.0;
                            let mut fraction_coeff = 0.5;
                            for k in 0..BODY_MOTION_CAST_ITERATIONS {
                                let fraction = low + (hi - low) * fraction_coeff;
                                body_shape_info.transform.translation = vector_to_rapier(
                                    body_shape_transform.origin + p_motion * fraction,
                                );
                                let step_contact = physics_engine.shapes_contact(
                                    body_shape_info,
                                    col_shape_info,
                                    0.0,
                                );
                                if step_contact.collided
                                    && !step_contact.within_margin
                                    && is_motion_blocked_by_contact(&step_contact, p_motion)
                                {
                                    hi = fraction;
                                    if (k == 0) || (low > 0.0) {
                                        // Did it not collide before?
                                        // When alternating or first iteration, use dichotomy.
                                        fraction_coeff = 0.5;
                                    } else {
                                        // When colliding again, converge faster towards low
                                        // fraction for more accurate results with long motions
                                        // that collide near the start.
                                        fraction_coeff = 0.25;
                                    }
                                } else {
                                    low = fraction;
                                    if (k == 0) || (hi < 1.0) {
                                        // Did it collide before?
                                        // When alternating or first iteration, use dichotomy.
                                        fraction_coeff = 0.5;
                                    } else {
                                        // When not colliding again, converge faster towards
                                        // high fraction for more accurate results with long
                                        // motions that collide near the end.
                                        fraction_coeff = 0.75;
                                    }
                                }
                            }
                            body_shape_info.transform.translation = vector_to_rapier(
                                body_shape_transform.origin
                                    + p_motion * (hi + self.get_contact_max_allowed_penetration()),
                            );
                            let contact = physics_engine.shapes_contact(
                                body_shape_info,
                                col_shape_info,
                                p_margin,
                            );
                            if !contact.collided {
                                continue;
                            }
                            if !is_motion_blocked_by_contact(&contact, p_motion) {
                                continue;
                            }
                            if physics_engine.should_skip_collision_one_dir(
                                &contact,
                                body_shape,
                                shape_col_object,
                                shape_index,
                                &col_shape_transform,
                                body_shape_transform.origin
                                    + p_motion * (hi + self.get_contact_max_allowed_penetration()),
                                p_margin,
                                RapierSpace::get_last_step(),
                                p_motion,
                            ) {
                                continue;
                            }
                            if low < best_safe {
                                best_safe = low;
                                best_unsafe = hi;
                            }
                        }
                    }
                }
                // If stuck, immediately update closest values and stop checking other shapes
                if stuck {
                    *p_closest_safe = 0.0;
                    *p_closest_unsafe = 0.0;
                    *p_best_body_shape = body_shape_idx as i32;
                    break; // Break out of body shape loop
                }
                if best_safe == 1.0 {
                    continue;
                }
                if best_safe < *p_closest_safe {
                    *p_closest_safe = best_safe;
                    *p_closest_unsafe = best_unsafe;
                    *p_best_body_shape = body_shape_idx as i32;
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn body_motion_collide(
        &self,
        p_body: &RapierBody,
        p_transform: &Transform,
        p_motion: Vector,
        p_best_body_shape: i32,
        p_margin: f32,
        min_allowed_depth: Real,
        p_result: &mut PhysicsServerExtensionMotionResult,
        excluded_shape_pairs: &[ExcludedShapePair; MAX_EXCLUDED_SHAPE_PAIRS],
        excluded_shape_pair_count: usize,
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_ids: &PhysicsIds,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        let shape_count = p_body.get_base().get_shape_count();
        if shape_count < 1 {
            return false;
        }
        let body_aabb = p_body.get_aabb(physics_shapes, physics_ids);
        let margin_aabb = *p_transform * body_aabb;
        let margin_aabb = margin_aabb.grow(p_margin);
        // also check things at motion
        let mut motion_aabb = margin_aabb;
        motion_aabb.position += p_motion;
        motion_aabb = motion_aabb.merge(margin_aabb);
        let mut results = [PointHitInfo::default(); 32];
        let result_count = self.rapier_intersect_aabb(
            motion_aabb,
            p_body.get_base().get_collision_mask(),
            true,
            false,
            &mut results,
            32,
            p_body.get_base().get_rid(),
            physics_engine,
            physics_collision_objects,
            physics_ids,
        );
        // Optimization
        if result_count == 0 {
            return false;
        }
        let mut best_depth = 0.0;
        let mut best_collision_body = None;
        let mut best_collision_shape_index: i32 = -1;
        let mut best_body_shape_index = -1;
        let mut best_contact = ContactResult::default();
        let from_shape = if p_best_body_shape != -1 {
            p_best_body_shape
        } else {
            0
        };
        let to_shape = if p_best_body_shape != -1 {
            p_best_body_shape + 1
        } else {
            p_body.get_base().get_shape_count()
        };
        for body_shape_idx in from_shape..to_shape {
            if p_body.get_base().is_shape_disabled(body_shape_idx as usize) {
                continue;
            }
            let body_shape = p_body
                .get_base()
                .get_shape(physics_ids, body_shape_idx as usize);
            let body_shape_transform = *p_transform
                * p_body
                    .get_base()
                    .get_shape_transform(body_shape_idx as usize);
            if let Some(body_shape_obj) = physics_shapes.get(&body_shape) {
                let body_shape_info = shape_info_from_body_shape(
                    body_shape_obj.get_base().get_id(),
                    body_shape_transform,
                );
                for result_idx in 0..result_count {
                    let result = &mut results[result_idx as usize];
                    if !result.user_data.is_valid() {
                        continue;
                    }
                    let (shape_col_object, shape_index) =
                        RapierCollisionObjectBase::get_collider_user_data(
                            &result.user_data,
                            physics_ids,
                        );
                    // Check if this shape pair is in the excluded list
                    let mut is_excluded = false;
                    for excluded in excluded_shape_pairs.iter().take(excluded_shape_pair_count) {
                        if excluded.local_shape_index == body_shape_idx as usize
                            && excluded.collision_object_rid == shape_col_object
                            && excluded.collision_shape_index == shape_index
                        {
                            is_excluded = true;
                            break;
                        }
                    }
                    if is_excluded {
                        continue;
                    }
                    if let Some(shape_col_object) = physics_collision_objects.get(&shape_col_object)
                        && let Some(collision_body) = shape_col_object.get_body()
                    {
                        let moving_rid = p_body.get_base().get_rid();
                        let col_rid = collision_body.get_base().get_rid();
                        if collision_body.has_exception(moving_rid) || p_body.has_exception(col_rid)
                        {
                            continue;
                        }
                        let col_shape_rid = collision_body
                            .get_base()
                            .get_shape(physics_ids, shape_index);
                        if let Some(col_shape) = physics_shapes.get(&col_shape_rid) {
                            let col_shape_transform = collision_body.get_base().get_transform()
                                * collision_body.get_base().get_shape_transform(shape_index);
                            let col_shape_info = shape_info_from_body_shape(
                                col_shape.get_base().get_id(),
                                col_shape_transform,
                            );
                            let contact = physics_engine.shapes_contact(
                                body_shape_info,
                                col_shape_info,
                                p_margin,
                            );
                            if !contact.collided {
                                continue;
                            }
                            if physics_engine.should_skip_collision_one_dir(
                                &contact,
                                body_shape_obj,
                                shape_col_object,
                                shape_index,
                                &col_shape_transform,
                                body_shape_transform.origin,
                                p_margin,
                                RapierSpace::get_last_step(),
                                p_motion,
                            ) {
                                continue;
                            }
                            if !is_contact_depth_allowed(
                                contact.pixel_distance,
                                p_margin,
                                min_allowed_depth,
                            ) {
                                continue;
                            }
                            let depth = contact_depth(contact.pixel_distance, p_margin);
                            if depth > best_depth {
                                best_depth = depth;
                                best_collision_body = Some(collision_body);
                                best_collision_shape_index = shape_index as i32;
                                best_body_shape_index = body_shape_idx;
                                best_contact = contact;
                            }
                        }
                    }
                }
            }
        }
        if let Some(best_collision_body) = best_collision_body {
            // conveyer belt
            if best_collision_body.get_static_linear_velocity() != Vector::default() {
                p_result.travel +=
                    best_collision_body.get_static_linear_velocity() * RapierSpace::get_last_step();
            }
            p_result.collision_depth = p_margin - best_contact.pixel_distance;
            let collision_point = vector_to_godot(contact_collision_point(&best_contact));
            let local_position =
                collision_point - best_collision_body.get_base().get_transform().origin;
            set_collision_info(
                p_result,
                best_collision_body.get_base().get_rid(),
                ObjectId {
                    id: best_collision_body.get_base().get_instance_id(),
                },
                best_collision_shape_index,
                best_body_shape_index,
                collision_point,
                vector_to_godot(best_contact.normal2),
                best_collision_body.get_velocity_at_local_point(local_position, physics_engine),
            );
            return true;
        }
        false
    }
}
#[cfg(feature = "dim2")]
#[allow(clippy::too_many_arguments)]
fn set_collision_info(
    p_result: &mut PhysicsServerExtensionMotionResult,
    collider: Rid,
    collider_id: ObjectId,
    collider_shape: i32,
    collision_local_shape: i32,
    collision_point: Vector,
    collision_normal: Vector,
    collider_velocity: Vector,
) {
    p_result.collider = collider;
    p_result.collider_id = collider_id;
    p_result.collider_shape = collider_shape;
    p_result.collision_local_shape = collision_local_shape;
    p_result.collision_point = collision_point;
    p_result.collision_normal = collision_normal;
    p_result.collider_velocity = collider_velocity;
}
#[cfg(feature = "dim3")]
#[allow(clippy::too_many_arguments)]
fn set_collision_info(
    p_result: &mut PhysicsServerExtensionMotionResult,
    collider: Rid,
    collider_id: ObjectId,
    collider_shape: i32,
    collision_local_shape: i32,
    collision_point: Vector,
    collision_normal: Vector,
    collider_velocity: Vector,
) {
    use godot::classes::native::PhysicsServer3DExtensionMotionCollision;
    let id = collider_id.id;
    p_result.collisions = core::array::from_fn(|_i| PhysicsServer3DExtensionMotionCollision {
        position: collision_point,
        normal: collision_normal,
        collider_velocity,
        collider_angular_velocity: Vector::ZERO,
        depth: p_result.collision_depth,
        local_shape: collision_local_shape,
        collider_id: ObjectId { id },
        collider,
        collider_shape,
    });
    p_result.collision_count = 1;
}
#[cfg(feature = "dim2")]
fn get_transform_forward(transform: &Transform2D) -> Vector {
    -transform.b
}
#[cfg(feature = "dim3")]
fn get_transform_forward(transform: &Transform3D) -> Vector {
    -transform.basis.col_b()
}
fn one_way_valid_depth(
    owc_margin: f32,
    motion_margin: f32,
    platform_linear_velocity: Vector,
    last_step: f32,
    valid_dir: Vector,
) -> f32 {
    let mut valid_depth = owc_margin.max(motion_margin);
    let platform_motion = platform_linear_velocity * last_step;
    let platform_motion_len = platform_motion.length();
    if !platform_motion_len.is_zero_approx() {
        valid_depth +=
            platform_motion_len * vector_normalized(platform_motion).dot(-valid_dir).max(0.0);
    }
    valid_depth
}
fn is_one_way_contact_invalid(
    contact: &ContactResult,
    moving_shape_origin: Vector,
    platform_shape_origin: Vector,
    valid_dir: Vector,
    valid_depth: Real,
) -> bool {
    let rel_dir = vector_to_godot(contact.pixel_point1) - vector_to_godot(contact.pixel_point2);
    let rel_length_sq = rel_dir.length_squared();
    if rel_length_sq > valid_depth * valid_depth {
        return true;
    }
    let shape_rel_dir = moving_shape_origin - platform_shape_origin;
    shape_rel_dir.length_squared() > NORMAL_EPSILON
        && valid_dir.dot(vector_normalized(shape_rel_dir)) < DEFAULT_EPSILON
}
impl PhysicsEngine {
    #[allow(clippy::too_many_arguments)]
    fn should_skip_collision_one_dir(
        &self,
        contact: &ContactResult,
        body_shape: &RapierShape,
        collision_body: &RapierCollisionObject,
        shape_index: usize,
        col_shape_transform: &Transform,
        moving_shape_origin: Vector,
        p_margin: f32,
        last_step: f32,
        _p_motion: Vector,
    ) -> bool {
        if body_shape.allows_one_way_collision()
            && collision_body
                .get_base()
                .is_shape_set_as_one_way_collision(shape_index)
        {
            let valid_dir = vector_normalized(get_transform_forward(col_shape_transform));
            let owc_margin = collision_body
                .get_base()
                .get_shape_one_way_collision_margin(shape_index);
            let mut platform_linear_velocity = Vector::default();
            if let Some(b) = collision_body.get_body()
                && b.get_base().mode.ord() >= BodyMode::KINEMATIC.ord()
            {
                platform_linear_velocity = b.get_linear_velocity(self);
            }
            let valid_depth = one_way_valid_depth(
                owc_margin,
                p_margin,
                platform_linear_velocity,
                last_step,
                valid_dir,
            );
            if is_one_way_contact_invalid(
                contact,
                moving_shape_origin,
                col_shape_transform.origin,
                valid_dir,
                valid_depth,
            ) {
                return true;
            }
        }
        false
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "dim2")]
    fn x_motion(length: Real) -> Vector {
        Vector::new(length, 0.0)
    }
    #[cfg(feature = "dim2")]
    fn y_motion(length: Real) -> Vector {
        Vector::new(0.0, length)
    }
    #[cfg(feature = "dim3")]
    fn x_motion(length: Real) -> Vector {
        Vector::new(length, 0.0, 0.0)
    }
    #[cfg(feature = "dim3")]
    fn y_motion(length: Real) -> Vector {
        Vector::new(0.0, length, 0.0)
    }
    #[cfg(feature = "dim2")]
    fn motion_result() -> PhysicsServerExtensionMotionResult {
        PhysicsServerExtensionMotionResult {
            travel: Vector::default(),
            remainder: Vector::default(),
            collision_point: Vector::default(),
            collision_normal: Vector::default(),
            collider_velocity: Vector::default(),
            collision_depth: 0.0,
            collision_safe_fraction: 0.0,
            collision_unsafe_fraction: 0.0,
            collision_local_shape: 0,
            collider_id: ObjectId { id: 0 },
            collider: Rid::Invalid,
            collider_shape: 0,
        }
    }
    #[cfg(feature = "dim3")]
    fn motion_result() -> PhysicsServerExtensionMotionResult {
        use godot::classes::native::PhysicsServer3DExtensionMotionCollision;
        PhysicsServerExtensionMotionResult {
            travel: Vector::default(),
            remainder: Vector::default(),
            collision_depth: 0.0,
            collision_safe_fraction: 0.0,
            collision_unsafe_fraction: 0.0,
            collisions: core::array::from_fn(|_| PhysicsServer3DExtensionMotionCollision {
                position: Vector::default(),
                normal: Vector::default(),
                collider_velocity: Vector::default(),
                collider_angular_velocity: Vector::default(),
                depth: 0.0,
                local_shape: 0,
                collider_id: ObjectId { id: 0 },
                collider: Rid::Invalid,
                collider_shape: 0,
            }),
            collision_count: 0,
        }
    }
    fn assert_real_approx_eq(actual: Real, expected: Real) {
        assert!(
            (actual - expected).abs() <= 1.0e-5,
            "expected {expected}, got {actual}"
        );
    }
    #[test]
    fn recover_motion_uses_equal_priority_contacts() {
        let mut contacts = [Vector::default(); 64];
        let mut priorities = [0.0; 32];
        contacts[0] = x_motion(1.0);
        contacts[1] = Vector::default();
        contacts[2] = y_motion(1.0);
        contacts[3] = Vector::default();
        priorities[0] = 1.0;
        priorities[1] = 1.0;
        let recover_motion = recover_motion_from_contacts(&contacts, &priorities, 2, 0.0);
        assert_real_approx_eq(recover_motion.x, -BODY_MOTION_RECOVER_RATIO);
        assert_real_approx_eq(recover_motion.y, -BODY_MOTION_RECOVER_RATIO);
    }
    #[test]
    fn recover_motion_respects_min_contact_depth() {
        let mut contacts = [Vector::default(); 64];
        let mut priorities = [0.0; 32];
        contacts[0] = x_motion(0.01);
        contacts[1] = Vector::default();
        priorities[0] = 1.0;
        let recover_motion = recover_motion_from_contacts(&contacts, &priorities, 1, 0.02);
        assert_eq!(recover_motion, Vector::default());
    }
    #[cfg(feature = "dim2")]
    #[test]
    fn reset_body_motion_result_clears_2d_collision_state() {
        let mut result = motion_result();
        result.travel = x_motion(1.0);
        result.remainder = x_motion(2.0);
        result.collision_depth = 3.0;
        result.collision_safe_fraction = 0.4;
        result.collision_unsafe_fraction = 0.5;
        result.collider = Rid::new(99);
        result.collider_shape = 4;
        reset_body_motion_result(&mut result);
        assert_eq!(result.travel, Vector::default());
        assert_eq!(result.remainder, Vector::default());
        assert_eq!(result.collision_depth, 0.0);
        assert_eq!(result.collision_safe_fraction, 0.0);
        assert_eq!(result.collision_unsafe_fraction, 0.0);
        assert_eq!(result.collider, Rid::Invalid);
        assert_eq!(result.collider_shape, 0);
    }
    #[cfg(feature = "dim3")]
    #[test]
    fn reset_body_motion_result_clears_3d_collision_state() {
        let mut result = motion_result();
        result.travel = x_motion(1.0);
        result.remainder = x_motion(2.0);
        result.collision_depth = 3.0;
        result.collision_safe_fraction = 0.4;
        result.collision_unsafe_fraction = 0.5;
        result.collision_count = 4;
        reset_body_motion_result(&mut result);
        assert_eq!(result.travel, Vector::default());
        assert_eq!(result.remainder, Vector::default());
        assert_eq!(result.collision_depth, 0.0);
        assert_eq!(result.collision_safe_fraction, 0.0);
        assert_eq!(result.collision_unsafe_fraction, 0.0);
        assert_eq!(result.collision_count, 0);
    }
    #[test]
    fn contact_depth_filter_rejects_shallow_or_predictive_contacts() {
        assert!(is_contact_depth_allowed(0.02, 0.1, 0.05));
        assert!(!is_contact_depth_allowed(0.099, 0.1, 0.005));
        assert!(!is_contact_depth_allowed(0.2, 0.1, 0.0));
    }
    #[test]
    fn motion_blocking_filter_accepts_opposing_contact_normal() {
        let contact = ContactResult {
            normal2: vector_to_rapier(x_motion(-1.0)),
            ..Default::default()
        };
        assert!(is_motion_blocked_by_contact(&contact, x_motion(10.0)));
    }
    #[test]
    fn motion_blocking_filter_rejects_tangent_contact_normal() {
        let contact = ContactResult {
            normal2: vector_to_rapier(y_motion(-1.0)),
            ..Default::default()
        };
        assert!(!is_motion_blocked_by_contact(&contact, x_motion(10.0)));
    }
    #[test]
    fn clamp_near_zero_safe_motion_sets_sub_epsilon_travel_to_zero() {
        let motion = x_motion(10.0);
        let mut safe_fraction = (MOTION_EPSILON * 0.5) / motion.length();
        clamp_near_zero_safe_motion(motion, 0.0, &mut safe_fraction);
        assert_eq!(safe_fraction, 0.0);
    }
    #[test]
    fn clamp_near_zero_safe_motion_keeps_meaningful_travel() {
        let motion = x_motion(10.0);
        let mut safe_fraction = (blocked_motion_tolerance(0.0) * 2.0) / motion.length();
        let expected = safe_fraction;
        clamp_near_zero_safe_motion(motion, 0.0, &mut safe_fraction);
        assert_eq!(safe_fraction, expected);
    }
    #[test]
    fn clamp_near_zero_safe_motion_uses_margin_relative_tolerance() {
        let motion = x_motion(10.0);
        let mut safe_fraction = 0.003 / motion.length();
        clamp_near_zero_safe_motion(motion, 0.08, &mut safe_fraction);
        assert_eq!(safe_fraction, 0.0);
    }
    #[test]
    fn clamp_near_zero_blocked_travel_removes_tiny_forward_travel() {
        let motion = x_motion(10.0);
        let mut travel = x_motion(0.003);
        clamp_near_zero_blocked_travel(motion, 0.08, &mut travel);
        assert_eq!(travel, Vector::default());
    }
    #[test]
    fn clamp_near_zero_blocked_travel_keeps_recovery_against_motion() {
        let motion = x_motion(10.0);
        let mut travel = x_motion(-0.003);
        clamp_near_zero_blocked_travel(motion, 0.08, &mut travel);
        assert_eq!(travel, x_motion(-0.003));
    }
    #[test]
    fn one_way_valid_depth_ignores_test_body_motion() {
        let valid_dir = y_motion(1.0);
        let depth_without_platform_motion =
            one_way_valid_depth(1.0, 0.08, Vector::default(), 1.0 / 60.0, valid_dir);
        assert_eq!(depth_without_platform_motion, 1.0);
    }
    #[test]
    fn one_way_valid_depth_includes_platform_motion_against_one_way_direction() {
        let valid_dir = y_motion(1.0);
        let depth = one_way_valid_depth(1.0, 0.08, y_motion(-60.0), 1.0 / 60.0, valid_dir);
        assert_eq!(depth, 2.0);
    }
    #[test]
    fn one_way_contact_accepts_contact_along_valid_direction() {
        let contact = ContactResult {
            pixel_point1: vector_to_rapier(x_motion(-1.0)),
            pixel_point2: vector_to_rapier(Vector::default()),
            ..Default::default()
        };
        assert!(!is_one_way_contact_invalid(
            &contact,
            x_motion(1.0),
            Vector::default(),
            x_motion(1.0),
            1.0
        ));
    }
    #[test]
    fn one_way_contact_rejects_contact_against_valid_direction() {
        let contact = ContactResult {
            pixel_point1: vector_to_rapier(x_motion(-1.0)),
            pixel_point2: vector_to_rapier(Vector::default()),
            ..Default::default()
        };
        assert!(is_one_way_contact_invalid(
            &contact,
            x_motion(1.0),
            Vector::default(),
            x_motion(-1.0),
            1.0
        ));
    }
    #[test]
    fn one_way_contact_rejects_contact_beyond_margin() {
        let contact = ContactResult {
            pixel_point1: vector_to_rapier(x_motion(-2.0)),
            pixel_point2: vector_to_rapier(Vector::default()),
            ..Default::default()
        };
        assert!(is_one_way_contact_invalid(
            &contact,
            x_motion(1.0),
            Vector::default(),
            x_motion(1.0),
            1.0
        ));
    }
    #[test]
    fn clamp_near_zero_safe_motion_keeps_full_motion() {
        let motion = x_motion(10.0);
        let mut safe_fraction = 1.0;
        clamp_near_zero_safe_motion(motion, 0.0, &mut safe_fraction);
        assert_eq!(safe_fraction, 1.0);
    }
    #[test]
    fn small_motion_step_finishes_without_collision() {
        let mut result = motion_result();
        result.travel = x_motion(42.0);
        result.remainder = x_motion(13.0);
        result.collision_safe_fraction = 0.0;
        result.collision_unsafe_fraction = 0.0;
        let finished = finish_small_body_motion(x_motion(MIN_MOTION_THRESHOLD * 0.5), &mut result);
        assert!(finished);
        assert_eq!(result.travel, Vector::default());
        assert_eq!(result.remainder, Vector::default());
        assert_eq!(result.collision_safe_fraction, 1.0);
        assert_eq!(result.collision_unsafe_fraction, 1.0);
    }
    #[test]
    fn small_motion_step_keeps_regular_motion_for_later_steps() {
        let mut result = motion_result();
        let finished = finish_small_body_motion(x_motion(MIN_MOTION_THRESHOLD * 2.0), &mut result);
        assert!(!finished);
    }
    #[test]
    fn rest_info_step_runs_for_cast_collision() {
        assert!(should_collect_body_motion_collision(false, false, 0.5));
    }
    #[test]
    fn rest_info_step_runs_for_recovery_collision_when_requested() {
        assert!(should_collect_body_motion_collision(true, true, 1.0));
    }
    #[test]
    fn rest_info_step_skips_recovery_when_not_requested() {
        assert!(!should_collect_body_motion_collision(false, true, 1.0));
    }
    #[test]
    fn final_step_applies_collision_travel_remainder_and_fractions() {
        let mut result = motion_result();
        let recover_motion = x_motion(0.25);
        let motion = x_motion(8.0);
        finish_body_motion_result(&mut result, recover_motion, motion, 0.0, 0.5, 0.75, true);
        assert_eq!(result.travel, x_motion(4.25));
        assert_eq!(result.remainder, x_motion(4.0));
        assert_eq!(result.collision_safe_fraction, 0.5);
        assert_eq!(result.collision_unsafe_fraction, 0.75);
    }
    #[test]
    fn final_step_applies_full_motion_without_collision() {
        let mut result = motion_result();
        result.collision_depth = 1.0;
        finish_body_motion_result(
            &mut result,
            x_motion(0.25),
            x_motion(8.0),
            0.0,
            0.5,
            0.75,
            false,
        );
        assert_eq!(result.travel, x_motion(8.25));
        assert_eq!(result.remainder, Vector::default());
        assert_eq!(result.collision_depth, 0.0);
        assert_eq!(result.collision_safe_fraction, 1.0);
        assert_eq!(result.collision_unsafe_fraction, 1.0);
    }
}
