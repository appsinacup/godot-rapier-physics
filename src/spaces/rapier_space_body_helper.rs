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
const BODY_MOTION_RECOVER_RATIO: Real = 0.4;
const MAX_EXCLUDED_SHAPE_PAIRS: usize = 32;
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
        if canvas_excluded || layer_excluded || rid_excluded {
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
        result.travel = Vector::default();
        // Skip processing if motion is too small (prevents infinite micro-adjustments)
        const MIN_MOTION_THRESHOLD: Real = 0.001;
        if motion.length() < MIN_MOTION_THRESHOLD {
            result.remainder = Vector::default();
            result.collision_safe_fraction = 1.0;
            result.collision_unsafe_fraction = 1.0;
            return false;
        }
        let mut body_transform = from; // Because body_transform needs to be modified during recovery
        // Step 1: recover motion.
        // Expand the body colliders by the margin (grow) and check if now it collides with a collider,
        // if yes, "recover" / "push" out of this collider
        let mut recover_motion = Vector::default();
        let margin = Real::max(margin, TEST_MOTION_MARGIN);
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
        // If cast motion resulted in near-zero movement but recovery found no collision,
        // treat this as a numerical precision issue and allow the motion
        const MOTION_EPSILON: Real = 0.001;
        if !recovered && best_safe < MOTION_EPSILON && best_unsafe < MOTION_EPSILON {
            best_safe = 1.0;
            best_unsafe = 1.0;
        }
        // Step 3: Rest Info
        // Apply the motion and fill the collision information
        let mut collided = false;
        if (recovery_as_collision && recovered) || (best_safe < 1.0) {
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
                result,
                &excluded_shape_pairs,
                excluded_shape_pair_count,
                physics_engine,
                physics_shapes,
                physics_ids,
                physics_collision_objects,
            );
        }
        if collided {
            result.travel += recover_motion + motion * best_safe;
            result.remainder = motion - motion * best_safe;
            result.collision_safe_fraction = best_safe;
            result.collision_unsafe_fraction = best_unsafe;
        } else {
            result.travel += recover_motion + motion;
            result.remainder = Vector::default();
            result.collision_depth = 0.0;
            result.collision_safe_fraction = 1.0;
            result.collision_unsafe_fraction = 1.0;
        }
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
                            if collision_body.has_exception(p_body.get_base().get_rid()) {
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
                                let mut did_collide = true;
                                let skip_collision = physics_engine.should_skip_collision_one_dir(
                                    &contact,
                                    body_shape,
                                    shape_col_object,
                                    shape_index,
                                    &col_shape_transform,
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
            // First pass: calculate all depths and total priority (like Godot does)
            let mut depths = [0.0; 32];
            let mut total_priority = 0.0;
            for i in 0..contact_count {
                let a = sr[i * 2];
                let b = sr[i * 2 + 1];
                if let Some(n) = (a - b).try_normalized() {
                    let d = n.dot(b);
                    let depth = n.dot(a) - d;
                    // Count any penetration, even if shallow
                    if depth > DEFAULT_EPSILON {
                        depths[i] = depth;
                        total_priority += depth;
                    }
                }
            }
            // Second pass: apply recovery weighted by priority
            let mut recover_motion = Vector::default();
            if total_priority > 0.0 {
                for i in 0..contact_count {
                    if depths[i] <= 0.0 {
                        continue;
                    }
                    let a = sr[i * 2];
                    let b = sr[i * 2 + 1];
                    if let Some(n) = (a - b).try_normalized() {
                        let d = n.dot(b);
                        let depth = n.dot(a + recover_motion) - d;
                        if depth > DEFAULT_EPSILON {
                            // Priority weight: deeper contacts get more correction
                            let priority = depths[i] / total_priority;
                            let recovery_amount = (depth - min_contact_depth).max(depth * 0.4);
                            recover_motion -=
                                n * recovery_amount * BODY_MOTION_RECOVER_RATIO * priority;
                        }
                    }
                }
            }
            // Break if recovery motion is too small to be meaningful
            const MIN_RECOVERY_THRESHOLD: Real = 0.001;
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
                // Colliding separation rays allows to properly snap to the ground,
                // otherwise it's not needed in regular motion.
                //if !p_collide_separation_ray
                //    && body_shape.get_type() == PhysicsServer2D::SHAPE_SEPARATION_RAY
                //{
                // When slide on slope is on, separation ray shape acts like a
                // regular shape.
                //if !body_shape.downcast_ref::<RapierSeparationRayShape>().unwrap().get_slide_on_slope()
                //{
                //    continue;
                //}
                //}
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
                    for excluded_idx in 0..excluded_shape_pair_count {
                        let excluded = &excluded_shape_pairs[excluded_idx];
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
                        if collision_body.has_exception(p_body.get_base().get_rid()) {
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
                            body_shape_info.transform.translation.vector =
                                vector_to_rapier(body_shape_transform.origin + p_motion);
                            let end_contact =
                                physics_engine.shapes_contact(body_shape_info, col_shape_info, 0.0);
                            if !end_contact.collided {
                                // Doesn't collide at end, skip
                                continue;
                            }
                            // Test initial overlap - if colliding at start position, body is stuck
                            body_shape_info.transform.translation.vector =
                                vector_to_rapier(body_shape_transform.origin);
                            let initial_contact = physics_engine.shapes_contact(
                                body_shape_info,
                                col_shape_info,
                                p_margin, // Use same margin as recovery
                            );
                            // Only consider it stuck if there's deep actual penetration that recovery can't handle
                            // Shallow penetrations (< 0.1 pixels) should be handled by recovery
                            const STUCK_PENETRATION_THRESHOLD: Real = 0.1;
                            let penetration_depth = -initial_contact.pixel_distance;
                            if initial_contact.collided
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
                            for k in 0..8 {
                                let fraction = low + (hi - low) * fraction_coeff;
                                body_shape_info.transform.translation.vector = vector_to_rapier(
                                    body_shape_transform.origin + p_motion * fraction,
                                );
                                let step_contact = physics_engine.shapes_contact(
                                    body_shape_info,
                                    col_shape_info,
                                    0.0,
                                );
                                if step_contact.collided && !step_contact.within_margin {
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
                            body_shape_info.transform.translation.vector = vector_to_rapier(
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
                            if physics_engine.should_skip_collision_one_dir(
                                &contact,
                                body_shape,
                                shape_col_object,
                                shape_index,
                                &col_shape_transform,
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
        let mut min_distance = f32::INFINITY;
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
                    for i in 0..excluded_shape_pair_count {
                        let excluded = &excluded_shape_pairs[i];
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
                        if collision_body.has_exception(p_body.get_base().get_rid()) {
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
                                p_margin,
                                RapierSpace::get_last_step(),
                                p_motion,
                            ) {
                                continue;
                            }
                            if contact.pixel_distance < min_distance {
                                min_distance = contact.pixel_distance;
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
            let collision_point = vector_to_godot(best_contact.pixel_point1);
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
    -Vector2::new(transform.a.y, transform.b.y)
}
#[cfg(feature = "dim3")]
fn get_transform_forward(transform: &Transform3D) -> Vector {
    -transform.basis.col_c()
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
        p_margin: f32,
        last_step: f32,
        p_motion: Vector,
    ) -> bool {
        let dist = contact.pixel_distance;
        if body_shape.allows_one_way_collision()
            && collision_body
                .get_base()
                .is_shape_set_as_one_way_collision(shape_index)
        {
            let valid_dir = -vector_normalized(get_transform_forward(col_shape_transform));
            let owc_margin = collision_body
                .get_base()
                .get_shape_one_way_collision_margin(shape_index);
            let mut valid_depth = owc_margin.max(p_margin);
            if let Some(b) = collision_body.get_body()
                && b.get_base().mode.ord() >= BodyMode::KINEMATIC.ord()
            {
                // Increase margin by platform movement in the one-way direction
                let lv = b.get_linear_velocity(self);
                let mut motion = lv * last_step;
                let motion_len = motion.length();
                if !motion_len.is_zero_approx() {
                    motion = vector_normalized(motion);
                }
                valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
            }
            let _motion = p_motion;
            let motion_len = p_motion.length();
            if !motion_len.is_zero_approx() {
                valid_depth += motion_len * vector_normalized(p_motion).dot(valid_dir).max(0.0);
            }
            let motion_dot_valid_dir = if motion_len.is_zero_approx() {
                0.0
            } else {
                vector_normalized(p_motion).dot(valid_dir)
            };
            let contact_normal = vector_to_godot(contact.normal1);
            let normal_dot_direction = contact_normal.dot(valid_dir);
            // If motion opposes one-way direction, skip collision (allows passage through platforms)
            if motion_dot_valid_dir < 0.0 {
                return true;
            }
            // Check if contact normal is valid (non-zero)
            let normal_length_sq = contact_normal.length_squared();
            const NORMAL_EPSILON: Real = 0.01;
            if normal_length_sq > NORMAL_EPSILON {
                // Skip side edges (perpendicular contacts)
                const ONE_WAY_PERPENDICULAR_THRESHOLD: Real = 0.1;
                if normal_dot_direction.abs() < ONE_WAY_PERPENDICULAR_THRESHOLD {
                    return true;
                }
                // Skip if contact normal opposes one-way direction
                if normal_dot_direction < 0.0 {
                    return true;
                }
            }
            // Skip deeply penetrating contacts
            if dist < -valid_depth {
                return true;
            }
        }
        false
    }
}
