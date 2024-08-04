use std::ops::Deref;

use godot::classes::native::ObjectId;
use godot::classes::physics_server_2d::BodyMode;
use godot::prelude::*;
use rapier::geometry::ColliderHandle;
use rapier::math::Real;
use rapier::math::DEFAULT_EPSILON;
use servers::rapier_physics_singleton::PhysicsCollisionObjects;
use servers::rapier_physics_singleton::PhysicsShapes;

use super::rapier_space::RapierSpace;
use super::RapierDirectSpaceState;
use crate::bodies::rapier_body::RapierBody;
use crate::bodies::rapier_collision_object::*;
use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::IRapierShape;
use crate::types::*;
use crate::*;
const TEST_MOTION_MARGIN: Real = 1e-4;
const TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR: Real = 0.05;
const BODY_MOTION_RECOVER_ATTEMPTS: i32 = 4;
const BODY_MOTION_RECOVER_RATIO: Real = 0.4;
impl RapierSpace {
    pub fn is_handle_excluded_callback(
        &self,
        collider_handle: ColliderHandle,
        user_data: &UserData,
        handle_excluded_info: &QueryExcludedInfo,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        for exclude_index in 0..handle_excluded_info.query_exclude_size {
            if handle_excluded_info.query_exclude[exclude_index] == collider_handle {
                return true;
            }
        }
        let (collision_object_2d, _) = RapierCollisionObject::get_collider_user_data(user_data);
        let Some(collision_object_2d) = physics_collision_objects.get(&collision_object_2d) else {
            return false;
        };
        let collision_object_base = collision_object_2d.get_base();
        let canvas_excluded = collision_object_base.get_canvas_instance_id()
            != handle_excluded_info.query_canvas_instance_id;
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
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        result.travel = Vector::default();
        let mut body_transform = from; // Because body_transform needs to be modified during recovery
                                       // Step 1: recover motion.
                                       // Expand the body colliders by the margin (grow) and check if now it collides with a collider,
                                       // if yes, "recover" / "push" out of this collider
        let mut recover_motion = Vector::default();
        let margin = Real::max(margin, TEST_MOTION_MARGIN);
        let recovered = self.body_motion_recover(
            body,
            &mut body_transform,
            motion,
            margin,
            &mut recover_motion,
            physics_engine,
            physics_shapes,
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
            physics_engine,
            physics_shapes,
            physics_collision_objects,
        );
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
                physics_engine,
                physics_shapes,
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
    ) -> i32 {
        if max_results < 1 {
            return 0;
        }
        let rect_begin = aabb.position;
        let rect_end = aabb.end();
        let mut handle_excluded_info = QueryExcludedInfo::default();
        let mut query_exclude = Vec::new();
        query_exclude.resize_with(max_results, Default::default);
        handle_excluded_info.query_exclude = query_exclude;
        handle_excluded_info.query_collision_layer_mask = collision_mask;
        handle_excluded_info.query_exclude_size = 0;
        handle_excluded_info.query_exclude_body = exclude_body.to_u64() as i64;
        physics_engine.intersect_aabb(
            self.get_handle(),
            vector_to_rapier(rect_begin),
            vector_to_rapier(rect_end),
            collide_with_bodies,
            collide_with_areas,
            results,
            max_results,
            &handle_excluded_info,
            physics_collision_objects,
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
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        let shape_count = p_body.get_base().get_shape_count();
        if shape_count < 1 {
            return false;
        }
        let min_contact_depth = p_margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;
        let mut recovered = false;
        let mut recover_attempts = BODY_MOTION_RECOVER_ATTEMPTS;
        let body_aabb = p_body.get_aabb(physics_shapes);
        loop {
            let mut results = [PointHitInfo::default(); 32];
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
            );
            // Optimization
            if result_count == 0 {
                break;
            }
            let mut recover_step = Vector::default();
            for body_shape_idx in 0..p_body.get_base().get_shape_count() {
                let body_shape_idx = body_shape_idx as usize;
                if p_body.get_base().is_shape_disabled(body_shape_idx) {
                    continue;
                }
                if let Some(body_shape) =
                    physics_shapes.get(&p_body.get_base().get_shape(body_shape_idx))
                {
                    let body_shape_transform =
                        *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
                    let body_shape_info =
                        shape_info_from_body_shape(body_shape.get_handle(), body_shape_transform);
                    for result_idx in 0..result_count {
                        let result_idx = result_idx as usize;
                        let result = &mut results[result_idx];
                        if !result.user_data.is_valid() {
                            continue;
                        }
                        let (shape_col_object, shape_index) =
                            RapierCollisionObject::get_collider_user_data(&result.user_data);
                        if let Some(shape_col_object) =
                            physics_collision_objects.get(&shape_col_object)
                        {
                            if let Some(collision_body) = shape_col_object.get_body() {
                                if collision_body.has_exception(p_body.get_base().get_rid()) {
                                    continue;
                                }
                                if let Some(col_shape) = physics_shapes
                                    .get(&collision_body.get_base().get_shape(shape_index))
                                {
                                    let col_shape_transform =
                                        collision_body.get_base().get_transform()
                                            * collision_body
                                                .get_base()
                                                .get_shape_transform(shape_index);
                                    let col_shape_info = shape_info_from_body_shape(
                                        col_shape.get_handle(),
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
                                        body_shape.deref(),
                                        collision_body,
                                        shape_index,
                                        &col_shape_transform,
                                        p_margin,
                                        RapierSpace::get_last_step(),
                                        p_motion,
                                    ) {
                                        continue;
                                    }
                                    let a = vector_to_godot(contact.pixel_point1);
                                    let b = vector_to_godot(contact.pixel_point2);
                                    recovered = true;
                                    // Compute plane on b towards a.
                                    let n = vector_to_godot(contact.normal1);
                                    // Move it outside as to fit the margin
                                    let d = n.dot(b);
                                    // Compute depth on recovered motion.
                                    let depth = n.dot(a + recover_step) - d;
                                    if depth > min_contact_depth + DEFAULT_EPSILON {
                                        // Only recover if there is penetration.
                                        recover_step -= n
                                            * (depth - min_contact_depth)
                                            * BODY_MOTION_RECOVER_RATIO;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if recover_step == Vector::default() {
                recovered = false;
                break;
            }
            if recovered {
                *p_recover_motion += recover_step;
                p_transform.origin += recover_step;
            }
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
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) {
        let body_aabb = p_body.get_aabb(physics_shapes);
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
                physics_shapes.get(&p_body.get_base().get_shape(body_shape_idx))
            {
                let body_shape_transform =
                    *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
                let mut body_shape_info =
                    shape_info_from_body_shape(body_shape.get_handle(), body_shape_transform);
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
                let _stuck = false;
                for result_idx in 0..result_count {
                    let result_idx = result_idx as usize;
                    let result = &mut results[result_idx];
                    if !result.user_data.is_valid() {
                        continue;
                    }
                    let (shape_col_object, shape_index) =
                        RapierCollisionObject::get_collider_user_data(&result.user_data);
                    if let Some(shape_col_object) = physics_collision_objects.get(&shape_col_object)
                    {
                        if let Some(collision_body) = shape_col_object.get_body() {
                            if collision_body.has_exception(p_body.get_base().get_rid()) {
                                continue;
                            }
                            if let Some(col_shape) = physics_shapes
                                .get(&collision_body.get_base().get_shape(shape_index))
                            {
                                let col_shape_transform = collision_body.get_base().get_transform()
                                    * collision_body.get_base().get_shape_transform(shape_index);
                                let col_shape_info = shape_info_from_body_shape(
                                    col_shape.get_handle(),
                                    col_shape_transform,
                                );
                                // stuck logic, check if body collides in place
                                body_shape_info.transform.translation.vector =
                                    vector_to_rapier(body_shape_transform.origin);
                                let step_contact = physics_engine.shapes_contact(
                                    body_shape_info,
                                    col_shape_info,
                                    0.0,
                                );
                                if step_contact.collided && !step_contact.within_margin {
                                    if body_shape.allows_one_way_collision()
                                        && collision_body
                                            .get_base()
                                            .is_shape_set_as_one_way_collision(shape_index)
                                        && !p_motion.is_zero_approx()
                                    {
                                        // TODO re-enable this
                                        //let direction = col_shape_transform.b.normalized();
                                        //if p_motion.normalized().dot(direction) <= 0.0 {
                                        //    continue;
                                        //}
                                    }
                                    *p_closest_safe = 0.0;
                                    *p_closest_unsafe = 0.0;
                                    *p_best_body_shape = body_shape_idx as i32; //sadly it's the best
                                    break;
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
                                        + p_motion
                                            * (hi + self.get_contact_max_allowed_penetration()),
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
                                    body_shape.deref(),
                                    collision_body,
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
        physics_engine: &PhysicsEngine,
        physics_shapes: &PhysicsShapes,
        physics_collision_objects: &PhysicsCollisionObjects,
    ) -> bool {
        let shape_count = p_body.get_base().get_shape_count();
        if shape_count < 1 {
            return false;
        }
        let body_aabb = p_body.get_aabb(physics_shapes);
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
            let body_shape = p_body.get_base().get_shape(body_shape_idx as usize);
            let body_shape_transform = *p_transform
                * p_body
                    .get_base()
                    .get_shape_transform(body_shape_idx as usize);
            if let Some(body_shape_obj) = physics_shapes.get(&body_shape) {
                let body_shape_info =
                    shape_info_from_body_shape(body_shape_obj.get_handle(), body_shape_transform);
                for result_idx in 0..result_count {
                    let result = &mut results[result_idx as usize];
                    if !result.user_data.is_valid() {
                        continue;
                    }
                    let (shape_col_object, shape_index) =
                        RapierCollisionObject::get_collider_user_data(&result.user_data);
                    if let Some(shape_col_object) = physics_collision_objects.get(&shape_col_object)
                    {
                        if let Some(collision_body) = shape_col_object.get_body() {
                            if collision_body.has_exception(p_body.get_base().get_rid()) {
                                continue;
                            }
                            let col_shape_rid = collision_body.get_base().get_shape(shape_index);
                            if let Some(col_shape) = physics_shapes.get(&col_shape_rid) {
                                let col_shape_transform = collision_body.get_base().get_transform()
                                    * collision_body.get_base().get_shape_transform(shape_index);
                                let col_shape_info = shape_info_from_body_shape(
                                    col_shape.get_handle(),
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
                                    body_shape_obj.deref(),
                                    collision_body,
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
impl PhysicsEngine {
    #[allow(clippy::too_many_arguments)]
    fn should_skip_collision_one_dir(
        &self,
        contact: &ContactResult,
        body_shape: &dyn IRapierShape,
        collision_body: &dyn IRapierCollisionObject,
        shape_index: usize,
        col_shape_transform: &Transform,
        p_margin: f32,
        last_step: f32,
        p_motion: Vector,
    ) -> bool {
        let dist = contact.pixel_distance;
        if !contact.within_margin
            && body_shape.allows_one_way_collision()
            && collision_body
                .get_base()
                .is_shape_set_as_one_way_collision(shape_index)
        {
            let valid_dir = vector_normalized(col_shape_transform.origin);
            let owc_margin = collision_body
                .get_base()
                .get_shape_one_way_collision_margin(shape_index);
            let mut valid_depth = owc_margin.max(p_margin);
            if let Some(b) = collision_body.get_body() {
                if b.get_base().mode.ord() >= BodyMode::KINEMATIC.ord() {
                    // fix for moving platforms (kinematic and dynamic), margin is increased by how much it moved in the
                    // given direction
                    let lv = b.get_linear_velocity(self);
                    // compute displacement from linear velocity
                    let mut motion = lv * last_step;
                    let motion_len = motion.length();
                    if !motion_len.is_zero_approx() {
                        motion = vector_normalized(motion);
                    }
                    valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
                }
            }
            let motion = p_motion;
            let mut motion_normalized = motion;
            let motion_len = motion.length();
            if !motion_len.is_zero_approx() {
                motion_normalized = vector_normalized(motion_normalized);
            }
            valid_depth += motion_len * motion_normalized.dot(valid_dir).max(0.0);
            if dist < -valid_depth || motion_normalized.dot(valid_dir) < DEFAULT_EPSILON {
                return true;
            }
        }
        false
    }
}
