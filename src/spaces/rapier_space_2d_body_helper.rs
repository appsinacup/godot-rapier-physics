use crate::bodies::rapier_body_2d::RapierBody2D;
use crate::bodies::rapier_collision_object_2d::{CollisionObjectType, IRapierCollisionObject2D};
use crate::rapier2d::handle::Handle;
use crate::rapier2d::query::{default_query_excluded_info, intersect_aabb, ContactResult};
use crate::rapier2d::vector::Vector;
use crate::shapes::rapier_shape_2d::IRapierShape2D;
use crate::{
    rapier2d::{
        query::{PointHitInfo},
    },
};
use godot::engine::physics_server_2d::BodyMode;
use godot::{
    engine::{
        native::{PhysicsServer2DExtensionMotionResult},
    },
    prelude::*,
};

use std::f32::EPSILON;
use super::rapier_direct_space_state_2d::is_handle_excluded_callback;
use super::rapier_space_2d::RapierSpace2D;

const TEST_MOTION_MARGIN: f64 = 1e-4;

const TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR: f32 = 0.05;
const BODY_MOTION_RECOVER_ATTEMPTS: i32 = 4;
const BODY_MOTION_RECOVER_RATIO: f32 = 0.4;


impl RapierSpace2D {
    pub fn test_body_motion(
        &self,
        body: Rid,
        from: Transform2D,
        motion: Vector2,
        margin: f64,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: &PhysicsServer2DExtensionMotionResult,
    ) -> bool {
        /*
        result.travel = Vector2::default();
	    let mut body_transform = from.clone(); // Because body_transform needs to be modified during recovery
        // Step 1: recover motion.
        // Expand the body colliders by the margin (grow) and check if now it collides with a collider,
        // if yes, "recover" / "push" out of this collider
	    let recover_motion;
        let margin = f64::max(margin, TEST_MOTION_MARGIN);

	    let recovered = false;//RapierBodyUtils2D::body_motion_recover(*this, *p_body, body_transform, p_motion, margin, recover_motion);
        // Step 2: Cast motion.
        // Try to to find what is the possible motion (how far it can move, it's a shapecast, when you try to find the safe point (max you can move without collision ))
        let best_safe = 1.0;
        let best_unsafe = 1.0;
        let mut best_body_shape = -1;
	    //RapierBodyUtils2D::cast_motion(*this, *p_body, body_transform, p_motion, p_collide_separation_ray, contact_max_allowed_penetration, margin, best_safe, best_unsafe, best_body_shape);

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

            collided = false;// RapierBodyUtils2D::body_motion_collide(*this, *p_body, body_transform, p_motion, best_body_shape, margin, r_result);
        }

		if collided {
			result.travel += recover_motion + motion * best_safe;
			result.remainder = motion - motion * best_safe;
			result.collision_safe_fraction = best_safe;
			result.collision_unsafe_fraction = best_unsafe;
		} else {
			result.travel += recover_motion + motion;
			result.remainder = Vector2::default();
			result.collision_depth = 0.0;
			result.collision_safe_fraction = 1.0;
			result.collision_unsafe_fraction = 1.0;
		}

        collided
         */
        false
    }

    pub fn rapier_intersect_aabb(
        &self,
        aabb: Rect2,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: &mut [PointHitInfo],
        max_results: usize,
        exclude_body: Rid,
    ) -> i32 {
        let max_results = max_results;
        if max_results < 1 {
            return 0;
        }

        let rect_begin = Vector::new( aabb.position.x, aabb.position.y );
        let rect_end = Vector::new( aabb.end().x, aabb.end().y );
        let mut handle_excluded_info = default_query_excluded_info();
        let query_exclude: Vec<Handle> = Vec::with_capacity(max_results);
        handle_excluded_info.query_exclude = query_exclude;
        handle_excluded_info.query_collision_layer_mask = collision_mask;
        handle_excluded_info.query_exclude_size = 0;
        handle_excluded_info.query_exclude_body = exclude_body.to_u64() as i64;
    
        intersect_aabb(self.handle, &rect_begin, &rect_end, collide_with_bodies, collide_with_areas, results, max_results, is_handle_excluded_callback, &handle_excluded_info) as i32
    }


fn body_motion_recover(
    &mut self,
    p_body: &mut RapierBody2D,
    p_transform: &mut Transform2D,
    p_motion: Vector2,
    p_margin: f32,
    p_recover_motion: &mut Vector2,
) -> bool {
    /*
    let shape_count = p_body.get_base().get_shape_count();
    if shape_count < 1 {
        return false;
    }
    let min_contact_depth = p_margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;

    let mut recovered = false;
    let mut recover_attempts = BODY_MOTION_RECOVER_ATTEMPTS;
    loop {
        let mut results = [PointHitInfo::default(); 32];

        let body_aabb = p_body.get_aabb();
        // Undo the currently transform the physics server is aware of and apply the provided one
        let margin_aabb = p_transform.basis_xform(body_aabb);
        let margin_aabb = margin_aabb.grow(p_margin);
        
        let result_count = self.rapier_intersect_aabb(
            margin_aabb,
            p_body.get_base().get_collision_mask(),
            true,
            false,
            &mut results,
            32,
            p_body.get_base().get_rid(),
        );
        // Optimization
        if result_count == 0 {
            break;
        }

        let mut recover_step = Vector2::default();

        for body_shape_idx in 0..p_body.get_base().get_shape_count() {
            if p_body.get_base().is_shape_disabled(body_shape_idx) {
                continue;
            }

            let body_shape = p_body.get_base().get_shape(body_shape_idx);
            let body_shape_transform = *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
            let body_shape_info =
                shape_info_from_body_shape(body_shape.get_rapier_shape(), body_shape_transform);

            for result_idx in 0..result_count {
                let result = &mut results[result_idx];
                if !result.user_data.is_valid() {
                    continue;
                }
                let (shape_index, shape_col_object) =
                    RapierCollisionObject2D::get_collider_user_data(result.user_data);
                if shape_col_object.is_none() {
                    continue;
                }
                let shape_col_object = shape_col_object.unwrap();
                if shape_col_object.get_type() != RapierCollisionObject2D::TYPE_BODY {
                    continue;
                }
                let collision_body = shape_col_object.downcast_ref::<RapierBody2D>().unwrap();

                let col_shape = collision_body.get_shape(shape_index).unwrap();

                let col_shape_transform =
                    collision_body.get_transform() * collision_body.get_shape_transform(shape_index);
                let col_shape_info =
                    shape_info_from_body_shape(col_shape.get_rapier_shape(), col_shape_transform);

                let contact = shapes_contact(body_shape_info, col_shape_info, p_margin);

                if !contact.collided {
                    continue;
                }
                if should_skip_collision_one_dir(
                    contact,
                    body_shape,
                    collision_body,
                    shape_index,
                    &col_shape_transform,
                    p_margin,
                    self.get_last_step(),
                    p_motion,
                ) {
                    continue;
                }
                let a = Vector2::new(contact.pixel_point1.x, contact.pixel_point1.y);
                let b = Vector2::new(contact.pixel_point2.x, contact.pixel_point2.y);

                recovered = true;

                // Compute plane on b towards a.
                let n = Vector2::new(contact.normal1.x, contact.normal1.y);
                // Move it outside as to fit the margin
                let d = n.dot(b);

                // Compute depth on recovered motion.
                let depth = n.dot(a + recover_step) - d;
                if depth > min_contact_depth + EPSILON {
                    // Only recover if there is penetration.
                    recover_step -= n * (depth - min_contact_depth) * BODY_MOTION_RECOVER_RATIO;
                }
            }
        }
        if recover_step == Vector2::default() {
            recovered = false;
            break;
        }
        if recovered {
            *p_recover_motion += recover_step;
            p_transform.columns[2] += recover_step;
        }
        recover_attempts -= 1;
        if recover_attempts == 0 {
            break;
        }
    }

    recovered
     */
    false
}

fn cast_motion(
    &mut self,
    p_body: &mut RapierBody2D,
    p_transform: &Transform2D,
    p_motion: Vector2,
    p_collide_separation_ray: bool,
    contact_max_allowed_penetration: f32,
    p_margin: f32,
    p_closest_safe: &mut f32,
    p_closest_unsafe: &mut f32,
    p_best_body_shape: &mut i32,
) {
    /*
    let body_aabb = p_body.get_aabb();
    let margin_aabb = p_transform.basis_xform(body_aabb);

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
    );

    if result_count == 0 {
        return;
    }

    for body_shape_idx in 0..p_body.get_base().get_shape_count() {
        if p_body.get_base().is_shape_disabled(body_shape_idx) {
            continue;
        }

        let body_shape = p_body.get_base().get_shape(body_shape_idx);
        let body_shape_transform = *p_transform * p_body.get_base().get_shape_transform(body_shape_idx);
        let body_shape_info =
            shape_info_from_body_shape(body_shape.get_rapier_shape(), body_shape_transform);

        // Colliding separation rays allows to properly snap to the ground,
        // otherwise it's not needed in regular motion.
        //if !p_collide_separation_ray
        //    && body_shape.get_type() == PhysicsServer2D::SHAPE_SEPARATION_RAY
        //{
            // When slide on slope is on, separation ray shape acts like a
            // regular shape.
            //if !body_shape.downcast_ref::<RapierSeparationRayShape2D>().unwrap().get_slide_on_slope()
            //{
            //    continue;
            //}
        //}

        let mut stuck = false;
        let mut best_safe = 1.0;
        let mut best_unsafe = 1.0;

        for result_idx in 0..result_count {
            let result = &mut results[result_idx];

            if !result.user_data.is_valid() {
                continue;
            }
            let (shape_index, shape_col_object) =
                RapierCollisionObject2D::get_collider_user_data(result.user_data);
            if shape_col_object.is_none() {
                continue;
            }
            let shape_col_object = shape_col_object.unwrap();
            if shape_col_object.get_type() != RapierCollisionObject2D::TYPE_BODY {
                continue;
            }
            let collision_body = shape_col_object.downcast_ref::<RapierBody2D>().unwrap();

            let col_shape = collision_body.get_shape(shape_index).unwrap();

            let col_shape_transform =
                collision_body.get_transform() * collision_body.get_shape_transform(shape_index);
            let col_shape_info =
                shape_info_from_body_shape(col_shape.get_rapier_shape(), col_shape_transform);

            let contact = shapes_contact(body_shape_info, col_shape_info, p_margin);

            if !contact.collided {
                continue;
            }
            if should_skip_collision_one_dir(
                contact,
                body_shape,
                collision_body,
                shape_index,
                &col_shape_transform,
                p_margin,
                self.get_last_step(),
                p_motion,
            ) {
                continue;
            }
            let a = Vector2::new(contact.pixel_point1.x, contact.pixel_point1.y);
            let b = Vector2::new(contact.pixel_point2.x, contact.pixel_point2.y);

            stuck = true;

            // Compute plane on b towards a.
            let n = Vector2::new(contact.normal1.x, contact.normal1.y);
            // Move it outside as to fit the margin
            let d = n.dot(b);

            // Compute depth on recovered motion.
            let depth = n.dot(a + p_motion) - d;
            if depth > EPSILON {
                // Only recover if there is penetration.
                best_safe = best_safe.min(0.0);
                best_unsafe = best_unsafe.min(1.0);
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
     */
}

fn body_motion_collide(
    &mut self,
    p_body: &mut RapierBody2D,
    p_transform: &Transform2D,
    p_motion: Vector2,
    p_best_body_shape: i32,
    p_margin: f32,
    p_result: Option<&mut PhysicsServer2DExtensionMotionResult>,
) -> bool {
    /*
    let shape_count = p_body.get_base().get_shape_count();
    if shape_count < 1 {
        return false;
    }
    let body_aabb = p_body.get_aabb();
    let margin_aabb = p_transform.basis_xform(body_aabb);
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
        let mut shapes_lock = shapes_singleton().lock().unwrap();
        let body_shape = p_body.get_base().get_shape(body_shape_idx as usize);
        let body_shape_transform = *p_transform * p_body.get_base().get_shape_transform(body_shape_idx as usize);
        let mut body_shape_obj = shapes_lock.shapes.get(&body_shape).unwrap();
        let body_shape_info =
            shape_info_from_body_shape(body_shape_obj.get_rapier_shape(), body_shape_transform);

        for result_idx in 0..result_count {
            let result = &mut results[result_idx as usize];

            if !is_user_data_valid(result.user_data) {
                continue;
            }
            let (shape_col_object, shape_index) =
                RapierCollisionObject2D::get_collider_user_data(&result.user_data);
            if shape_col_object.is_invalid() {
                continue;
            }
            let bodies_lock = bodies_singleton().lock().unwrap();
            let shape_col_object = bodies_lock.collision_objects.get(&shape_col_object).unwrap();
            if shape_col_object.get_base().get_type() != CollisionObjectType::Body {
                continue;
            }
            let collision_body = shape_col_object.get_body().unwrap();

            let col_shape_rid = collision_body.get_base().get_shape(shape_index);
            let col_shape = shapes_lock.shapes.get_mut(&col_shape_rid).unwrap();

            let col_shape_transform =
                collision_body.get_base().get_transform() * collision_body.get_base().get_shape_transform(shape_index);
            let col_shape_info =
                shape_info_from_body_shape(col_shape.get_rapier_shape(), col_shape_transform);

            let contact = shapes_contact(body_shape_info, col_shape_info, p_margin);
            if !contact.collided {
                continue;
            }

            let a = Vector2::new(contact.pixel_point1.x, contact.pixel_point1.y);
            let b = Vector2::new(contact.pixel_point2.x, contact.pixel_point2.y);

            if should_skip_collision_one_dir(
                contact,
                body_shape_obj,
                collision_body,
                shape_index,
                &col_shape_transform,
                p_margin,
                self.get_last_step(),
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
    if let Some(best_collision_body) = best_collision_body {
        if let Some(p_result) = p_result {
            // conveyer belt
            if best_collision_body.get_static_linear_velocity() != Vector2::default() {
                p_result.travel += best_collision_body.get_static_linear_velocity() * self.get_last_step();
            }
            p_result.collider = best_collision_body.get_base().get_rid();
            p_result.collider_id = ObjectId{id : best_collision_body.get_base().get_instance_id()};
            p_result.collider_shape = best_collision_shape_index as i32;
            p_result.collision_local_shape = best_body_shape_index;
            // World position from the moving body to get the contact point
            p_result.collision_point = Vector2::new(best_contact.pixel_point1.x, best_contact.pixel_point1.y);
            // Normal from the collided object to get the contact normal
            p_result.collision_normal = Vector2::new(best_contact.normal2.x, best_contact.normal2.y);
            // compute distance without sign
            p_result.collision_depth = p_margin - best_contact.pixel_distance;

            let local_position = p_result.collision_point - best_collision_body.get_base().get_transform().origin;
            p_result.collider_velocity = best_collision_body.get_velocity_at_local_point(local_position);
        }

        return true;
    } */

    false
}

}


fn should_skip_collision_one_dir(
    contact: ContactResult,
    body_shape: &Box<dyn IRapierShape2D>,
    collision_body: &dyn IRapierCollisionObject2D,
    shape_index: usize,
    col_shape_transform: &Transform2D,
    p_margin: f32,
    last_step: f32,
    p_motion: Vector2,
) -> bool {
    let dist = contact.pixel_distance;
    if !contact.within_margin
        && body_shape.allows_one_way_collision()
        && collision_body
            .get_base()
            .is_shape_set_as_one_way_collision(shape_index)
    {
        let valid_dir = col_shape_transform.origin.normalized();

        let owc_margin = collision_body
            .get_base()
            .get_shape_one_way_collision_margin(shape_index);
        let mut valid_depth = owc_margin.max(p_margin);

        if collision_body.get_base().get_type() == CollisionObjectType::Body {
            let b = collision_body.get_body().unwrap();
            if b.get_base().mode.ord() >= BodyMode::KINEMATIC.ord() {
                // fix for moving platforms (kinematic and dynamic), margin is increased by how much it moved in the
                // given direction
                let lv = b.get_linear_velocity();
                // compute displacement from linear velocity
                let motion = lv * last_step;
                let motion_len = motion.length();
                let motion = motion.normalized();
                valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
            }
        }
        let motion = p_motion;
        let motion_len = motion.length();
        let motion = motion.normalized();
        valid_depth += motion_len * motion.dot(valid_dir).max(0.0);
        if dist < -valid_depth || p_motion.normalized().dot(valid_dir) < EPSILON {
            return true;
        }
    }
    false
}
