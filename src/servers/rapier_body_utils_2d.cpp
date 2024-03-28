#include "rapier_body_utils_2d.h"
#include "../bodies/rapier_body_2d.h"
#include "../shapes/rapier_separation_ray_shape_2d.h"
#include "../shapes/rapier_shape_2d.h"
#include "../spaces/rapier_space_2d.h"

#define TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR 0.05
#define BODY_MOTION_RECOVER_ATTEMPTS 4
#define BODY_MOTION_RECOVER_RATIO 0.4

bool should_skip_collision_one_dir(rapier2d::ContactResult contact, RapierShape2D *body_shape, RapierBody2D *collision_body, int shape_index, const Transform2D &col_shape_transform, real_t p_margin, real_t last_step, Vector2 p_motion) {
	real_t dist = contact.distance;
	if (!contact.within_margin && body_shape->allows_one_way_collision() && collision_body->is_shape_set_as_one_way_collision(shape_index)) {
		real_t valid_depth = 10e20;
		Vector2 valid_dir = col_shape_transform.columns[1].normalized();

		real_t owc_margin = collision_body->get_shape_one_way_collision_margin(shape_index);
		valid_depth = MAX(owc_margin, p_margin);

		if (collision_body->get_type() == RapierCollisionObject2D::TYPE_BODY) {
			const RapierBody2D *b = collision_body;
			if (b->get_mode() == PhysicsServer2D::BODY_MODE_KINEMATIC || b->get_mode() == PhysicsServer2D::BODY_MODE_RIGID) {
				//fix for moving platforms (kinematic and dynamic), margin is increased by how much it moved in the
				//given direction
				Vector2 lv = b->get_linear_velocity();
				//compute displacement from linear velocity
				Vector2 motion = lv * last_step;
				real_t motion_len = motion.length();
				motion.normalize();
				valid_depth += motion_len * MAX(motion.dot(valid_dir), 0.0);
			}
		}
		Vector2 motion = p_motion;
		real_t motion_len = motion.length();
		valid_depth += motion_len * MAX(motion.normalized().dot(valid_dir), 0.0);
		if ((dist < -valid_depth) || (p_motion.normalized().dot(valid_dir) < CMP_EPSILON * 10.0)) {
			return true;
		}
	}
	return false;
}

bool RapierBodyUtils2D::body_motion_recover(
		const RapierSpace2D &p_space, RapierBody2D &p_body, Transform2D &p_transform, const Vector2 &p_motion, real_t p_margin, Vector2 &p_recover_motion) {
	int shape_count = p_body.get_shape_count();
	ERR_FAIL_COND_V(shape_count < 1, false);
	real_t min_contact_depth = p_margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;

	bool recovered = false;
	int recover_attempts = BODY_MOTION_RECOVER_ATTEMPTS;
	do {
		rapier2d::PointHitInfo results[32];

		Rect2 body_aabb = p_body.get_aabb();
		// Undo the currently transform the physics server is aware of and apply the provided one
		Rect2 margin_aabb = p_transform.xform(body_aabb);
		margin_aabb = margin_aabb.grow(p_margin);

		int result_count = p_space.rapier_intersect_aabb(
				margin_aabb, p_body.get_collision_mask(), true, false, results, 32, &result_count, p_body.get_rid());
		// Optimization
		if (result_count == 0) {
			break;
		}

		Vector2 recover_step;

		for (int body_shape_idx = 0; body_shape_idx < p_body.get_shape_count(); body_shape_idx++) {
			if (p_body.is_shape_disabled(body_shape_idx)) {
				continue;
			}

			RapierShape2D *body_shape = p_body.get_shape(body_shape_idx);
			Transform2D const &body_shape_transform = p_transform * p_body.get_shape_transform(body_shape_idx);
			rapier2d::ShapeInfo body_shape_info =
					rapier2d::shape_info_from_body_shape(body_shape->get_rapier_shape(), body_shape_transform);

			for (int result_idx = 0; result_idx < result_count; ++result_idx) {
				rapier2d::PointHitInfo &result = results[result_idx];
				ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
				uint32_t shape_index = 0;
				RapierCollisionObject2D *shape_col_object = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
				ERR_CONTINUE(!shape_col_object);
				ERR_CONTINUE(shape_col_object->get_type() != RapierCollisionObject2D::TYPE_BODY);
				RapierBody2D *collision_body = static_cast<RapierBody2D *>(shape_col_object);

				RapierShape2D *col_shape = collision_body->get_shape(shape_index);

				Transform2D const &col_shape_transform = collision_body->get_transform() * collision_body->get_shape_transform(shape_index);
				rapier2d::ShapeInfo col_shape_info =
						rapier2d::shape_info_from_body_shape(col_shape->get_rapier_shape(), col_shape_transform);

				rapier2d::ContactResult contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_info, col_shape_info, p_margin);

				if (!contact.collided) {
					continue;
				}
				if (should_skip_collision_one_dir(contact, body_shape, collision_body, shape_index, col_shape_transform, p_margin, p_space.get_last_step(), p_motion)) {
					continue;
				}
				Vector2 a(contact.point1.x, contact.point1.y);
				Vector2 b(contact.point2.x, contact.point2.y);

				recovered = true;

				// Compute plane on b towards a.
				Vector2 n = Vector2(contact.normal1.x, contact.normal1.y);
				// Move it outside as to fit the margin
				real_t d = n.dot(b);

				// Compute depth on recovered motion.
				real_t depth = n.dot(a + recover_step) - d;
				if (depth > min_contact_depth + CMP_EPSILON) {
					// Only recover if there is penetration.
					recover_step -= n * (depth - min_contact_depth) * BODY_MOTION_RECOVER_RATIO;
				}
			}
		}
		if (recover_step == Vector2()) {
			recovered = false;
			break;
		}
		if (recovered) {
			p_recover_motion += recover_step;
			p_transform.columns[2] += recover_step;
		}
		recover_attempts--;
	} while (recover_attempts);

	return recovered;
}

void RapierBodyUtils2D::cast_motion(const RapierSpace2D &p_space, RapierBody2D &p_body, const Transform2D &p_transform,
		const Vector2 &p_motion, bool p_collide_separation_ray, real_t contact_max_allowed_penetration, real_t p_margin,
		real_t &p_closest_safe, real_t &p_closest_unsafe, int &p_best_body_shape) {
	Rect2 body_aabb = p_body.get_aabb();
	Rect2 margin_aabb = p_transform.xform(body_aabb);

	margin_aabb = margin_aabb.grow(p_margin);
	Rect2 motion_aabb = margin_aabb;
	motion_aabb.position += p_motion;
	motion_aabb = motion_aabb.merge(margin_aabb);

	rapier2d::PointHitInfo results[32];
	int result_count = p_space.rapier_intersect_aabb(
			motion_aabb, p_body.get_collision_mask(), true, false, results, 32, &result_count, p_body.get_rid());

	if (result_count == 0) {
		return;
	}

	for (int body_shape_idx = 0; body_shape_idx < p_body.get_shape_count(); body_shape_idx++) {
		if (p_body.is_shape_disabled(body_shape_idx)) {
			continue;
		}

		RapierShape2D *body_shape = p_body.get_shape(body_shape_idx);
		Transform2D const &body_shape_transform = p_transform * p_body.get_shape_transform(body_shape_idx);
		rapier2d::ShapeInfo body_shape_info = rapier2d::shape_info_from_body_shape(body_shape->get_rapier_shape(), body_shape_transform);

		// Colliding separation rays allows to properly snap to the ground,
		// otherwise it's not needed in regular motion.
		if (!p_collide_separation_ray && (body_shape->get_type() == PhysicsServer2D::SHAPE_SEPARATION_RAY)) {
			// When slide on slope is on, separation ray shape acts like a
			// regular shape.
			if (!static_cast<RapierSeparationRayShape2D *>(body_shape)->get_slide_on_slope()) {
				continue;
			}
		}

		bool stuck = false;
		real_t best_safe = 1.0;
		real_t best_unsafe = 1.0;

		for (int result_idx = 0; result_idx < result_count; ++result_idx) {
			rapier2d::PointHitInfo &result = results[result_idx];

			ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
			uint32_t shape_index = 0;
			RapierCollisionObject2D *shape_col_object = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
			ERR_CONTINUE(!shape_col_object);

			ERR_CONTINUE(shape_col_object->get_type() != RapierCollisionObject2D::TYPE_BODY);
			RapierBody2D *collision_body = static_cast<RapierBody2D *>(shape_col_object);
			RapierShape2D *col_shape = collision_body->get_shape(shape_index);
			Transform2D const &col_shape_transform = collision_body->get_transform() * collision_body->get_shape_transform(shape_index);
			rapier2d::ShapeInfo col_shape_info = rapier2d::shape_info_from_body_shape(col_shape->get_rapier_shape(), col_shape_transform);
			// stuck logic, check if body collides in place
			body_shape_info.pixel_position = { (body_shape_transform.get_origin()).x, (body_shape_transform.get_origin()).y };
			rapier2d::ContactResult step_contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_info, col_shape_info, 0.0);
			if (step_contact.collided && !step_contact.within_margin) {
				if (body_shape->allows_one_way_collision() && collision_body->is_shape_set_as_one_way_collision(shape_index)) {
					Vector2 direction = col_shape_transform.columns[1].normalized();
					if (p_motion.normalized().dot(direction) < 0) {
						continue;
					}
				}
				p_closest_safe = 0;
				p_closest_unsafe = 0;
				p_best_body_shape = body_shape_idx; //sadly it's the best
				break;
			}

			//just do kinematic solving
			real_t low = 0.0;
			real_t hi = 1.0;
			real_t fraction_coeff = 0.5;

			for (int k = 0; k < 8; k++) {
				real_t fraction = low + (hi - low) * fraction_coeff;

				body_shape_info.pixel_position = rapier2d::Vector{ (body_shape_transform.get_origin() + p_motion * fraction).x,
					(body_shape_transform.get_origin() + p_motion * fraction).y };
				rapier2d::ContactResult step_contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_info, col_shape_info, 0.0);
				if (step_contact.collided && !step_contact.within_margin) {
					hi = fraction;
					if ((k == 0) || (low > 0.0)) { // Did it not collide before?
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
					if ((k == 0) || (hi < 1.0)) { // Did it collide before?
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
			body_shape_info.pixel_position =
					rapier2d::Vector{ (body_shape_transform.get_origin() + p_motion * (hi + contact_max_allowed_penetration)).x,
						(body_shape_transform.get_origin() + p_motion * (hi + contact_max_allowed_penetration)).y };
			rapier2d::ContactResult contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_info, col_shape_info, 0.0);
			if (should_skip_collision_one_dir(contact, body_shape, collision_body, shape_index, col_shape_transform, p_margin, p_space.get_last_step(), p_motion)) {
				continue;
			}
			if (low < best_safe) {
				best_safe = low;
				best_unsafe = hi;
			}
		}

		if (best_safe == 1.0) {
			continue;
		}
		if (best_safe < p_closest_safe) {
			p_closest_safe = best_safe;
			p_closest_unsafe = best_unsafe;
			p_best_body_shape = body_shape_idx;
		}
	}
}

bool RapierBodyUtils2D::body_motion_collide(const RapierSpace2D &p_space, RapierBody2D &p_body, const Transform2D &p_transform,
		const Vector2 &p_motion, int p_best_body_shape, real_t p_margin, PhysicsServer2DExtensionMotionResult *p_result) {
	int shape_count = p_body.get_shape_count();
	ERR_FAIL_COND_V(shape_count < 1, false);
	Rect2 body_aabb = p_body.get_aabb();
	Rect2 margin_aabb = p_transform.xform(body_aabb);
	margin_aabb = margin_aabb.grow(p_margin);

	// also check things at motion
	Rect2 motion_aabb = margin_aabb;
	motion_aabb.position += p_motion;
	motion_aabb = motion_aabb.merge(margin_aabb);

	rapier2d::PointHitInfo results[32];
	int result_count = p_space.rapier_intersect_aabb(
			motion_aabb, p_body.get_collision_mask(), true, false, results, 32, &result_count, p_body.get_rid());
	// Optimization
	if (result_count == 0) {
		return false;
	}

	real_t min_distance = INFINITY;
	RapierBody2D *best_collision_body = nullptr;
	int best_collision_shape_index = -1;
	int best_body_shape_index = -1;
	rapier2d::ContactResult best_contact;

	int from_shape = p_best_body_shape != -1 ? p_best_body_shape : 0;
	int to_shape = p_best_body_shape != -1 ? p_best_body_shape + 1 : p_body.get_shape_count();
	for (int body_shape_idx = from_shape; body_shape_idx < to_shape; body_shape_idx++) {
		if (p_body.is_shape_disabled(body_shape_idx)) {
			continue;
		}

		RapierShape2D *body_shape = p_body.get_shape(body_shape_idx);
		Transform2D const &body_shape_transform = p_transform * p_body.get_shape_transform(body_shape_idx);
		rapier2d::ShapeInfo body_shape_info = rapier2d::shape_info_from_body_shape(body_shape->get_rapier_shape(), body_shape_transform);

		for (int result_idx = 0; result_idx < result_count; ++result_idx) {
			rapier2d::PointHitInfo &result = results[result_idx];

			ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
			uint32_t shape_index = 0;
			RapierCollisionObject2D *shape_col_object = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
			ERR_CONTINUE(!shape_col_object);

			ERR_CONTINUE(shape_col_object->get_type() != RapierCollisionObject2D::TYPE_BODY);
			RapierBody2D *collision_body = static_cast<RapierBody2D *>(shape_col_object);

			RapierShape2D *col_shape = collision_body->get_shape(shape_index);
			Transform2D const &col_shape_transform = collision_body->get_transform() * collision_body->get_shape_transform(shape_index);
			rapier2d::ShapeInfo col_shape_info = rapier2d::shape_info_from_body_shape(col_shape->get_rapier_shape(), col_shape_transform);

			rapier2d::ContactResult contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_info, col_shape_info, p_margin);
			if (!contact.collided) {
				continue;
			}

			Vector2 a(contact.point1.x, contact.point1.y);
			Vector2 b(contact.point2.x, contact.point2.y);

			if (should_skip_collision_one_dir(contact, body_shape, collision_body, shape_index, col_shape_transform, p_margin, p_space.get_last_step(), p_motion)) {
				continue;
			}
			if (contact.distance < min_distance) {
				min_distance = contact.distance;
				best_collision_body = collision_body;
				best_collision_shape_index = shape_index;
				best_body_shape_index = body_shape_idx;
				best_contact = contact;
			}
		}
	}
	if (best_collision_body) {
		// conveyer belt
		if (best_collision_body->get_static_linear_velocity() != Vector2()) {
			p_result->travel += best_collision_body->get_static_linear_velocity() * p_space.get_last_step();
		}
		if (p_result) {
			p_result->collider = best_collision_body->get_rid();
			p_result->collider_id = best_collision_body->get_instance_id();
			p_result->collider_shape = best_collision_shape_index;
			p_result->collision_local_shape = best_body_shape_index;
			// World position from the moving body to get the contact point
			p_result->collision_point = Vector2(best_contact.point1.x, best_contact.point1.y);
			// Normal from the collided object to get the contact normal
			p_result->collision_normal = Vector2(best_contact.normal2.x, best_contact.normal2.y);
			// compute distance without sign
			p_result->collision_depth = p_margin - best_contact.distance;

			Vector2 local_position = p_result->collision_point - best_collision_body->get_transform().get_origin();
			p_result->collider_velocity = best_collision_body->get_velocity_at_local_point(local_position);
		}

		return true;
	}

	return false;
}
