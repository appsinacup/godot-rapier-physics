#include "rapier_body_2d.h"
#include "rapier_space_2d.h"
#include "rapier_shape_2d.h"
#include "rapier_body_utils_2d.h"

#define TEST_MOTION_MARGIN_MIN_VALUE 0.0001
#define TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR 0.05

bool RapierBodyUtils2D::body_motion_recover(
    const RapierSpace2D& p_space,
	const RapierBody2D& p_body,
	Transform2D& p_transform,
	real_t p_margin,
	Vector2& p_recover_motion,
	Rect2& p_body_aabb
) {
	int shape_count = p_body.get_shape_count();
	ERR_FAIL_COND_V(shape_count < 1, false);

	// Create compound shape for the body if needed
	/*rapier2d::Handle body_shape_handle;
	if (shape_count > 1) {
		rapier2d::ShapeInfo* shapes = (rapier2d::ShapeInfo*)alloca((shape_count) * sizeof(rapier2d::ShapeInfo));
		for (int i = 0; i < shape_count; ++i) {
			if (p_body.is_shape_disabled(i)) {
				continue;
			}
			rapier2d::ShapeInfo& shape_info = shapes[i];
			shape_info.handle = p_body.get_shape(i)->get_rapier_shape();
			Transform2D const& shape_transform = p_body.get_shape_transform(i);
			Vector2 const& shape_pos = shape_transform.get_origin();
			shape_info.position.x = shape_pos.x;
			shape_info.position.y = shape_pos.y;
			shape_info.rotation = shape_transform.get_rotation();
		}

		body_shape_handle = rapier2d::shape_create_compound(shapes, shape_count);
	} else {
		body_shape_handle = p_body.get_shape(0)->get_rapier_shape();
	}*/

	real_t margin = MAX(p_margin, TEST_MOTION_MARGIN_MIN_VALUE);
	real_t min_contact_depth = margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;

	// Undo the currently transform the physics server is aware of and apply the provided one
	p_body_aabb = p_transform.xform(p_body_aabb);
	Rect2 margin_body_aabb = p_body_aabb.grow(margin);

	bool recovered = false;
	int recover_attempts = 4;
	do {
		rapier2d::PointHitInfo results[32];
		int result_count = p_space.rapier_intersect_aabb(margin_body_aabb, p_body.get_collision_mask(), true, false, results, 32, &result_count, p_body.get_rid());
		
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
			rapier2d::Handle body_shape_handle = body_shape->get_rapier_shape();
			Transform2D const& body_shape_transform = p_transform * p_body.get_shape_transform(body_shape_idx);
			Vector2 body_shape_pos = body_shape_transform.get_origin();
			rapier2d::Vector rapier_body_shape_pos {body_shape_pos.x, body_shape_pos.y};
			real_t rapier_body_shape_rot = body_shape_transform.get_rotation();

			for (int result_idx = 0; result_idx < result_count; ++result_idx) {
				rapier2d::PointHitInfo& result = results[result_idx];

				ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
				uint32_t shape_index = 0;
				RapierCollisionObject2D* shape_col_object = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
				ERR_CONTINUE(!shape_col_object);

				ERR_CONTINUE(shape_col_object->get_type() != RapierCollisionObject2D::TYPE_BODY);
				RapierBody2D* collision_body = static_cast<RapierBody2D*>(shape_col_object);

				RapierShape2D *col_shape = collision_body->get_shape(shape_index);
				rapier2d::Handle col_shape_handle = col_shape->get_rapier_shape();
				Transform2D const& col_shape_transform = collision_body->get_transform() * collision_body->get_shape_transform(shape_index);
				Vector2 col_shape_pos = col_shape_transform.get_origin();
				rapier2d::Vector rapier_col_shape_pos {col_shape_pos.x, col_shape_pos.y};
				real_t rapier_col_shape_rot = col_shape_transform.get_rotation();

				rapier2d::ContactResult contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_handle, &rapier_body_shape_pos, rapier_body_shape_rot, col_shape_handle, &rapier_col_shape_pos, rapier_col_shape_rot, margin);
				if (!contact.collided) {
					continue;
				}

				recovered = true;

				Vector2 a(contact.point1.x, contact.point1.y);
				Vector2 b(contact.point2.x, contact.point2.y);

				// Compute plane on b towards a.
				Vector2 n = (a - b).normalized();
				real_t d = n.dot(b);

				// Compute depth on recovered motion.
				real_t depth = n.dot(a + recover_step) - d;
				if (depth > min_contact_depth + CMP_EPSILON) {
					// Only recover if there is penetration.
					recover_step -= n * (depth - min_contact_depth) * 0.4f;
				}
			}
		}

		if (recovered) {
			p_recover_motion += recover_step;
			p_transform.columns[2] += recover_step;
			p_body_aabb.position += recover_step;
			margin_body_aabb.position += recover_step;
		} else {
			break;
		}
	} while (--recover_attempts);

	return recovered;
}

void RapierBodyUtils2D::cast_motion(
        const RapierSpace2D& p_space,
 		const RapierBody2D& p_body,
        const Transform2D& p_transform,
        const Vector2& p_motion,
		const Rect2& p_body_aabb,
        real_t& p_closest_safe,
        real_t& p_closest_unsafe,
		int& p_best_body_shape
    ) {

	Rect2 motion_aabb = p_body_aabb;
	motion_aabb.position += p_motion;
	motion_aabb = motion_aabb.merge(p_body_aabb);

	rapier2d::PointHitInfo results[32];
	int result_count = p_space.rapier_intersect_aabb(motion_aabb, p_body.get_collision_mask(), true, false, results, 32, &result_count, p_body.get_rid());
	
	if (result_count == 0) {
		return;
	}

	for (int body_shape_idx = 0; body_shape_idx < p_body.get_shape_count(); body_shape_idx++) {
		if (p_body.is_shape_disabled(body_shape_idx)) {
			continue;
		}

		RapierShape2D *body_shape = p_body.get_shape(body_shape_idx);
		rapier2d::Handle body_shape_handle = body_shape->get_rapier_shape();
		Transform2D const& body_shape_transform = p_transform * p_body.get_shape_transform(body_shape_idx);
		Vector2 body_shape_pos = body_shape_transform.get_origin();
		real_t body_shape_rot = body_shape_transform.get_rotation();

		bool stuck = false;
		real_t best_safe = 1;
		real_t best_unsafe = 1;

		// Check collision along the whole motion (optimization)
		/*rapier2d::ShapeCastResult shape_cast_result;
		int32_t shape_cast_result_count = 0;
		p_space.rapier_shape_cast(body_shape_handle, body_shape_transform, p_motion, p_body.get_collision_mask(), true, false, &shape_cast_result, 1, &shape_cast_result_count);
		if (shape_cast_result_count == 0) {
			// Didn't collide with anything, no need for further testing
			continue;
		} else if (shape_cast_result.toi < CMP_EPSILON) {
			// Collided on initial position, considered stuck
			p_closest_safe = 0.0;
			p_closest_unsafe = 0.0;
			p_best_body_shape = body_shape_idx;
			break;
		}*/

		for (int result_idx = 0; result_idx < result_count; ++result_idx) {
			rapier2d::PointHitInfo& result = results[result_idx];

			ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
			//ERR_CONTINUE(!rapier2d::is_user_data_valid(shape_cast_result.user_data));
			uint32_t shape_index = 0;
			RapierCollisionObject2D* shape_col_object = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
			//RapierCollisionObject2D* shape_col_object = RapierCollisionObject2D::get_collider_user_data(shape_cast_result.user_data, shape_index);
			ERR_CONTINUE(!shape_col_object);

			ERR_CONTINUE(shape_col_object->get_type() != RapierCollisionObject2D::TYPE_BODY);
			RapierBody2D* collision_body = static_cast<RapierBody2D*>(shape_col_object);

			RapierShape2D *col_shape = collision_body->get_shape(shape_index);
			rapier2d::Handle col_shape_handle = col_shape->get_rapier_shape();
			Transform2D const& col_shape_transform = collision_body->get_transform() * collision_body->get_shape_transform(shape_index);
			Vector2 col_shape_pos = col_shape_transform.get_origin();
			rapier2d::Vector rapier_col_shape_pos {col_shape_pos.x, col_shape_pos.y};
			real_t rapier_col_shape_rot = col_shape_transform.get_rotation();

			//just do kinematic solving
			//real_t low = MAX(shape_cast_result.toi - 0.01, 0.0);
			//real_t hi = MIN(shape_cast_result.toi + 0.01, 1.0);
			real_t low = 0.0f;
			real_t hi = 1.0f;
			real_t fraction_coeff = 0.5f;

			// TODO: Check the formula for 2D and replace 8
			// Figure out the number of steps we need in our binary search in order to achieve millimeter
			// precision, within reason. Derived from `2^-step_count * motion_length = 0.001`.
			const int32_t step_count = CLAMP(int32_t(logf(1000.0f * p_motion.length()) / Math_LN2), 4, 16);
			const int32_t step_count2 = CLAMP(int32_t(logf(100.0f * p_motion.length()) / Math_LN2), 4, 16);

			for (int k = 0; k < 4; k++) { //steps should be customizable..
				real_t fraction = low + (hi - low) * fraction_coeff;

				rapier2d::Vector rapier_body_shape_step_pos {body_shape_pos.x + p_motion.x * fraction, body_shape_pos.y + p_motion.y * fraction};
				rapier2d::ContactResult step_contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_handle, &rapier_body_shape_step_pos, body_shape_rot, col_shape_handle, &rapier_col_shape_pos, rapier_col_shape_rot, 0);
				
				if (step_contact.collided) {
					hi = fraction;
					if ((k == 0) || (low > 0.0)) { // Did it not collide before?
						// When alternating or first iteration, use dichotomy.
						fraction_coeff = 0.5;
					} else {
						// When colliding again, converge faster towards low fraction
						// for more accurate results with long motions that collide near the start.
						fraction_coeff = 0.25;
					}
				} else {
					low = fraction;
					if ((k == 0) || (hi < 1.0)) { // Did it collide before?
						// When alternating or first iteration, use dichotomy.
						fraction_coeff = 0.5;
					} else {
						// When not colliding again, converge faster towards high fraction
						// for more accurate results with long motions that collide near the end.
						fraction_coeff = 0.75;
					}
				}
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

bool RapierBodyUtils2D::body_motion_collide(
        const RapierSpace2D& p_space,
        const RapierBody2D& p_body,
        const Transform2D& p_transform,
		const Rect2& p_body_aabb,
		int p_best_body_shape,
        real_t p_margin,
        PhysicsServer2DExtensionMotionResult* p_result
) {
	int shape_count = p_body.get_shape_count();
	ERR_FAIL_COND_V(shape_count < 1, false);

	// Create compound shape for the body if needed
	/*rapier2d::Handle body_shape_handle;
	if (shape_count > 1) {
		rapier2d::ShapeInfo* shapes = (rapier2d::ShapeInfo*)alloca((shape_count) * sizeof(rapier2d::ShapeInfo));
		for (int i = 0; i < shape_count; ++i) {
			if (p_body.is_shape_disabled(i)) {
				continue;
			}
			rapier2d::ShapeInfo& shape_info = shapes[i];
			shape_info.handle = p_body.get_shape(i)->get_rapier_shape();
			Transform2D const& shape_transform = p_body.get_shape_transform(i);
			Vector2 const& shape_pos = shape_transform.get_origin();
			shape_info.position.x = shape_pos.x;
			shape_info.position.y = shape_pos.y;
			shape_info.rotation = shape_transform.get_rotation();
		}

		body_shape_handle = rapier2d::shape_create_compound(shapes, shape_count);
	} else {
		body_shape_handle = p_body.get_shape(0)->get_rapier_shape();
	}*/

	real_t margin = MAX(p_margin, TEST_MOTION_MARGIN_MIN_VALUE);
	real_t min_contact_depth = margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;

	Rect2 margin_body_aabb = p_body_aabb.grow(margin);

	rapier2d::PointHitInfo results[32];
	int result_count = p_space.rapier_intersect_aabb(margin_body_aabb, p_body.get_collision_mask(), true, false, results, 32, &result_count, p_body.get_rid());
		
	// Optimization
	if (result_count == 0) {
		return false;
	}

	real_t min_distance = INFINITY;
	RapierBody2D* best_collision_body = nullptr;
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
		rapier2d::Handle body_shape_handle = body_shape->get_rapier_shape();
		Transform2D const& body_shape_transform = p_transform * p_body.get_shape_transform(body_shape_idx);
		Vector2 body_shape_pos = body_shape_transform.get_origin();
		rapier2d::Vector rapier_body_shape_pos {body_shape_pos.x, body_shape_pos.y};
		real_t rapier_body_shape_rot = body_shape_transform.get_rotation();

		for (int result_idx = 0; result_idx < result_count; ++result_idx) {
			rapier2d::PointHitInfo& result = results[result_idx];

			ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
			uint32_t shape_index = 0;
			RapierCollisionObject2D* shape_col_object = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
			ERR_CONTINUE(!shape_col_object);

			ERR_CONTINUE(shape_col_object->get_type() != RapierCollisionObject2D::TYPE_BODY);
			RapierBody2D* collision_body = static_cast<RapierBody2D*>(shape_col_object);

			RapierShape2D *col_shape = collision_body->get_shape(shape_index);
			rapier2d::Handle col_shape_handle = col_shape->get_rapier_shape();
			Transform2D const& col_shape_transform = collision_body->get_transform() * collision_body->get_shape_transform(shape_index);
			Vector2 col_shape_pos = col_shape_transform.get_origin();
			rapier2d::Vector rapier_col_shape_pos {col_shape_pos.x, col_shape_pos.y};
			real_t rapier_col_shape_rot = col_shape_transform.get_rotation();

			rapier2d::ContactResult contact = rapier2d::shapes_contact(p_space.get_handle(), body_shape_handle, &rapier_body_shape_pos, rapier_body_shape_rot, col_shape_handle, &rapier_col_shape_pos, rapier_col_shape_rot, margin);
			if (!contact.collided) {
				continue;
			}

			// Distance is negative when there's penetration
			//real_t depth = Math::abs(contact.distance);
			//if (depth < min_contact_depth + CMP_EPSILON) {
			//	continue;
			//}

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
		if (p_result) {
			p_result->collider = best_collision_body->get_rid();
			p_result->collider_id = best_collision_body->get_instance_id();
			p_result->collider_shape = best_collision_shape_index;
			p_result->collision_local_shape = best_body_shape_index;
			// World position from the moving body to get the contact point
			p_result->collision_point = Vector2(best_contact.point1.x, best_contact.point1.y);
			// Normal from the collided object to get the contact normal
			p_result->collision_normal = Vector2(best_contact.normal2.x, best_contact.normal2.y);
			p_result->collision_depth = Math::abs(best_contact.distance);

			Vector2 local_position = p_result->collision_point - best_collision_body->get_transform().get_origin();
			p_result->collider_velocity = best_collision_body->get_velocity_at_local_point(local_position);
		}

		return true;
	}

	return false;
}
