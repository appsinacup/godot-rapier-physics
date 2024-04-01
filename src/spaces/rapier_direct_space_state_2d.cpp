#include "rapier_direct_space_state_2d.h"

int RapierDirectSpaceState2D::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) {
	ERR_FAIL_COND_V(space->locked, 0);
	ERR_FAIL_COND_V(!is_handle_valid(space->handle), 0);
	ERR_FAIL_COND_V(p_result_max < 0, 0);

	rapier2d::Vector rapier_pos = { position.x, position.y };

	rapier2d::PointHitInfo *hit_info_array = (rapier2d::PointHitInfo *)alloca(p_result_max * sizeof(rapier2d::PointHitInfo));

	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();
	query_excluded_info.query_canvas_instance_id = canvas_instance_id;
	query_excluded_info.query_collision_layer_mask = collision_mask;

	uint32_t result_count = rapier2d::intersect_point(space->handle, &rapier_pos, collide_with_bodies, collide_with_areas, hit_info_array, p_result_max, RapierSpace2D::_is_handle_excluded_callback, &query_excluded_info);
	ERR_FAIL_COND_V(result_count > (uint32_t)p_result_max, 0);

	for (uint32_t i = 0; i < result_count; i++) {
		rapier2d::PointHitInfo &hit_info = hit_info_array[i];
		PhysicsServer2DExtensionShapeResult &result = r_results[i];
		ERR_CONTINUE(!rapier2d::is_user_data_valid(hit_info.user_data));

		uint32_t shape_index = 0;
		RapierCollisionObject2D *collision_object_2d = RapierCollisionObject2D::get_collider_user_data(hit_info.user_data, shape_index);
		ERR_CONTINUE(collision_object_2d == nullptr);

		result.shape = shape_index;
		result.collider_id = collision_object_2d->get_instance_id();
		result.rid = collision_object_2d->get_rid();

		if (result.collider_id.is_valid()) {
			result.collider = RapierSpace2D::_get_object_instance_hack(result.collider_id);
		}
	}

	return result_count;
}

bool RapierDirectSpaceState2D::_intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *r_result) {
	ERR_FAIL_COND_V(space->locked, false);
	ERR_FAIL_COND_V(!is_handle_valid(space->handle), false);

	// Raycast Info
	Vector2 begin, end, dir;
	begin = from;
	end = (to - from);
	dir = end.normalized();
	real_t length = end.length();

	rapier2d::Vector rapier_from = { begin.x, begin.y };
	rapier2d::Vector rapier_dir = { dir.x, dir.y };

	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;

	rapier2d::RayHitInfo hit_info;
	bool collide = rapier2d::intersect_ray(space->handle,
			&rapier_from,
			&rapier_dir,
			length,
			collide_with_bodies,
			collide_with_areas,
			hit_from_inside,
			&hit_info,
			RapierSpace2D::_is_handle_excluded_callback,
			&query_excluded_info);

	if (collide) {
		r_result->position = Vector2(hit_info.pixel_position.x, hit_info.pixel_position.y);
		r_result->normal = Vector2(hit_info.normal.x, hit_info.normal.y);

		ERR_FAIL_COND_V(!rapier2d::is_user_data_valid(hit_info.user_data), false);
		uint32_t shape_index = 0;
		RapierCollisionObject2D *collision_object_2d = RapierCollisionObject2D::get_collider_user_data(hit_info.user_data, shape_index);
		ERR_FAIL_NULL_V(collision_object_2d, false);

		r_result->shape = shape_index;
		r_result->collider_id = collision_object_2d->get_instance_id();
		r_result->rid = collision_object_2d->get_rid();

		if (r_result->collider_id.is_valid()) {
			r_result->collider = RapierSpace2D::_get_object_instance_hack(r_result->collider_id);
		}

		return true;
	}

	return false;
}

bool RapierDirectSpaceState2D::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *p_closest_safe, float *p_closest_unsafe) {
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion = { motion.x, motion.y };
	rapier2d::ShapeInfo shape_info = rapier2d::shape_info_from_body_shape(shape_handle, transform);

	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;
	real_t hit = rapier2d::shape_casting(space->handle, &rapier_motion, shape_info, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback, &query_excluded_info, false).toi;
	*p_closest_safe = hit;
	*p_closest_unsafe = hit;
	return true;
}

bool RapierDirectSpaceState2D::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion{ motion.x, motion.y };

	Vector2 *results_out = static_cast<Vector2 *>(results);
	rapier2d::ShapeInfo shape_info = rapier2d::shape_info_from_body_shape(shape_handle, transform);
	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;
	query_excluded_info.query_exclude = (rapier2d::Handle *)alloca((max_results) * sizeof(rapier2d::Handle));
	query_excluded_info.query_exclude_size = 0;

	int cpt = 0;
	int array_idx = 0;
	do {
		rapier2d::ShapeCastResult result = rapier2d::shape_casting(space->handle, &rapier_motion, shape_info, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback, &query_excluded_info, true);
		if (!result.collided) {
			break;
		}
		(*result_count)++;
		query_excluded_info.query_exclude[query_excluded_info.query_exclude_size++] = result.collider;

		results_out[array_idx++] = Vector2(result.pixel_witness1.x, result.pixel_witness1.y);
		results_out[array_idx++] = Vector2(result.pixel_witness2.x, result.pixel_witness2.y);

		cpt++;
	} while (cpt < max_results);

	return array_idx > 0;
}

int RapierDirectSpaceState2D::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) {
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion{ motion.x, motion.y };
	rapier2d::ShapeInfo shape_info = rapier2d::shape_info_from_body_shape(shape_handle, transform);

	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;
	query_excluded_info.query_exclude = (rapier2d::Handle *)alloca((p_result_max) * sizeof(rapier2d::Handle));
	query_excluded_info.query_exclude_size = 0;
	int cpt = 0;
	do {
		rapier2d::ShapeCastResult result = rapier2d::shape_casting(space->handle, &rapier_motion, shape_info, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback, &query_excluded_info, true);
		if (!result.collided) {
			break;
		}
		query_excluded_info.query_exclude[query_excluded_info.query_exclude_size++] = result.collider;
		ERR_CONTINUE_MSG(!rapier2d::is_user_data_valid(result.user_data), "Invalid user data");
		uint32_t shape_index = 0;
		RapierCollisionObject2D *collision_object_2d = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
		ERR_CONTINUE_MSG(!collision_object_2d, "Invalid collision object");
		PhysicsServer2DExtensionShapeResult &result_out = r_results[cpt];
		result_out.shape = shape_index;
		result_out.rid = collision_object_2d->get_rid();
		result_out.collider_id = collision_object_2d->get_instance_id();
		if (result_out.collider_id.is_valid()) {
			result_out.collider = RapierSpace2D::_get_object_instance_hack(result_out.collider_id);
		}
		cpt++;

	} while (cpt < p_result_max);

	return cpt;
}

bool RapierDirectSpaceState2D::_rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *r_info) {
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion{ motion.x, motion.y };
	rapier2d::ShapeInfo shape_info = rapier2d::shape_info_from_body_shape(shape_handle, transform);
	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;

	rapier2d::ShapeCastResult result = rapier2d::shape_casting(space->handle, &rapier_motion, shape_info, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback, &query_excluded_info, true);
	if (!result.collided) {
		return false;
	}
	uint32_t shape_index = 0;
	RapierCollisionObject2D *collision_object_2d = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
	ERR_FAIL_COND_V(!collision_object_2d, false);
	r_info->collider_id = collision_object_2d->get_instance_id();
	if (collision_object_2d->get_type() == RapierCollisionObject2D::Type::TYPE_BODY) {
		const RapierBody2D *body = static_cast<const RapierBody2D *>(collision_object_2d);
		Vector2 rel_vec = r_info->point - (body->get_transform().get_origin() + body->get_center_of_mass());
		r_info->linear_velocity = Vector2(-body->get_angular_velocity() * rel_vec.y, body->get_angular_velocity() * rel_vec.x) + body->get_linear_velocity();

	} else {
		r_info->linear_velocity = Vector2();
	}
	r_info->normal = Vector2(result.normal1.x, result.normal1.y);
	r_info->rid = collision_object_2d->get_rid();
	r_info->shape = shape_index;
	return true;
}
