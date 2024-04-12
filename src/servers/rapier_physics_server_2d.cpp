#include "rapier_physics_server_2d.h"

#include "../bodies/rapier_direct_body_state_2d.h"

#include "../spaces/rapier_direct_space_state_2d.h"

#include "../shapes/rapier_capsule_shape_2d.h"
#include "../shapes/rapier_circle_shape_2d.h"
#include "../shapes/rapier_concave_polygon_shape_2d.h"
#include "../shapes/rapier_convex_polygon_shape_2d.h"
#include "../shapes/rapier_rectangle_shape_2d.h"
#include "../shapes/rapier_segment_shape_2d.h"
#include "../shapes/rapier_separation_ray_shape_2d.h"
#include "../shapes/rapier_world_boundary_shape_2d.h"

#include "../joints/rapier_damped_spring_joint_2d.h"
#include "../joints/rapier_groove_joint_2d.h"
#include "../joints/rapier_pin_joint_2d.h"

#define FLUSH_QUERY_CHECK(m_object) \
	ERR_FAIL_COND_MSG(m_object->get_space() && flushing_queries, "Can't change this state while flushing queries. Use call_deferred() or set_deferred() to change monitoring state instead.");

RID RapierPhysicsServer2D::_shape_create(ShapeType p_shape) {
	RapierShape2D *shape = nullptr;
	switch (p_shape) {
		case SHAPE_WORLD_BOUNDARY: {
			shape = memnew(RapierWorldBoundaryShape2D);
		} break;
		case SHAPE_SEPARATION_RAY: {
			shape = memnew(RapierSeparationRayShape2D);
		} break;
		case SHAPE_SEGMENT: {
			shape = memnew(RapierSegmentShape2D);
		} break;
		case SHAPE_CIRCLE: {
			shape = memnew(RapierCircleShape2D);
		} break;
		case SHAPE_RECTANGLE: {
			shape = memnew(RapierRectangleShape2D);
		} break;
		case SHAPE_CAPSULE: {
			shape = memnew(RapierCapsuleShape2D);
		} break;
		case SHAPE_CONVEX_POLYGON: {
			shape = memnew(RapierConvexPolygonShape2D);
		} break;
		case SHAPE_CONCAVE_POLYGON: {
			shape = memnew(RapierConcavePolygonShape2D);
		} break;
		case SHAPE_CUSTOM: {
			ERR_FAIL_V_MSG(RID(), "Unsupported shape");

		} break;
		default:
			ERR_FAIL_V_MSG(RID(), "Unsupported shape");
	}

	RID id = shape_owner.make_rid(shape);
	shape->set_rid(id);

	return id;
}

RID RapierPhysicsServer2D::_world_boundary_shape_create() {
	return _shape_create(SHAPE_WORLD_BOUNDARY);
}

RID RapierPhysicsServer2D::_separation_ray_shape_create() {
	return _shape_create(SHAPE_SEPARATION_RAY);
}

RID RapierPhysicsServer2D::_segment_shape_create() {
	return _shape_create(SHAPE_SEGMENT);
}

RID RapierPhysicsServer2D::_circle_shape_create() {
	return _shape_create(SHAPE_CIRCLE);
}

RID RapierPhysicsServer2D::_rectangle_shape_create() {
	return _shape_create(SHAPE_RECTANGLE);
}

RID RapierPhysicsServer2D::_capsule_shape_create() {
	return _shape_create(SHAPE_CAPSULE);
}

RID RapierPhysicsServer2D::_convex_polygon_shape_create() {
	return _shape_create(SHAPE_CONVEX_POLYGON);
}

RID RapierPhysicsServer2D::_concave_polygon_shape_create() {
	return _shape_create(SHAPE_CONCAVE_POLYGON);
}

void RapierPhysicsServer2D::_shape_set_data(const RID &p_shape, const Variant &p_data) {
	RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	shape->set_data(p_data);
};

void RapierPhysicsServer2D::_shape_set_custom_solver_bias(const RID &p_shape, double p_bias) {
	if (p_bias != 0.0) {
		WARN_PRINT_ONCE("Shape custom solver bias is not supported with Rapier");
	}
}

PhysicsServer2D::ShapeType RapierPhysicsServer2D::_shape_get_type(const RID &p_shape) const {
	const RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND_V(!shape, SHAPE_CUSTOM);
	return shape->get_type();
};

Variant RapierPhysicsServer2D::_shape_get_data(const RID &p_shape) const {
	const RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND_V(!shape, Variant());
	ERR_FAIL_COND_V(!shape->is_configured(), Variant());
	return shape->get_data();
};

double RapierPhysicsServer2D::_shape_get_custom_solver_bias(const RID &p_shape) const {
	return 0.0;
}

bool RapierPhysicsServer2D::_shape_collide(const RID &p_shape_A, const Transform2D &p_xform_A, const Vector2 &p_motion_A, const RID &p_shape_B, const Transform2D &p_xform_B, const Vector2 &p_motion_B, void *r_results, int32_t p_result_max, int32_t *p_result_count) {
	RapierShape2D *shape_A = shape_owner.get_or_null(p_shape_A);
	ERR_FAIL_COND_V(!shape_A, false);
	RapierShape2D *shape_B = shape_owner.get_or_null(p_shape_B);
	ERR_FAIL_COND_V(!shape_B, false);

	rapier2d::Handle shape_A_handle = shape_A->get_rapier_shape();
	rapier2d::Handle shape_B_handle = shape_B->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_A_handle), false);
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_B_handle), false);
	rapier2d::ShapeInfo shape_A_info = rapier2d::shape_info_from_body_shape(shape_A_handle, p_xform_A);
	rapier2d::ShapeInfo shape_B_info = rapier2d::shape_info_from_body_shape(shape_B_handle, p_xform_B);
	rapier2d::Vector rapier_A_motion{ p_motion_A.x, p_motion_A.y };
	rapier2d::Vector rapier_B_motion{ p_motion_B.x, p_motion_B.y };

	Vector2 *results_out = static_cast<Vector2 *>(r_results);

	rapier2d::QueryExcludedInfo query_excluded_info = rapier2d::default_query_excluded_info();

	int array_idx = 0;
	rapier2d::ShapeCastResult result = rapier2d::shape_collide(&rapier_A_motion, shape_A_info, &rapier_A_motion, shape_A_info);
	if (!result.collided) {
		return false;
	}
	(*p_result_count)++;

	results_out[array_idx++] = Vector2(result.pixel_witness1.x, result.pixel_witness1.y);
	results_out[array_idx++] = Vector2(result.pixel_witness2.x, result.pixel_witness2.y);

	return array_idx > 0;
}

RID RapierPhysicsServer2D::_space_create() {
	RapierSpace2D *space = memnew(RapierSpace2D);
	RID id = space_owner.make_rid(space);
	space->set_rid(id);
	// RID area_id = area_create();
	// RapierArea2D *area = area_owner.get_or_null(area_id);
	// ERR_FAIL_COND_V(!area, RID());
	// space->set_default_area(area);
	// area->set_space(space);
	// area->set_priority(-1);

	return id;
};

void RapierPhysicsServer2D::_space_set_active(const RID &p_space, bool p_active) {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND(!space);
	if (p_active) {
		active_spaces.insert(rapier2d::handle_hash(space->get_handle()), space);
	} else {
		active_spaces.erase(rapier2d::handle_hash(space->get_handle()));
	}
}

bool RapierPhysicsServer2D::_space_is_active(const RID &p_space) const {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, false);

	return active_spaces.has(rapier2d::handle_hash(space->get_handle()));
}

void RapierPhysicsServer2D::_space_set_param(const RID &p_space, SpaceParameter p_param, double p_value) {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND(!space);

	space->set_param(p_param, p_value);
}

double RapierPhysicsServer2D::_space_get_param(const RID &p_space, SpaceParameter p_param) const {
	const RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, 0);
	return space->get_param(p_param);
}

void RapierPhysicsServer2D::_space_set_debug_contacts(const RID &p_space, int p_max_contacts) {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND(!space);
	space->set_debug_contacts(p_max_contacts);
}

PackedVector2Array RapierPhysicsServer2D::_space_get_contacts(const RID &p_space) const {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, PackedVector2Array());
	return space->get_debug_contacts();
}

int RapierPhysicsServer2D::_space_get_contact_count(const RID &p_space) const {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, 0);
	return space->get_debug_contact_count();
}

PhysicsDirectSpaceState2D *RapierPhysicsServer2D::_space_get_direct_state(const RID &p_space) {
	RapierSpace2D *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, nullptr);
	ERR_FAIL_COND_V_MSG((using_threads && !doing_sync) || space->is_locked(), nullptr, "Space state is inaccessible right now, wait for iteration or physics process notification.");

	return space->get_direct_state();
}

RID RapierPhysicsServer2D::_area_create() {
	RapierArea2D *area = memnew(RapierArea2D);

	RID rid = area_owner.make_rid(area);
	area->set_rid(rid);

	return rid;
}

void RapierPhysicsServer2D::_area_set_space(const RID &p_area, const RID &p_space) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	RapierSpace2D *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_COND(!space);
	}

	if (area->get_space() == space) {
		return; //pointless
	}

	area->set_space(space);
}

RID RapierPhysicsServer2D::_area_get_space(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, RID());

	RapierSpace2D *space = area->get_space();
	if (!space) {
		return RID();
	}
	return space->get_rid();
}

void RapierPhysicsServer2D::_area_add_shape(const RID &p_area, const RID &p_shape, const Transform2D &p_transform, bool p_disabled) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);

	area->add_shape(shape, p_transform, p_disabled);
}

void RapierPhysicsServer2D::_area_set_shape(const RID &p_area, int p_shape_idx, const RID &p_shape) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	ERR_FAIL_COND(!shape->is_configured());

	area->set_shape(p_shape_idx, shape);
}

void RapierPhysicsServer2D::_area_set_shape_transform(const RID &p_area, int p_shape_idx, const Transform2D &p_transform) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_shape_transform(p_shape_idx, p_transform);
}

void RapierPhysicsServer2D::_area_set_shape_disabled(const RID &p_area, int p_shape, bool p_disabled) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	ERR_FAIL_INDEX(p_shape, area->get_shape_count());
	FLUSH_QUERY_CHECK(area);

	area->set_shape_disabled(p_shape, p_disabled);
}

int RapierPhysicsServer2D::_area_get_shape_count(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, -1);

	return area->get_shape_count();
}

RID RapierPhysicsServer2D::_area_get_shape(const RID &p_area, int p_shape_idx) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, RID());

	RapierShape2D *shape = area->get_shape(p_shape_idx);
	ERR_FAIL_COND_V(!shape, RID());

	return shape->get_rid();
}

Transform2D RapierPhysicsServer2D::_area_get_shape_transform(const RID &p_area, int p_shape_idx) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, Transform2D());

	return area->get_shape_transform(p_shape_idx);
}

void RapierPhysicsServer2D::_area_remove_shape(const RID &p_area, int p_shape_idx) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->remove_shape(p_shape_idx);
}

void RapierPhysicsServer2D::_area_clear_shapes(const RID &p_area) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	while (area->get_shape_count()) {
		area->remove_shape(0);
	}
}

void RapierPhysicsServer2D::_area_attach_object_instance_id(const RID &p_area, uint64_t p_id) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_instance_id(ObjectID(p_id));
}

uint64_t RapierPhysicsServer2D::_area_get_object_instance_id(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, 0);
	return area->get_instance_id();
}

void RapierPhysicsServer2D::_area_attach_canvas_instance_id(const RID &p_area, uint64_t p_id) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_canvas_instance_id(ObjectID(p_id));
}

uint64_t RapierPhysicsServer2D::_area_get_canvas_instance_id(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, 0);
	return area->get_canvas_instance_id();
}

void RapierPhysicsServer2D::_area_set_param(const RID &p_area, AreaParameter p_param, const Variant &p_value) {
	if (space_owner.owns(p_area)) {
		RapierSpace2D *space = space_owner.get_or_null(p_area);
		ERR_FAIL_COND(!space);
		space->set_default_area_param(p_param, p_value);
	} else {
		RapierArea2D *area = area_owner.get_or_null(p_area);
		ERR_FAIL_COND(!area);
		area->set_param(p_param, p_value);
	}
};

void RapierPhysicsServer2D::_area_set_transform(const RID &p_area, const Transform2D &p_transform) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_transform(p_transform);
};

Variant RapierPhysicsServer2D::_area_get_param(const RID &p_area, AreaParameter p_param) const {
	if (space_owner.owns(p_area)) {
		RapierSpace2D *space = space_owner.get_or_null(p_area);
		ERR_FAIL_COND_V(!space, Variant());
		return space->get_default_area_param(p_param);
	}

	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, Variant());
	return area->get_param(p_param);
};

Transform2D RapierPhysicsServer2D::_area_get_transform(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, Transform2D());

	return area->get_transform();
};

void RapierPhysicsServer2D::_area_set_pickable(const RID &p_area, bool p_pickable) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_pickable(p_pickable);
}

void RapierPhysicsServer2D::_area_set_monitorable(const RID &p_area, bool p_monitorable) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	FLUSH_QUERY_CHECK(area);

	area->set_monitorable(p_monitorable);
}

void RapierPhysicsServer2D::_area_set_collision_mask(const RID &p_area, uint32_t p_mask) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_collision_mask(p_mask);
}

uint32_t RapierPhysicsServer2D::_area_get_collision_mask(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, 0);

	return area->get_collision_mask();
}

void RapierPhysicsServer2D::_area_set_collision_layer(const RID &p_area, uint32_t p_layer) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_collision_layer(p_layer);
}

uint32_t RapierPhysicsServer2D::_area_get_collision_layer(const RID &p_area) const {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, 0);

	return area->get_collision_layer();
}

void RapierPhysicsServer2D::_area_set_monitor_callback(const RID &p_area, const Callable &p_callback) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_monitor_callback(p_callback.is_valid() ? p_callback : Callable());
}

void RapierPhysicsServer2D::_area_set_area_monitor_callback(const RID &p_area, const Callable &p_callback) {
	RapierArea2D *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_area_monitor_callback(p_callback.is_valid() ? p_callback : Callable());
}

/* BODY API */

RID RapierPhysicsServer2D::_body_create() {
	RapierBody2D *body = memnew(RapierBody2D);
	RID rid = body_owner.make_rid(body);
	body->set_rid(rid);
	return rid;
}

/**
 * Add the body in the physics space (world).
 */
void RapierPhysicsServer2D::_body_set_space(const RID &p_body, const RID &p_space) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	RapierSpace2D *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_COND(!space);
	}

	body->set_space(space);
};

RID RapierPhysicsServer2D::_body_get_space(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, RID());

	RapierSpace2D *space = body->get_space();
	if (!space) {
		return RID();
	}

	return space->get_rid();
};

void RapierPhysicsServer2D::_body_set_mode(const RID &p_body, BodyMode p_mode) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	FLUSH_QUERY_CHECK(body);

	body->set_mode(p_mode);
};

PhysicsServer2D::BodyMode RapierPhysicsServer2D::_body_get_mode(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, BODY_MODE_STATIC);

	return body->get_mode();
};

void RapierPhysicsServer2D::_body_add_shape(const RID &p_body, const RID &p_shape, const Transform2D &p_transform, bool p_disabled) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);

	body->add_shape(shape, p_transform, p_disabled);
}

void RapierPhysicsServer2D::_body_set_shape(const RID &p_body, int p_shape_idx, const RID &p_shape) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	RapierShape2D *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	ERR_FAIL_COND(!shape->is_configured());

	body->set_shape(p_shape_idx, shape);
}

void RapierPhysicsServer2D::_body_set_shape_transform(const RID &p_body, int p_shape_idx, const Transform2D &p_transform) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_transform(p_shape_idx, p_transform);
}

int RapierPhysicsServer2D::_body_get_shape_count(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, -1);

	return body->get_shape_count();
}

RID RapierPhysicsServer2D::_body_get_shape(const RID &p_body, int p_shape_idx) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, RID());

	RapierShape2D *shape = body->get_shape(p_shape_idx);
	ERR_FAIL_COND_V(!shape, RID());

	return shape->get_rid();
}

Transform2D RapierPhysicsServer2D::_body_get_shape_transform(const RID &p_body, int p_shape_idx) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Transform2D());

	return body->get_shape_transform(p_shape_idx);
}

void RapierPhysicsServer2D::_body_remove_shape(const RID &p_body, int p_shape_idx) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->remove_shape(p_shape_idx);
}

void RapierPhysicsServer2D::_body_clear_shapes(const RID &p_body) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	while (body->get_shape_count()) {
		body->remove_shape(0);
	}
}

void RapierPhysicsServer2D::_body_set_shape_disabled(const RID &p_body, int p_shape_idx, bool p_disabled) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	ERR_FAIL_INDEX(p_shape_idx, body->get_shape_count());
	FLUSH_QUERY_CHECK(body);

	body->set_shape_disabled(p_shape_idx, p_disabled);
}

void RapierPhysicsServer2D::_body_set_shape_as_one_way_collision(const RID &p_body, int p_shape_idx, bool p_enable, double p_margin) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	ERR_FAIL_INDEX(p_shape_idx, body->get_shape_count());
	FLUSH_QUERY_CHECK(body);

	body->set_shape_as_one_way_collision(p_shape_idx, p_enable, p_margin);
}

void RapierPhysicsServer2D::_body_set_continuous_collision_detection_mode(const RID &p_body, CCDMode p_mode) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_continuous_collision_detection_mode(p_mode);
}

RapierPhysicsServer2D::CCDMode RapierPhysicsServer2D::_body_get_continuous_collision_detection_mode(const RID &p_body) const {
	const RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, CCD_MODE_DISABLED);

	return body->get_continuous_collision_detection_mode();
}

void RapierPhysicsServer2D::_body_attach_object_instance_id(const RID &p_body, uint64_t p_id) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_instance_id(ObjectID(p_id));
}

uint64_t RapierPhysicsServer2D::_body_get_object_instance_id(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_instance_id();
}

void RapierPhysicsServer2D::_body_attach_canvas_instance_id(const RID &p_body, uint64_t p_id) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_canvas_instance_id(ObjectID(p_id));
}

uint64_t RapierPhysicsServer2D::_body_get_canvas_instance_id(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_canvas_instance_id();
}

void RapierPhysicsServer2D::_body_set_collision_layer(const RID &p_body, uint32_t p_layer) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_collision_layer(p_layer);
}

uint32_t RapierPhysicsServer2D::_body_get_collision_layer(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_layer();
}

void RapierPhysicsServer2D::_body_set_collision_mask(const RID &p_body, uint32_t p_mask) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_collision_mask(p_mask);
}

uint32_t RapierPhysicsServer2D::_body_get_collision_mask(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_mask();
}

void RapierPhysicsServer2D::_body_set_collision_priority(const RID &p_body, double p_priority) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_collision_priority(p_priority);
}

double RapierPhysicsServer2D::_body_get_collision_priority(const RID &p_body) const {
	const RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_collision_priority();
}

void RapierPhysicsServer2D::_body_set_param(const RID &p_body, BodyParameter p_param, const Variant &p_value) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_param(p_param, p_value);
}

Variant RapierPhysicsServer2D::_body_get_param(const RID &p_body, BodyParameter p_param) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_param(p_param);
}

void RapierPhysicsServer2D::_body_reset_mass_properties(const RID &p_body) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	return body->reset_mass_properties();
}

void RapierPhysicsServer2D::_body_set_state(const RID &p_body, BodyState p_state, const Variant &p_variant) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_state(p_state, p_variant);
}

Variant RapierPhysicsServer2D::_body_get_state(const RID &p_body, BodyState p_state) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Variant());

	return body->get_state(p_state);
}

void RapierPhysicsServer2D::_body_apply_central_impulse(const RID &p_body, const Vector2 &p_impulse) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->apply_central_impulse(p_impulse);
}

void RapierPhysicsServer2D::_body_apply_torque_impulse(const RID &p_body, double p_torque) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->apply_torque_impulse(p_torque);
}

void RapierPhysicsServer2D::_body_apply_impulse(const RID &p_body, const Vector2 &p_impulse, const Vector2 &p_position) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->apply_impulse(p_impulse, p_position);
}

void RapierPhysicsServer2D::_body_apply_central_force(const RID &p_body, const Vector2 &p_force) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->apply_central_force(p_force);
}

void RapierPhysicsServer2D::_body_apply_force(const RID &p_body, const Vector2 &p_force, const Vector2 &p_position) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->apply_force(p_force, p_position);
}

void RapierPhysicsServer2D::_body_apply_torque(const RID &p_body, double p_torque) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->apply_torque(p_torque);
}

void RapierPhysicsServer2D::_body_add_constant_central_force(const RID &p_body, const Vector2 &p_force) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->add_constant_central_force(p_force);
}

void RapierPhysicsServer2D::_body_add_constant_force(const RID &p_body, const Vector2 &p_force, const Vector2 &p_position) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->add_constant_force(p_force, p_position);
}

void RapierPhysicsServer2D::_body_add_constant_torque(const RID &p_body, double p_torque) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->add_constant_torque(p_torque);
}

void RapierPhysicsServer2D::_body_set_constant_force(const RID &p_body, const Vector2 &p_force) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_constant_force(p_force);
}

Vector2 RapierPhysicsServer2D::_body_get_constant_force(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Vector2());

	return body->get_constant_force();
}

void RapierPhysicsServer2D::_body_set_constant_torque(const RID &p_body, double p_torque) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_constant_torque(p_torque);
}

double RapierPhysicsServer2D::_body_get_constant_torque(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);

	return body->get_constant_torque();
}

void RapierPhysicsServer2D::_body_set_axis_velocity(const RID &p_body, const Vector2 &p_axis_velocity) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	Vector2 v = body->get_direct_state()->get_linear_velocity();
	Vector2 axis = p_axis_velocity.normalized();
	v -= axis * axis.dot(v);
	v += p_axis_velocity;
	body->get_direct_state()->set_linear_velocity(v);
	body->wakeup();
};

void RapierPhysicsServer2D::_body_add_collision_exception(const RID &p_body, const RID &p_body_b) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->add_exception(p_body_b);
	body->wakeup();
};

void RapierPhysicsServer2D::_body_remove_collision_exception(const RID &p_body, const RID &p_body_b) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->remove_exception(p_body_b);
	body->wakeup();
};

TypedArray<RID> RapierPhysicsServer2D::_body_get_collision_exceptions(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, TypedArray<RID>());

	size_t exception_count = body->get_exceptions().size();

	TypedArray<RID> array;
	array.resize(exception_count);

	for (int i = 0; i < exception_count; i++) {
		array[i] = body->get_exceptions()[i];
	}

	return array;
};

void RapierPhysicsServer2D::_body_set_contacts_reported_depth_threshold(const RID &p_body, double p_threshold) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
};

double RapierPhysicsServer2D::_body_get_contacts_reported_depth_threshold(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return 0;
};

void RapierPhysicsServer2D::_body_set_omit_force_integration(const RID &p_body, bool p_omit) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_omit_force_integration(p_omit);
};

bool RapierPhysicsServer2D::_body_is_omitting_force_integration(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, false);
	return body->get_omit_force_integration();
};

void RapierPhysicsServer2D::_body_set_max_contacts_reported(const RID &p_body, int p_contacts) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_max_contacts_reported(p_contacts);
}

int RapierPhysicsServer2D::_body_get_max_contacts_reported(const RID &p_body) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, -1);
	return body->get_max_contacts_reported();
}

void RapierPhysicsServer2D::_body_set_state_sync_callback(const RID &p_body, const Callable &p_callable) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_state_sync_callback(p_callable);
}

void RapierPhysicsServer2D::_body_set_force_integration_callback(const RID &p_body, const Callable &p_callable, const Variant &p_udata) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_force_integration_callback(p_callable, p_udata);
}

bool RapierPhysicsServer2D::_body_collide_shape(const RID &p_body, int32_t p_body_shape, const RID &p_shape, const Transform2D &p_shape_xform, const Vector2 &p_motion, void *r_results, int32_t p_result_max, int32_t *r_result_count) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, false);
	ERR_FAIL_INDEX_V(p_body_shape, body->get_shape_count(), false);

	return _shape_collide(body->get_shape(p_body_shape)->get_rid(), body->get_transform() * body->get_shape_transform(p_body_shape), Vector2(), p_shape, p_shape_xform, p_motion, r_results, p_result_max, r_result_count);
}

void RapierPhysicsServer2D::_body_set_pickable(const RID &p_body, bool p_pickable) {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_pickable(p_pickable);
}

bool RapierPhysicsServer2D::_body_test_motion(const RID &p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *r_result) const {
	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, false);
	ERR_FAIL_COND_V(!body->get_space(), false);
	ERR_FAIL_COND_V(body->get_space()->is_locked(), false);

	return body->get_space()->test_body_motion(body, p_from, p_motion, p_margin, p_collide_separation_ray, p_recovery_as_collision, r_result);
}

PhysicsDirectBodyState2D *RapierPhysicsServer2D::_body_get_direct_state(const RID &p_body) {
	ERR_FAIL_COND_V_MSG((using_threads && !doing_sync), nullptr, "Body state is inaccessible right now, wait for iteration or physics process notification.");

	if (!body_owner.owns(p_body)) {
		return nullptr;
	}

	RapierBody2D *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, nullptr);

	if (!body->get_space()) {
		return nullptr;
	}

	ERR_FAIL_COND_V_MSG(body->get_space()->is_locked(), nullptr, "Body state is inaccessible right now, wait for iteration or physics process notification.");

	return body->get_direct_state();
}

/* JOINT API */

RID RapierPhysicsServer2D::_joint_create() {
	RapierJoint2D *joint = memnew(RapierJoint2D);
	RID joint_rid = joint_owner.make_rid(joint);
	joint->set_rid(joint_rid);
	return joint_rid;
}

void RapierPhysicsServer2D::_joint_clear(const RID &p_joint) {
	RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	if (joint->get_type() != JOINT_TYPE_MAX) {
		RapierJoint2D *empty_joint = memnew(RapierJoint2D);
		empty_joint->copy_settings_from(joint);

		joint_owner.replace(p_joint, empty_joint);
		memdelete(joint);
	}
}

void RapierPhysicsServer2D::_joint_set_param(const RID &p_joint, JointParam p_param, double p_value) {
	RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);

	switch (p_param) {
		case JOINT_PARAM_BIAS:
			joint->set_bias(p_value);
			break;
		case JOINT_PARAM_MAX_BIAS:
			joint->set_max_bias(p_value);
			break;
		case JOINT_PARAM_MAX_FORCE:
			joint->set_max_force(p_value);
			break;
	}
}

double RapierPhysicsServer2D::_joint_get_param(const RID &p_joint, JointParam p_param) const {
	const RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, -1);

	switch (p_param) {
		case JOINT_PARAM_BIAS:
			return joint->get_bias();
			break;
		case JOINT_PARAM_MAX_BIAS:
			return joint->get_max_bias();
			break;
		case JOINT_PARAM_MAX_FORCE:
			return joint->get_max_force();
			break;
	}
	return 0;
}

void RapierPhysicsServer2D::_joint_disable_collisions_between_bodies(const RID &p_joint, const bool p_disable) {
	RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);

	joint->disable_collisions_between_bodies(p_disable);
}

bool RapierPhysicsServer2D::_joint_is_disabled_collisions_between_bodies(const RID &p_joint) const {
	const RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, true);

	return joint->is_disabled_collisions_between_bodies();
}

void RapierPhysicsServer2D::_joint_make_pin(const RID &p_joint, const Vector2 &p_pos, const RID &p_body_a, const RID &p_body_b) {
	RapierBody2D *A = body_owner.get_or_null(p_body_a);
	ERR_FAIL_COND(!A);
	RapierBody2D *B = nullptr;
	if (body_owner.owns(p_body_b)) {
		B = body_owner.get_or_null(p_body_b);
		ERR_FAIL_COND(!B);
	}

	RapierJoint2D *prev_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(prev_joint == nullptr);

	RapierJoint2D *joint = memnew(RapierPinJoint2D(p_pos, A, B));

	joint_owner.replace(p_joint, joint);
	joint->copy_settings_from(prev_joint);
	memdelete(prev_joint);
}

void RapierPhysicsServer2D::_joint_make_groove(const RID &p_joint, const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, const RID &p_body_a, const RID &p_body_b) {
	RapierBody2D *A = body_owner.get_or_null(p_body_a);
	ERR_FAIL_COND(!A);

	RapierBody2D *B = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(!B);

	RapierJoint2D *prev_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(prev_joint == nullptr);

	RapierJoint2D *joint = memnew(RapierGrooveJoint2D(p_a_groove1, p_a_groove2, p_b_anchor, A, B));

	joint_owner.replace(p_joint, joint);
	joint->copy_settings_from(prev_joint);
	memdelete(prev_joint);
}

void RapierPhysicsServer2D::_joint_make_damped_spring(const RID &p_joint, const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, const RID &p_body_a, const RID &p_body_b) {
	RapierBody2D *A = body_owner.get_or_null(p_body_a);
	ERR_FAIL_COND(!A);

	RapierBody2D *B = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(!B);

	RapierJoint2D *prev_joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(prev_joint == nullptr);

	RapierJoint2D *joint = memnew(RapierDampedSpringJoint2D(p_anchor_a, p_anchor_b, A, B));

	joint_owner.replace(p_joint, joint);
	joint->copy_settings_from(prev_joint);
	memdelete(prev_joint);
}

void RapierPhysicsServer2D::_pin_joint_set_flag(const RID &p_joint, PhysicsServer2D::PinJointFlag p_flag, bool p_enabled) {
	RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND(joint->get_type() != JOINT_TYPE_PIN);

	RapierPinJoint2D *pin_joint = static_cast<RapierPinJoint2D *>(joint);
	pin_joint->set_flag(p_flag, p_enabled);
}

bool RapierPhysicsServer2D::_pin_joint_get_flag(const RID &p_joint, PhysicsServer2D::PinJointFlag p_flag) const {
	RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_NULL_V(joint, 0);
	ERR_FAIL_COND_V(joint->get_type() != JOINT_TYPE_PIN, 0);

	RapierPinJoint2D *pin_joint = static_cast<RapierPinJoint2D *>(joint);
	return pin_joint->get_flag(p_flag);
}

void RapierPhysicsServer2D::_pin_joint_set_param(const RID &p_joint, PinJointParam p_param, double p_value) {
	RapierJoint2D *j = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!j);
	ERR_FAIL_COND(j->get_type() != JOINT_TYPE_PIN);

	RapierPinJoint2D *pin_joint = static_cast<RapierPinJoint2D *>(j);
	pin_joint->set_param(p_param, p_value);
}

double RapierPhysicsServer2D::_pin_joint_get_param(const RID &p_joint, PinJointParam p_param) const {
	RapierJoint2D *j = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!j, 0);
	ERR_FAIL_COND_V(j->get_type() != JOINT_TYPE_PIN, 0);

	RapierPinJoint2D *pin_joint = static_cast<RapierPinJoint2D *>(j);
	return pin_joint->get_param(p_param);
}

void RapierPhysicsServer2D::_damped_spring_joint_set_param(const RID &p_joint, DampedSpringParam p_param, double p_value) {
	RapierJoint2D *j = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!j);
	ERR_FAIL_COND(j->get_type() != JOINT_TYPE_DAMPED_SPRING);

	RapierDampedSpringJoint2D *dsj = static_cast<RapierDampedSpringJoint2D *>(j);
	dsj->set_param(p_param, p_value);
}

double RapierPhysicsServer2D::_damped_spring_joint_get_param(const RID &p_joint, DampedSpringParam p_param) const {
	RapierJoint2D *j = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!j, 0);
	ERR_FAIL_COND_V(j->get_type() != JOINT_TYPE_DAMPED_SPRING, 0);

	RapierDampedSpringJoint2D *dsj = static_cast<RapierDampedSpringJoint2D *>(j);
	return dsj->get_param(p_param);
}

PhysicsServer2D::JointType RapierPhysicsServer2D::_joint_get_type(const RID &p_joint) const {
	RapierJoint2D *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, JOINT_TYPE_PIN);

	return joint->get_type();
}

RID RapierPhysicsServer2D::fluid_create() {
	RapierFluid2D *fluid = memnew(RapierFluid2D);
	RID fluid_rid = fluid_owner.make_rid(fluid);
	fluid->set_rid(fluid_rid);
	return fluid_rid;
}


void RapierPhysicsServer2D::fluid_set_space(const RID &fluid_rid, const RID &space_rid) {
	RapierFluid2D *fluid = fluid_owner.get_or_null(fluid_rid);
	ERR_FAIL_COND(!fluid);

	RapierSpace2D *space = nullptr;
	if (space_rid.is_valid()) {
		space = space_owner.get_or_null(space_rid);
		ERR_FAIL_COND(!space);
	}

	fluid->set_space(space);
};

void RapierPhysicsServer2D::fluid_set_density(const RID &p_fluid, real_t density) {
	RapierFluid2D *fluid = fluid_owner.get_or_null(p_fluid);
	ERR_FAIL_COND(!fluid);

	fluid->set_density(density);
}

void RapierPhysicsServer2D::_free_rid(const RID &p_rid) {
	if (shape_owner.owns(p_rid)) {
		RapierShape2D *shape = shape_owner.get_or_null(p_rid);

		while (shape->get_owners().size()) {
			RapierShapeOwner2D *so = shape->get_owners().begin()->key;
			so->remove_shape(shape);
		}

		shape_owner.free(p_rid);
		memdelete(shape);
	} else if (body_owner.owns(p_rid)) {
		RapierBody2D *body = body_owner.get_or_null(p_rid);

		body->set_space(nullptr);

		while (body->get_shape_count()) {
			body->remove_shape(0);
		}

		body_owner.free(p_rid);
		memdelete(body);
	} else if (area_owner.owns(p_rid)) {
		RapierArea2D *area = area_owner.get_or_null(p_rid);

		area->set_space(nullptr);

		while (area->get_shape_count()) {
			area->remove_shape(0);
		}

		area_owner.free(p_rid);
		memdelete(area);
	} else if (space_owner.owns(p_rid)) {
		RapierSpace2D *space = space_owner.get_or_null(p_rid);

		// TODO: recheck later
		// while (space->get_objects().size()) {
		// 	RapierCollisionObject2D *co = static_cast<RapierCollisionObject2D *>(*space->get_objects().begin());
		// 	co->set_space(nullptr);
		// }

		active_spaces.erase(rapier2d::handle_hash(space->get_handle()));

		space_owner.free(p_rid);
		memdelete(space);
	} else if (joint_owner.owns(p_rid)) {
		RapierJoint2D *joint = joint_owner.get_or_null(p_rid);

		joint_owner.free(p_rid);
		memdelete(joint);
	} else if (fluid_owner.owns(p_rid)) {
		RapierFluid2D *fluid = fluid_owner.get_or_null(p_rid);

		fluid_owner.free(p_rid);
		memdelete(fluid);
	} else {
		ERR_FAIL_MSG("Invalid ID.");
	}
}

void RapierPhysicsServer2D::_set_active(bool p_active) {
	active = p_active;
}

void RapierPhysicsServer2D::_init() {
	doing_sync = false;
}

void RapierPhysicsServer2D::_step(double p_step) {
	if (!active) {
		return;
	}

	++frame;

	island_count = 0;
	active_objects = 0;
	collision_pairs = 0;
	for (auto const &iterator : active_spaces) {
		RapierSpace2D *space = iterator.value;
		space->step(p_step);
		island_count += space->get_island_count();
		active_objects += space->get_active_objects();
		collision_pairs += space->get_collision_pairs();
	}
}

void RapierPhysicsServer2D::_sync() {
	doing_sync = true;
}

/**
 * Synchronize the infos (like transforms) from the rapier to Godot nodes/script.
 */
void RapierPhysicsServer2D::_flush_queries() {
	if (!active) {
		return;
	}

	flushing_queries = true;

	// uint64_t time_beg = OS::get_singleton()->get_ticks_usec();

	for (auto const &iterator : active_spaces) {
		RapierSpace2D *space = iterator.value;
		space->call_queries();
	}

	flushing_queries = false;

	// if (EngineDebugger::is_profiling("servers")) {
	// 	uint64_t total_time[RapierSpace2D::ELAPSED_TIME_MAX];
	// 	static const char *time_name[RapierSpace2D::ELAPSED_TIME_MAX] = {
	// 		"integrate_forces",
	// 		"generate_islands",
	// 		"setup_constraints",
	// 		"solve_constraints",
	// 		"integrate_velocities"
	// 	};

	// 	for (int i = 0; i < RapierSpace2D::ELAPSED_TIME_MAX; i++) {
	// 		total_time[i] = 0;
	// 	}

	// 	for (RapierSpace2D *space : active_spaces) {
	// 		for (int i = 0; i < RapierSpace2D::ELAPSED_TIME_MAX; i++) {
	// 			// total_time[i] += space->get_elapsed_time(RapierSpace2D::ElapsedTime(i));
	// 		}
	// 	}

	// 	Array values;
	// 	values.resize(RapierSpace2D::ELAPSED_TIME_MAX * 2);
	// 	for (int i = 0; i < RapierSpace2D::ELAPSED_TIME_MAX; i++) {
	// 		values[i * 2 + 0] = time_name[i];
	// 		values[i * 2 + 1] = USEC_TO_SEC(total_time[i]);
	// 	}
	// 	values.push_back("flush_queries");
	// 	values.push_back(USEC_TO_SEC(OS::get_singleton()->get_ticks_usec() - time_beg));

	// 	values.push_front("physics_2d");
	// 	EngineDebugger::profiler_add_frame_data("servers", values);
	// }
}

void RapierPhysicsServer2D::_end_sync() {
	doing_sync = false;
}

void RapierPhysicsServer2D::_finish() {
}

int RapierPhysicsServer2D::_get_process_info(ProcessInfo p_info) {
	switch (p_info) {
		case INFO_ACTIVE_OBJECTS: {
			return active_objects;
		} break;
		case INFO_COLLISION_PAIRS: {
			return collision_pairs;
		} break;
		case INFO_ISLAND_COUNT: {
			return island_count;
		} break;
	}

	return 0;
}

RapierPhysicsServer2D *RapierPhysicsServer2D::singleton = nullptr;

RapierPhysicsServer2D::RapierPhysicsServer2D(bool p_using_threads) {
	singleton = this;

	using_threads = p_using_threads;
}

void RapierPhysicsServer2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("fluid_create"), &RapierPhysicsServer2D::fluid_create);
}
