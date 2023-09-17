#include "rapier_space_2d.h"

#include "rapier_body_utils_2d.h"
#include "rapier_physics_server_2d.h"

#include <godot_cpp/classes/project_settings.hpp>

void RapierSpace2D::body_add_to_active_list(SelfList<RapierBody2D> *p_body) {
	active_list.add(p_body);
}

void RapierSpace2D::body_add_to_mass_properties_update_list(SelfList<RapierBody2D> *p_body) {
	mass_properties_update_list.add(p_body);
}

void RapierSpace2D::body_add_to_gravity_update_list(SelfList<RapierBody2D>* p_body) {
	gravity_update_list.add(p_body);
}

void RapierSpace2D::body_add_to_state_query_list(SelfList<RapierBody2D> *p_body) {
	state_query_list.add(p_body);
}

void RapierSpace2D::area_add_to_monitor_query_list(SelfList<RapierArea2D>* p_area) {
	monitor_query_list.add(p_area);
}

void RapierSpace2D::area_add_to_area_update_list(SelfList<RapierArea2D>* p_area) {
	area_update_list.add(p_area);
}

void RapierSpace2D::body_add_to_area_update_list(SelfList<RapierBody2D>* p_body) {
	body_area_update_list.add(p_body);
}

void RapierSpace2D::add_removed_collider(rapier2d::Handle p_handle, RapierCollisionObject2D* p_object, uint32_t p_shape_index) {
	uint64_t handle_hash = rapier2d::handle_hash(p_handle);
	ERR_FAIL_COND(removed_colliders.has(handle_hash));
	removed_colliders.insert(handle_hash, { p_object->get_rid(), p_object->get_instance_id(), p_shape_index, p_object->get_type() });
}

bool RapierSpace2D::get_removed_collider_info(rapier2d::Handle p_handle, RID& r_rid, ObjectID& r_instance_id, uint32_t& r_shape_index, RapierCollisionObject2D::Type& r_type) const {
	uint64_t handle_hash = rapier2d::handle_hash(p_handle);
	auto foundIt = removed_colliders.find(handle_hash);
	if (foundIt == removed_colliders.end()) {
		return false;
	}

	r_rid = foundIt->value.rid;
	r_instance_id = foundIt->value.instance_id;
	r_shape_index = foundIt->value.shape_index;
	r_type = foundIt->value.type;
	return true;
}

void RapierSpace2D::active_body_callback(rapier2d::Handle world_handle, const rapier2d::ActiveBodyInfo* active_body_info) {
	RapierSpace2D* space = RapierPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND(!space);

	RapierCollisionObject2D* pObject = nullptr;
	if (rapier2d::is_user_data_valid(active_body_info->body_user_data)) {
		pObject = RapierCollisionObject2D::get_body_user_data(active_body_info->body_user_data);
	}

	ERR_FAIL_COND(!pObject);
	ERR_FAIL_COND(pObject->get_type() != RapierCollisionObject2D::TYPE_BODY);

	RapierBody2D* pBody = static_cast<RapierBody2D*>(pObject);
	pBody->on_marked_active();
}

bool RapierSpace2D::collision_filter_common_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo* filter_info, CollidersInfo& r_colliders_info) {
	RapierSpace2D* space = RapierPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND_V(!space, false);

	if (rapier2d::is_user_data_valid(filter_info->user_data1)) {
		r_colliders_info.object1 = RapierCollisionObject2D::get_collider_user_data(filter_info->user_data1, r_colliders_info.shape1);
	}

	if (rapier2d::is_user_data_valid(filter_info->user_data2)) {
		r_colliders_info.object2 = RapierCollisionObject2D::get_collider_user_data(filter_info->user_data2, r_colliders_info.shape2);
	}

	ERR_FAIL_COND_V(!r_colliders_info.object1, false);
	ERR_FAIL_COND_V(!r_colliders_info.object2, false);

	if (!r_colliders_info.object1->interacts_with(r_colliders_info.object2)) {
		return false;
	}

	return true;
}

bool RapierSpace2D::collision_filter_body_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo* filter_info) {
	CollidersInfo colliders_info;
	if (!collision_filter_common_callback(world_handle, filter_info, colliders_info)) {
		return false;
	}

	const RapierBody2D* body1 = static_cast<const RapierBody2D*>(colliders_info.object1);
	const RapierBody2D* body2 = static_cast<const RapierBody2D*>(colliders_info.object2);
	if (body1->has_exception(body2->get_rid()) || body2->has_exception(body1->get_rid())) {
		return false;
	}

	return true;
}

bool RapierSpace2D::collision_filter_sensor_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo* filter_info) {
	CollidersInfo colliders_info;
	return collision_filter_common_callback(world_handle, filter_info, colliders_info);
}

#define COLLISION_EVENT_LOG	0

void RapierSpace2D::collision_event_callback(rapier2d::Handle world_handle, const rapier2d::CollisionEventInfo* event_info) {
	RapierSpace2D* space = RapierPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND(!space);

	uint32_t shape1 = 0;
	RapierCollisionObject2D* pObject1 = nullptr;
	if (rapier2d::is_user_data_valid(event_info->user_data1)) {
		pObject1 = RapierCollisionObject2D::get_collider_user_data(event_info->user_data1, shape1);
	}

	uint32_t shape2 = 0;
	RapierCollisionObject2D* pObject2 = nullptr;
	if (rapier2d::is_user_data_valid(event_info->user_data2)) {
		pObject2 = RapierCollisionObject2D::get_collider_user_data(event_info->user_data2, shape2);
	}

	rapier2d::Handle collider_handle1 = event_info->collider1;
	rapier2d::Handle collider_handle2 = event_info->collider2;

	RID rid1, rid2;
	ObjectID instanceId1;
	ObjectID instanceId2;
	RapierCollisionObject2D::Type type1, type2;

	if (event_info->is_removed) {
		// Try to get backup info for missing objects
		if (!pObject1) {
			space->get_removed_collider_info(collider_handle1, rid1, instanceId1, shape1, type1);
		} else {
			rid1 = pObject1->get_rid();
			instanceId1 = pObject1->get_instance_id();
			type1 = pObject1->get_type();
		}
		if (!pObject2) {
			space->get_removed_collider_info(collider_handle2, rid2, instanceId2, shape2, type2);
		} else {
			rid2 = pObject2->get_rid();
			instanceId2 = pObject2->get_instance_id();
			type2 = pObject2->get_type();
		}
	} else {
		ERR_FAIL_COND(!pObject1);
		rid1 = pObject1->get_rid();
		instanceId1 = pObject1->get_instance_id();
		type1 = pObject1->get_type();

		ERR_FAIL_COND(!pObject2);
		rid2 = pObject2->get_rid();
		instanceId2 = pObject2->get_instance_id();
		type2 = pObject2->get_type();
	}

#if COLLISION_EVENT_LOG
	String frame_str(itos(RapierPhysicsServer2D::singleton->get_frame()));
	String str_area("A");
	String str_body("B");
	String str_invalid("*/");
	String str_valid("/");
	String id1_str = (type1 == RapierCollisionObject2D::TYPE_AREA ? str_area : str_body) + (pObject1 ? str_valid : str_invalid) + itos(rid1.get_id()) + ":" + itos(shape1);
	String id2_str = (type2 == RapierCollisionObject2D::TYPE_AREA ? str_area : str_body) + (pObject2 ? str_valid : str_invalid) + itos(rid2.get_id()) + ":" + itos(shape2);
	String event_str(event_info->is_started ? "[COLLISION_ENTER]" : "[COLLISION_EXIT]");
	String remove_str(event_info->is_removed ? " [REMOVED]" : "");
	String sensor_str(event_info->is_sensor ? "SENSOR" : "CONTACT");
	print_line("[" + frame_str + "] " + event_str + remove_str + " [" + sensor_str + "] " + itos(collider_handle1.id) + "(" + id1_str + ")" + " | " + itos(collider_handle2.id) + "(" + id2_str + ")");
#endif

	if (event_info->is_sensor) {
		if (!instanceId1.is_valid()) {
			ERR_FAIL_COND_MSG(pObject2, "Should be able to get info about a removed object if the other one is still valid.");
			return;
		}
		if (!instanceId2.is_valid()) {
			ERR_FAIL_COND_MSG(pObject2, "Should be able to get info about a removed object if the other one is still valid.");
			return;
		}

		if (type1 != RapierCollisionObject2D::TYPE_AREA) {
			ERR_FAIL_COND(type2 != RapierCollisionObject2D::TYPE_AREA);
			SWAP(pObject1, pObject2);
			SWAP(type1, type2);
			SWAP(shape1, shape2);
			SWAP(collider_handle1, collider_handle2);
			SWAP(rid1, rid2);
			SWAP(instanceId1, instanceId2);
		}

		RapierArea2D* pArea = static_cast<RapierArea2D*>(pObject1);
		uint32_t area_shape = shape1;
		uint32_t other_shape = shape2;
		RID other_rid = rid2;
		ObjectID other_instance_id = instanceId2;
		if (type2 == RapierCollisionObject2D::TYPE_AREA) {
			RapierArea2D* pArea2 = static_cast<RapierArea2D*>(pObject2);
			if (event_info->is_started) {
				ERR_FAIL_COND(!pArea);
				ERR_FAIL_COND(!pArea2);
				pArea->on_area_enter(collider_handle2, pArea2, other_shape, other_rid, other_instance_id, collider_handle1, area_shape);
				pArea2->on_area_enter(collider_handle1, pArea, area_shape, other_rid, other_instance_id, collider_handle2, other_shape);
			} else {
				if (pArea) {
					pArea->on_area_exit(collider_handle2, pArea2, other_shape, other_rid, other_instance_id, collider_handle1, area_shape);
				} else {
					// Try to retrieve area if not destroyed yet
					pArea = space->get_area_from_rid(rid1);
					if (pArea) {
						// Use invalid area case to keep counters consistent for already removed collider
						pArea->on_area_exit(collider_handle2, nullptr, other_shape, other_rid, other_instance_id, collider_handle1, area_shape);
					}
				}
				if (pArea2) {
					pArea2->on_area_exit(collider_handle1, pArea, area_shape, other_rid, other_instance_id, collider_handle2, other_shape);
				} else {
					// Try to retrieve area if not destroyed yet
					pArea2 = space->get_area_from_rid(rid2);
					if (pArea2) {
						// Use invalid area case to keep counters consistent for already removed collider
						pArea2->on_area_exit(collider_handle1, nullptr, area_shape, other_rid, other_instance_id, collider_handle2, other_shape);
					}
				}
			}
		} else {
			RapierBody2D* pBody = static_cast<RapierBody2D*>(pObject2);
			if (event_info->is_started) {
				ERR_FAIL_COND(!pArea);
				pArea->on_body_enter(collider_handle2, pBody, other_shape, other_rid, other_instance_id, collider_handle1, area_shape);
			} else if (pArea) {
				pArea->on_body_exit(collider_handle2, pBody, other_shape, other_rid, other_instance_id, collider_handle1, area_shape);
			} else {
				// Try to retrieve area if not destroyed yet
				pArea = space->get_area_from_rid(rid1);
				if (pArea) {
					// Use invalid body case to keep counters consistent for already removed collider
					pArea->on_body_exit(collider_handle2, nullptr, other_shape, other_rid, other_instance_id, collider_handle1, area_shape, false);
				}
			}
		}
	} else {
		// Body contacts use contact_force_event_callback instead
		ERR_FAIL_MSG("Shouldn't receive rigidbody collision events.");
	}
}

RapierSpace2D* g_contact_events_space = nullptr;
RapierBody2D* g_contact_events_body1 = nullptr;
RapierBody2D* g_contact_events_body2 = nullptr;
uint32_t g_contact_events_shape1 = 0;
uint32_t g_contact_events_shape2 = 0;

bool RapierSpace2D::contact_force_event_callback(rapier2d::Handle world_handle, const rapier2d::ContactForceEventInfo* event_info) {
	RapierSpace2D* space = RapierPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND_V(!space, false);

	g_contact_events_space = space;

	bool send_contacts = false;

#ifdef DEBUG_ENABLED
	if (space->is_debugging_contacts()) {
		send_contacts = true;
	}
#endif

	uint32_t shape1 = 0;
	RapierCollisionObject2D* pObject1 = nullptr;
	if (rapier2d::is_user_data_valid(event_info->user_data1)) {
		pObject1 = RapierCollisionObject2D::get_collider_user_data(event_info->user_data1, shape1);
	}
	ERR_FAIL_COND_V(!pObject1, false);
	ERR_FAIL_COND_V(pObject1->get_type() != RapierCollisionObject2D::TYPE_BODY, false);
	g_contact_events_body1 = static_cast<RapierBody2D*>(pObject1);
	g_contact_events_shape1 = shape1;

	uint32_t shape2 = 0;
	RapierCollisionObject2D* pObject2 = nullptr;
	if (rapier2d::is_user_data_valid(event_info->user_data2)) {
		pObject2 = RapierCollisionObject2D::get_collider_user_data(event_info->user_data2, shape2);
	}
	ERR_FAIL_COND_V(!pObject2, false);
	ERR_FAIL_COND_V(pObject2->get_type() != RapierCollisionObject2D::TYPE_BODY, false);
	g_contact_events_body2 = static_cast<RapierBody2D*>(pObject2);
	g_contact_events_shape2 = shape2;

	if (g_contact_events_body1->can_report_contacts()) {
		send_contacts = true;
	}

	if (g_contact_events_body2->can_report_contacts()) {
		send_contacts = true;
	}

	return send_contacts;
}

bool RapierSpace2D::contact_point_callback(rapier2d::Handle world_handle, const rapier2d::ContactPointInfo* contact_info) {
	RapierSpace2D* space = g_contact_events_space;
	ERR_FAIL_COND_V(!space, false);
	ERR_FAIL_COND_V(space->get_handle().id != world_handle.id, false);

	Vector2 pos1(contact_info->local_pos_1.x, contact_info->local_pos_1.y);
	Vector2 pos2(contact_info->local_pos_2.x, contact_info->local_pos_2.y);

	bool keep_sending_contacts = false;

#ifdef DEBUG_ENABLED
	if (space->is_debugging_contacts()) {
		keep_sending_contacts = true;
		space->add_debug_contact(pos1);
		space->add_debug_contact(pos2);
	}
#endif

	ERR_FAIL_COND_V(!g_contact_events_body1, false);
	RapierBody2D* body1 = g_contact_events_body1;
	ERR_FAIL_COND_V(!g_contact_events_body2, false);
	RapierBody2D* body2 = g_contact_events_body2;

	real_t depth = MAX(0.0, -contact_info->distance); // negative distance means penetration

	Vector2 normal(contact_info->normal.x, contact_info->normal.y);
	Vector2 tangent = normal.orthogonal();
	Vector2 impulse = contact_info->impulse * normal + contact_info->tangent_impulse * tangent;

	if(body1->can_report_contacts()) {
		keep_sending_contacts = true;
		Vector2 vel_pos2(contact_info->velocity_pos_2.x, contact_info->velocity_pos_2.y);
		body1->add_contact(pos1, -normal, depth, (int)g_contact_events_shape1, pos2, (int)g_contact_events_shape2, body2->get_instance_id(), body2->get_rid(), vel_pos2, impulse);
	}

	if (body2->can_report_contacts()) {
		keep_sending_contacts = true;
		Vector2 vel_pos1(contact_info->velocity_pos_1.x, contact_info->velocity_pos_1.y);
		body2->add_contact(pos2, normal, depth, (int)g_contact_events_shape2, pos1, (int)g_contact_events_shape1, body1->get_instance_id(), body1->get_rid(), vel_pos1, impulse);
	}

	return keep_sending_contacts;
}

void RapierSpace2D::step(real_t p_step) {
	last_step = p_step;
	contact_debug_count = 0;
	
	ProjectSettings* project_settings = ProjectSettings::get_singleton();

	default_gravity_dir = project_settings->get_setting_with_override("physics/2d/default_gravity_vector");
	default_gravity_value = project_settings->get_setting_with_override("physics/2d/default_gravity");

	default_linear_damping = project_settings->get_setting_with_override("physics/2d/default_linear_damp");
	default_angular_damping = project_settings->get_setting_with_override("physics/2d/default_angular_damp");

	for (SelfList<RapierBody2D>* body_iterator = mass_properties_update_list.first(); body_iterator; ) {
		RapierBody2D* body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->update_mass_properties();
	}

	for (SelfList<RapierArea2D>* area_iterator = area_update_list.first(); area_iterator; ) {
		RapierArea2D* area = area_iterator->self();
		area_iterator = area_iterator->next();
		area->update_area_override();
	}

	for (SelfList<RapierBody2D>* body_iterator = body_area_update_list.first(); body_iterator; ) {
		RapierBody2D* body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->update_area_override();
	}

	for (SelfList<RapierBody2D>* body_iterator = gravity_update_list.first(); body_iterator; ) {
		RapierBody2D* body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->update_gravity(p_step);
	}

	rapier2d::SimulationSettings settings = rapier2d::default_simulation_settings();
	settings.delta_time = p_step;
	settings.gravity.x = default_gravity_dir.x * default_gravity_value;
	settings.gravity.y = default_gravity_dir.y * default_gravity_value;
	settings.max_velocity_iterations = 8;
	settings.max_velocity_friction_iterations = 8;
	settings.max_stabilization_iterations = 1;

	ERR_FAIL_COND(!is_handle_valid(handle));
	rapier2d::world_step(handle , &settings);

	// Needed only for one physics step to retrieve lost info
	removed_colliders.clear();

	for (SelfList<RapierBody2D>* body_iterator = active_list.first(); body_iterator; ) {
		RapierBody2D* body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->on_update_active();
	}
}

uint32_t g_query_collision_layer_mask = 0;
ObjectID g_query_canvas_instance_id;
rapier2d::Handle* g_query_exclude = nullptr;
uint32_t g_query_exclude_size = 0;
RID* g_query_exclude_body  = nullptr;

struct QueryCallbackScope
{
	QueryCallbackScope(uint32_t collision_layer_mask)
	{
		g_query_collision_layer_mask = collision_layer_mask;
	}

	~QueryCallbackScope()
	{
		g_query_collision_layer_mask = 0;
		g_query_canvas_instance_id = ObjectID();
		g_query_exclude = nullptr;
		g_query_exclude_size = 0;
		g_query_exclude_body = nullptr;
	}
};

// Returns true to ignore the collider
bool RapierSpace2D::_is_handle_excluded_callback(const rapier2d::Handle world_handle, const rapier2d::Handle collider_handle, const rapier2d::UserData* user_data) {
	for (uint32_t exclude_index = 0; exclude_index < g_query_exclude_size; ++exclude_index) {
		if (rapier2d::are_handles_equal(g_query_exclude[exclude_index], collider_handle)) {
			return true;
		}
	}

	ERR_FAIL_COND_V(!rapier2d::is_user_data_valid(*user_data), false);

	uint32_t shape_index = 0;
	RapierCollisionObject2D* collision_object_2d = RapierCollisionObject2D::get_collider_user_data(*user_data, shape_index);
	ERR_FAIL_COND_V(!collision_object_2d, false);

	if (g_query_canvas_instance_id.is_valid()) {
		if (g_query_canvas_instance_id != collision_object_2d->get_canvas_instance_id()) {
			return true;
		}
	}

	if (0 == (collision_object_2d->get_collision_layer() & g_query_collision_layer_mask)) {
		return true;
	}

	if(g_query_exclude_body && *g_query_exclude_body == collision_object_2d->get_rid()) {
		return true;
	}

	return RapierPhysicsServer2D::singleton->get_active_space(world_handle)->get_direct_state()->is_body_excluded_from_query(collision_object_2d->get_rid());
}

void RapierSpace2D::call_queries() {
	for (SelfList<RapierBody2D>* body_iterator = state_query_list.first(); body_iterator; ) {
		RapierBody2D* body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->call_queries();
	}

	for (SelfList<RapierArea2D>* area_iterator = monitor_query_list.first(); area_iterator; ) {
		RapierArea2D* area = area_iterator->self();
		area_iterator = area_iterator->next();
		area->call_queries();
	}
}

void RapierSpace2D::set_param(PhysicsServer2D::SpaceParameter p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer2D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
			contact_recycle_radius = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
			contact_max_separation = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
			contact_max_allowed_penetration = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
			contact_bias = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			body_linear_velocity_sleep_threshold = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			body_angular_velocity_sleep_threshold = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			body_time_to_sleep = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
			constraint_bias = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_SOLVER_ITERATIONS:
			solver_iterations = p_value;
			break;
	}
}

real_t RapierSpace2D::get_param(PhysicsServer2D::SpaceParameter p_param) const {
	switch (p_param) {
		case PhysicsServer2D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
			return contact_recycle_radius;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
			return contact_max_separation;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
			return contact_max_allowed_penetration;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
			return contact_bias;
		case PhysicsServer2D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			return body_linear_velocity_sleep_threshold;
		case PhysicsServer2D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			return body_angular_velocity_sleep_threshold;
		case PhysicsServer2D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			return body_time_to_sleep;
		case PhysicsServer2D::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
			return constraint_bias;
		case PhysicsServer2D::SPACE_PARAM_SOLVER_ITERATIONS:
			return solver_iterations;
	}
	return 0;
}

void RapierSpace2D::set_default_area_param(PhysicsServer2D::AreaParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer2D::AREA_PARAM_GRAVITY: {
			default_gravity_value = p_value;
		} break;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_VECTOR: {
			default_gravity_dir = p_value;
		} break;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP: {
			default_linear_damping = p_value;
		} break;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP: {
			default_angular_damping = p_value;
		} break;
		default:
			ERR_FAIL_MSG("Unsupported space default area param " + itos(p_param));
	}
}

Variant RapierSpace2D::get_default_area_param(PhysicsServer2D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer2D::AREA_PARAM_GRAVITY:
			return default_gravity_value;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_VECTOR:
			return default_gravity_dir;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP:
			return default_linear_damping;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP:
			return default_angular_damping;
	}

	ERR_FAIL_V_MSG(Variant(), "Unsupported space default area param " + itos(p_param));
}

RapierArea2D* RapierSpace2D::get_area_from_rid(RID p_area_rid) const {
	return RapierPhysicsServer2D::singleton->area_owner.get_or_null(p_area_rid);
}

RapierBody2D* RapierSpace2D::get_body_from_rid(RID p_body_rid) const {
	return RapierPhysicsServer2D::singleton->body_owner.get_or_null(p_body_rid);
}

RapierShape2D* RapierSpace2D::get_shape_from_rid(RID p_shape_rid) const {
	return RapierPhysicsServer2D::singleton->shape_owner.get_or_null(p_shape_rid);
}

void RapierSpace2D::lock() {
	locked = true;
}

void RapierSpace2D::unlock() {
	locked = false;
}

bool RapierSpace2D::is_locked() const {
	return locked;
}

RapierDirectSpaceState2D *RapierSpace2D::get_direct_state() {
	return direct_access;
}

RapierSpace2D::RapierSpace2D() {
	ProjectSettings* project_settings = ProjectSettings::get_singleton();

	// Use the default physics step for force application on the first frame
	int physics_fps = project_settings->get_setting_with_override("physics/common/physics_ticks_per_second");
	last_step = physics_fps ? 1.0f / (real_t)physics_fps : 0.001f;
	
	body_linear_velocity_sleep_threshold = project_settings->get_setting_with_override("physics/2d/sleep_threshold_linear");
	body_angular_velocity_sleep_threshold = project_settings->get_setting_with_override("physics/2d/sleep_threshold_angular");
	body_time_to_sleep = project_settings->get_setting_with_override("physics/2d/time_before_sleep");
	solver_iterations = project_settings->get_setting_with_override("physics/2d/solver/solver_iterations");
	contact_recycle_radius = project_settings->get_setting_with_override("physics/2d/solver/contact_recycle_radius");
	contact_max_separation = project_settings->get_setting_with_override("physics/2d/solver/contact_max_separation");
	contact_max_allowed_penetration = project_settings->get_setting_with_override("physics/2d/solver/contact_max_allowed_penetration");
	contact_bias = project_settings->get_setting_with_override("physics/2d/solver/default_contact_bias");
	constraint_bias = project_settings->get_setting_with_override("physics/2d/solver/default_constraint_bias");

	direct_access = memnew(RapierDirectSpaceState2D);
	direct_access->space = this;

	ERR_FAIL_COND(is_handle_valid(handle));

	rapier2d::WorldSettings world_settings = rapier2d::default_world_settings();
	world_settings.sleep_linear_threshold = body_linear_velocity_sleep_threshold;
	world_settings.sleep_angular_threshold = body_angular_velocity_sleep_threshold;
	world_settings.sleep_time_until_sleep = body_time_to_sleep;

	handle = rapier2d::world_create(&world_settings);
	ERR_FAIL_COND(!is_handle_valid(handle));

	rapier2d::world_set_active_body_callback(handle, active_body_callback);
	rapier2d::world_set_body_collision_filter_callback(handle, collision_filter_body_callback);
	rapier2d::world_set_sensor_collision_filter_callback(handle, collision_filter_sensor_callback);
	rapier2d::world_set_collision_event_callback(handle, collision_event_callback);
	rapier2d::world_set_contact_force_event_callback(handle, contact_force_event_callback);
	rapier2d::world_set_contact_point_callback(handle, contact_point_callback);
}

RapierSpace2D::~RapierSpace2D() {
	ERR_FAIL_COND(!is_handle_valid(handle));
	rapier2d::world_destroy(handle);
	handle = rapier2d::invalid_handle();

	memdelete(direct_access);
}

int RapierDirectSpaceState2D::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) {
	ERR_FAIL_COND_V(space->locked, 0);
	ERR_FAIL_COND_V(!is_handle_valid(space->handle), 0);
	ERR_FAIL_COND_V(p_result_max < 0, 0);

	rapier2d::Vector rapier_pos = { position.x, position.y };

	rapier2d::PointHitInfo* hit_info_array = (rapier2d::PointHitInfo*)alloca(p_result_max * sizeof(rapier2d::PointHitInfo));

	QueryCallbackScope queryCallback(collision_mask);
	g_query_canvas_instance_id = ObjectID(canvas_instance_id);

	uint32_t result_count = rapier2d::intersect_point(space->handle, &rapier_pos, collide_with_bodies, collide_with_areas, hit_info_array, p_result_max, RapierSpace2D::_is_handle_excluded_callback);
	ERR_FAIL_COND_V(result_count > (uint32_t)p_result_max, 0);

	for(uint32_t i = 0; i < result_count; i++){
		rapier2d::PointHitInfo& hit_info = hit_info_array[i];
		PhysicsServer2DExtensionShapeResult& result = r_results[i];
		ERR_CONTINUE(!rapier2d::is_user_data_valid(hit_info.user_data));

		uint32_t shape_index = 0;
		RapierCollisionObject2D* collision_object_2d = RapierCollisionObject2D::get_collider_user_data(hit_info.user_data, shape_index);
		ERR_CONTINUE(collision_object_2d == nullptr);
		
		result.shape = shape_index;
		result.collider_id = collision_object_2d->get_instance_id();
		result.rid = collision_object_2d->get_rid();

		if (result.collider_id.is_valid()){
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

	QueryCallbackScope queryCallback(collision_mask);

	rapier2d::RayHitInfo hit_info;
	bool collide = rapier2d::intersect_ray(space->handle,
											&rapier_from,
											&rapier_dir,
											length,
											collide_with_bodies,
											collide_with_areas,
											hit_from_inside,
											&hit_info,
											RapierSpace2D::_is_handle_excluded_callback);
	
	if (collide) {
		r_result->position = Vector2(hit_info.position.x, hit_info.position.y);
		r_result->normal = Vector2(hit_info.normal.x, hit_info.normal.y);

		ERR_FAIL_COND_V(!rapier2d::is_user_data_valid(hit_info.user_data), false);
		uint32_t shape_index = 0;
		RapierCollisionObject2D* collision_object_2d = RapierCollisionObject2D::get_collider_user_data(hit_info.user_data, shape_index);
		ERR_FAIL_NULL_V(collision_object_2d, false);

		r_result->shape = shape_index;
		r_result->collider_id = collision_object_2d->get_instance_id();
		r_result->rid = collision_object_2d->get_rid();

		if (r_result->collider_id.is_valid()){
			r_result->collider = RapierSpace2D::_get_object_instance_hack(r_result->collider_id);
		}
		
		return true;
	}
	
	return false;
}

bool RapierDirectSpaceState2D::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, real_t *p_closest_safe, real_t *p_closest_unsafe) {
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion = { motion.x, motion.y };
	rapier2d::Vector rapier_pos = { transform.get_origin().x, transform.get_origin().y };
	real_t rotation = transform.get_rotation();

	QueryCallbackScope queryCallback(collision_mask);
	real_t hit = rapier2d::shape_casting(space->handle, &rapier_motion, &rapier_pos, rotation, shape_handle, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback).toi;
	*p_closest_safe = hit;
	*p_closest_unsafe = hit;
	return true;
}

bool RapierDirectSpaceState2D::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion {motion.x, motion.y};
	rapier2d::Vector rapier_pos {transform.get_origin().x, transform.get_origin().y};
	real_t rotation = transform.get_rotation();

	Vector2* results_out = static_cast<Vector2*>(results);

	QueryCallbackScope queryCallback(collision_mask);
	g_query_exclude = (rapier2d::Handle*)alloca((max_results) * sizeof(rapier2d::Handle));
	g_query_exclude_size = 0;

	int cpt = 0;
	int array_idx = 0;
	do {
		
		rapier2d::ShapeCastResult result = rapier2d::shape_casting(space->handle, &rapier_motion, &rapier_pos, rotation, shape_handle, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback);
		if (!result.collided){
			break;
		}
		*result_count++;
		g_query_exclude[g_query_exclude_size++] = result.collider;

		results_out[array_idx++] = Vector2(result.witness1.x, result.witness1.y);
		results_out[array_idx++] = Vector2(result.witness2.x, result.witness2.y);

		cpt++;
	} while(cpt < max_results);

	return array_idx > 0;
}

int RapierDirectSpaceState2D::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) {
	
	RapierShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	rapier2d::Handle shape_handle = shape->get_rapier_shape();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(shape_handle), false);

	rapier2d::Vector rapier_motion {motion.x, motion.y};
	rapier2d::Vector rapier_pos {transform.get_origin().x, transform.get_origin().y};
	real_t rotation = transform.get_rotation();

	// Exclude BODY RID
	rapier2d::Handle *rapier_exclude = nullptr;

	QueryCallbackScope queryCallback(collision_mask);
	g_query_exclude = (rapier2d::Handle*)alloca((p_result_max) * sizeof(rapier2d::Handle));
	g_query_exclude_size = 0;

	int cpt = 0;
	do {
		
		rapier2d::ShapeCastResult result = rapier2d::shape_casting(space->handle, &rapier_motion, &rapier_pos, rotation, shape_handle, collide_with_bodies, collide_with_areas, RapierSpace2D::_is_handle_excluded_callback);
		if (!result.collided){
			break;
		}
		g_query_exclude[g_query_exclude_size++] = result.collider;
		
		ERR_CONTINUE(!rapier2d::is_user_data_valid(result.user_data));
		uint32_t shape_index = 0;
		RapierCollisionObject2D* collision_object_2d = RapierCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
		ERR_CONTINUE(!collision_object_2d);
		
		PhysicsServer2DExtensionShapeResult& result_out = r_results[cpt];

		result_out.shape = shape_index;
		result_out.rid = collision_object_2d->get_rid();
		result_out.collider_id = collision_object_2d->get_instance_id();

		if (result_out.collider_id.is_valid()){
			result_out.collider = RapierSpace2D::_get_object_instance_hack(result_out.collider_id);
		}

		cpt++;
	
	} while(cpt < p_result_max);

	return cpt;
}

bool RapierSpace2D::test_body_motion(RapierBody2D *p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *r_result) const {
	Transform2D body_transform = p_from; // Because body_transform needs to be modified during recovery
	
	// Step 1: recover motion.
	Vector2 recover_motion;
	Rect2 body_aabb = p_body->get_aabb();
	bool recovered = RapierBodyUtils2D::body_motion_recover(*this, *p_body, body_transform, p_margin, recover_motion, body_aabb);

	// Step 2: Cast motion.
	real_t best_safe = 1.0;
	real_t best_unsafe = 1.0;
	int best_body_shape = -1;
	RapierBodyUtils2D::cast_motion(*this, *p_body, body_transform, p_motion, body_aabb, best_safe, best_unsafe, best_body_shape);

	// Step 3: Rest Info
	bool collided = false;
	if ((p_recovery_as_collision && recovered) || (best_safe < 1.0)) {
		if (best_safe >= 1.0) {
			best_body_shape = -1; //no best shape with cast, reset to -1
		}

		// Get the rest info in unsafe advance
		Vector2 unsafe_motion = p_motion * best_unsafe;
		body_transform.columns[2] += unsafe_motion;
		body_aabb.position += unsafe_motion;

		collided = RapierBodyUtils2D::body_motion_collide(*this, *p_body, body_transform, body_aabb, best_body_shape, p_margin, r_result);
	}

	if (r_result) {
		if (collided) {
			r_result->travel = recover_motion + p_motion * best_safe;
			r_result->remainder = p_motion - p_motion * best_safe;
			r_result->collision_safe_fraction = best_safe;
			r_result->collision_unsafe_fraction = best_unsafe;
		} else {
			r_result->travel = recover_motion + p_motion;
			r_result->remainder = Vector2();
			r_result->collision_depth = 0.0f;
			r_result->collision_safe_fraction = 1.0f;
			r_result->collision_unsafe_fraction = 1.0f;
		}
	}

	return collided;
}

bool RapierSpace2D::rapier_shape_cast(rapier2d::Handle p_shape_handle, const Transform2D &p_transform, const Vector2 &p_motion, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, rapier2d::ShapeCastResult *p_results, int32_t p_max_results, int32_t *p_result_count) const {
	ERR_FAIL_COND_V(p_max_results < 1, false);

	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(p_shape_handle), false);

	rapier2d::Vector rapier_motion {p_motion.x, p_motion.y};
	rapier2d::Vector rapier_pos {p_transform.get_origin().x, p_transform.get_origin().y};
	real_t rotation = p_transform.get_rotation();

	QueryCallbackScope queryCallback(p_collision_mask);
	g_query_exclude = (rapier2d::Handle*)alloca((p_max_results) * sizeof(rapier2d::Handle));
	g_query_exclude_size = 0;

	int cpt = 0;
	int array_idx = 0;
	do {
		rapier2d::ShapeCastResult& result = p_results[cpt];
		result = rapier2d::shape_casting(handle, &rapier_motion, &rapier_pos, rotation, p_shape_handle, p_collide_with_bodies, p_collide_with_areas, RapierSpace2D::_is_handle_excluded_callback);
		if (!result.collided){
			break;
		}
		(*p_result_count)++;
		g_query_exclude[g_query_exclude_size++] = result.collider;
		cpt++;
	} while(cpt < p_max_results);

	return array_idx > 0;
}

int RapierSpace2D::rapier_intersect_shape(rapier2d::Handle p_shape_handle, const Transform2D &p_transform, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, rapier2d::PointHitInfo *p_results, int32_t p_max_results, int32_t *p_result_count, RID p_exclude_body) const {
	ERR_FAIL_COND_V(p_max_results < 1, false);

	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(p_shape_handle), false);

	rapier2d::Vector rapier_pos {p_transform.get_origin().x, p_transform.get_origin().y};
	real_t rotation = p_transform.get_rotation();

	QueryCallbackScope queryCallback(p_collision_mask);
	g_query_exclude = (rapier2d::Handle*)alloca((p_max_results) * sizeof(rapier2d::Handle));
	g_query_exclude_size = 0;
	g_query_exclude_body = &p_exclude_body;

	return rapier2d::intersect_shape(handle, &rapier_pos, rotation, p_shape_handle, p_collide_with_bodies, p_collide_with_areas, p_results, p_max_results, RapierSpace2D::_is_handle_excluded_callback);
}

int RapierSpace2D::rapier_intersect_aabb(Rect2 p_aabb, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, rapier2d::PointHitInfo *p_results, int32_t p_max_results, int32_t *p_result_count, RID p_exclude_body) const {
	ERR_FAIL_COND_V(p_max_results < 1, false);

	rapier2d::Vector rect_begin {p_aabb.position.x, p_aabb.position.y};
	rapier2d::Vector rect_end {p_aabb.get_end().x, p_aabb.get_end().y};

	QueryCallbackScope queryCallback(p_collision_mask);
	g_query_exclude = (rapier2d::Handle*)alloca((p_max_results) * sizeof(rapier2d::Handle));
	g_query_exclude_size = 0;
	g_query_exclude_body = &p_exclude_body;

	return rapier2d::intersect_aabb(handle, &rect_begin, &rect_end, p_collide_with_bodies, p_collide_with_areas, p_results, p_max_results, RapierSpace2D::_is_handle_excluded_callback);
}
