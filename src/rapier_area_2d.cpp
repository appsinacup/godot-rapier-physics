#include "rapier_area_2d.h"
#include "rapier_body_2d.h"
#include "rapier_physics_server_2d.h"
#include "rapier_space_2d.h"

void RapierArea2D::set_space(RapierSpace2D *p_space) {
	if (p_space == get_space()) {
		return;
	}

	if (get_space()) {
		// Need to keep in list to handle remove events for bodies
		if (!detected_bodies.is_empty() && !monitor_query_list.in_list()) {
			get_space()->area_add_to_monitor_query_list(&monitor_query_list);
		}

		for (auto E = detected_bodies.begin(); E != detected_bodies.end(); ++E) {
			RapierBody2D *body = get_space()->get_body_from_rid(E->key);
			if (!body || !body->get_space()) {
				continue;
			}
			body->remove_area(this);
		}
		detected_bodies.clear();

		area_override_update_list.remove_from_list();
	}

	monitored_objects.clear();

	_set_space(p_space);
}

void RapierArea2D::on_body_enter(rapier2d::Handle p_collider_handle, RapierBody2D *p_body, uint32_t p_body_shape, RID p_body_rid, ObjectID p_body_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape) {
	ERR_FAIL_COND(!p_body); // Shouldn't happen after removal

	// Add to keep track of currently detected bodies
	BodyRefCount &bodyRefCount = detected_bodies[p_body_rid];
	if (bodyRefCount.count++ == 0) {
		p_body->add_area(this);
	}

	if (monitor_callback.is_null()) {
		return;
	}

	area_detection_counter++;

	uint64_t handle_pair_hash = rapier2d::handle_pair_hash(p_collider_handle, p_area_collider_handle);
	ERR_FAIL_COND(monitored_objects.has(handle_pair_hash));
	monitored_objects.insert(handle_pair_hash, { p_body_rid, p_body_instance_id, p_body_shape, p_area_shape, Type::TYPE_BODY, 1 });

	if (!monitor_query_list.in_list()) {
		ERR_FAIL_COND(!get_space());
		get_space()->area_add_to_monitor_query_list(&monitor_query_list);
	}
}

void RapierArea2D::on_body_exit(rapier2d::Handle p_collider_handle, RapierBody2D *p_body, uint32_t p_body_shape, RID p_body_rid, ObjectID p_body_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape, bool p_update_detection) {
	if (p_update_detection) {
		// Remove from currently detected bodies
		auto foundIt = detected_bodies.find(p_body_rid);
		ERR_FAIL_COND(foundIt == detected_bodies.end());
		ERR_FAIL_COND(foundIt->value.count == 0);
		if (foundIt->value.count-- == 1) {
			detected_bodies.remove(foundIt);
			if (p_body) {
				p_body->remove_area(this);
			}
		}
	}

	if (monitor_callback.is_null()) {
		return;
	}

	if (p_body) {
		area_detection_counter--;
	}

	uint64_t handle_pair_hash = rapier2d::handle_pair_hash(p_collider_handle, p_area_collider_handle);

	if (monitored_objects.has(handle_pair_hash)) {
		ERR_FAIL_COND(monitored_objects[handle_pair_hash].state != 1);
		monitored_objects.erase(handle_pair_hash);
	} else {
		monitored_objects.insert(handle_pair_hash, { p_body_rid, p_body_instance_id, p_body_shape, p_area_shape, Type::TYPE_BODY, -1 });

		if (!monitor_query_list.in_list()) {
			ERR_FAIL_COND(!get_space());
			get_space()->area_add_to_monitor_query_list(&monitor_query_list);
		}
	}
}

void RapierArea2D::on_area_enter(rapier2d::Handle p_collider_handle, RapierArea2D *p_other_area, uint32_t p_other_area_shape, RID p_other_area_rid, ObjectID p_other_area_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape) {
	ERR_FAIL_COND(!p_other_area); // Shouldn't happen after removal

	if (area_monitor_callback.is_null()) {
		return;
	}

	if (!p_other_area->is_monitorable()) {
		return;
	}

	p_other_area->area_detection_counter++;

	uint64_t handle_pair_hash = rapier2d::handle_pair_hash(p_collider_handle, p_area_collider_handle);

	ERR_FAIL_COND(monitored_objects.has(handle_pair_hash));
	monitored_objects.insert(handle_pair_hash, { p_other_area_rid, p_other_area_instance_id, p_other_area_shape, p_area_shape, Type::TYPE_AREA, 1 });

	if (!monitor_query_list.in_list()) {
		ERR_FAIL_COND(!get_space());
		get_space()->area_add_to_monitor_query_list(&monitor_query_list);
	}
}

void RapierArea2D::on_area_exit(rapier2d::Handle p_collider_handle, RapierArea2D *p_other_area, uint32_t p_other_area_shape, RID p_other_area_rid, ObjectID p_other_area_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape) {
	if (area_monitor_callback.is_null()) {
		return;
	}

	if (p_other_area) {
		if (!p_other_area->is_monitorable()) {
			return;
		}

		ERR_FAIL_COND(p_other_area->area_detection_counter == 0);
		p_other_area->area_detection_counter--;
	}

	uint64_t handle_pair_hash = rapier2d::handle_pair_hash(p_collider_handle, p_area_collider_handle);

	if (monitored_objects.has(handle_pair_hash)) {
		ERR_FAIL_COND(monitored_objects[handle_pair_hash].state != 1);
		monitored_objects.erase(handle_pair_hash);
	} else {
		monitored_objects.insert(handle_pair_hash, { p_other_area_rid, p_other_area_instance_id, p_other_area_shape, p_area_shape, Type::TYPE_AREA, -1 });

		if (!monitor_query_list.in_list()) {
			ERR_FAIL_COND(!get_space());
			get_space()->area_add_to_monitor_query_list(&monitor_query_list);
		}
	}
}

void RapierArea2D::update_area_override() {
	area_override_update_list.remove_from_list();

	for (auto E = detected_bodies.begin(); E != detected_bodies.end(); ++E) {
		RapierBody2D *body = get_space()->get_body_from_rid(E->key);
		if (!body || !body->get_space()) {
			continue;
		}
		body->update_area_override();
	}
}

void RapierArea2D::set_monitor_callback(const Callable &p_callback) {
	uint64_t id = p_callback.get_object_id();

	if (id == monitor_callback.get_object_id()) {
		monitor_callback = p_callback;
		return;
	}

	_unregister_shapes();

	monitor_callback = p_callback;

	monitored_objects.clear();
}

void RapierArea2D::set_area_monitor_callback(const Callable &p_callback) {
	uint64_t id = p_callback.get_object_id();

	if (id == area_monitor_callback.get_object_id()) {
		area_monitor_callback = p_callback;
		return;
	}

	_unregister_shapes();

	area_monitor_callback = p_callback;

	monitored_objects.clear();
}

void RapierArea2D::_set_space_override_mode(PhysicsServer2D::AreaSpaceOverrideMode &r_mode, PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	bool had_override = has_any_space_override();
	r_mode = p_value;
	bool has_override = has_any_space_override();
	// Update currently detected bodies if needed
	if (has_override != had_override) {
		if (has_override) {
			_enable_space_override();
		} else {
			_disable_space_override();
		}
	}
}

bool RapierArea2D::has_any_space_override() const {
	if (gravity_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
		return true;
	}
	if (linear_damping_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
		return true;
	}
	if (angular_damping_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
		return true;
	}
	return false;
}

void RapierArea2D::_enable_space_override() {
	for (auto E = detected_bodies.begin(); E != detected_bodies.end(); ++E) {
		RapierBody2D *body = get_space()->get_body_from_rid(E->key);
		if (!body || !body->get_space()) {
			continue;
		}
		body->add_area(this);
	}

	// No need to update anymore if it was scheduled before
	area_override_update_list.remove_from_list();
}

void RapierArea2D::_disable_space_override() {
	for (auto E = detected_bodies.begin(); E != detected_bodies.end(); ++E) {
		RapierBody2D *body = get_space()->get_body_from_rid(E->key);
		if (!body || !body->get_space()) {
			continue;
		}
		body->remove_area(this);
	}

	// No need to update anymore if it was scheduled before
	area_override_update_list.remove_from_list();
}

void RapierArea2D::_reset_space_override() {
	for (auto E = detected_bodies.begin(); E != detected_bodies.end(); ++E) {
		RapierBody2D *body = get_space()->get_body_from_rid(E->key);
		if (!body || !body->get_space()) {
			continue;
		}
		body->remove_area(this);
		body->add_area(this);
	}

	// No need to update anymore if it was scheduled before
	area_override_update_list.remove_from_list();
}

void RapierArea2D::set_param(PhysicsServer2D::AreaParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer2D::AREA_PARAM_GRAVITY_OVERRIDE_MODE:
			_set_space_override_mode(gravity_override_mode, (PhysicsServer2D::AreaSpaceOverrideMode)(int)p_value);
			break;
		case PhysicsServer2D::AREA_PARAM_GRAVITY: {
			real_t new_gravity = p_value;
			if (new_gravity != gravity) {
				gravity = new_gravity;
				if (gravity_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					// Update currently detected bodies
					if (!area_override_update_list.in_list() && get_space()) {
						get_space()->area_add_to_area_update_list(&area_override_update_list);
					}
				}
			}
		} break;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_VECTOR: {
			Vector2 new_gravity_vector = p_value;
			if (gravity_vector != new_gravity_vector) {
				gravity_vector = new_gravity_vector;
				if (gravity_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					// Update currently detected bodies
					if (!area_override_update_list.in_list() && get_space()) {
						get_space()->area_add_to_area_update_list(&area_override_update_list);
					}
				}
			}
		} break;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_IS_POINT: {
			bool new_gravity_is_point = p_value;
			if (gravity_is_point != new_gravity_is_point) {
				gravity_is_point = new_gravity_is_point;
				if (gravity_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					// Update currently detected bodies
					if (!area_override_update_list.in_list() && get_space()) {
						get_space()->area_add_to_area_update_list(&area_override_update_list);
					}
				}
			}
		} break;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: {
			real_t new_gravity_point_unit_distance = p_value;
			if (gravity_point_unit_distance != new_gravity_point_unit_distance) {
				gravity_point_unit_distance = new_gravity_point_unit_distance;
				if (gravity_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					// Update currently detected bodies
					if (!area_override_update_list.in_list() && get_space()) {
						get_space()->area_add_to_area_update_list(&area_override_update_list);
					}
				}
			}
		} break;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE:
			_set_space_override_mode(linear_damping_override_mode, (PhysicsServer2D::AreaSpaceOverrideMode)(int)p_value);
			break;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP: {
			real_t new_linear_damp = p_value;
			if (linear_damp != new_linear_damp) {
				linear_damp = new_linear_damp;
				if (linear_damping_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					// Update currently detected bodies
					if (!area_override_update_list.in_list() && get_space()) {
						get_space()->area_add_to_area_update_list(&area_override_update_list);
					}
				}
			}
		} break;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE:
			_set_space_override_mode(angular_damping_override_mode, (PhysicsServer2D::AreaSpaceOverrideMode)(int)p_value);
			break;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP: {
			real_t new_angular_damp = p_value;
			if (angular_damp != new_angular_damp) {
				angular_damp = new_angular_damp;
				if (angular_damping_override_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					// Update currently detected bodies
					if (!area_override_update_list.in_list() && get_space()) {
						get_space()->area_add_to_area_update_list(&area_override_update_list);
					}
				}
			}
		} break;
		case PhysicsServer2D::AREA_PARAM_PRIORITY: {
			int new_priority = p_value;
			if (priority != new_priority) {
				priority = p_value;
				if (has_any_space_override()) {
					// Need to re-process priority list for each body
					_reset_space_override();
				}
			}
		} break;
	}
}

Variant RapierArea2D::get_param(PhysicsServer2D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer2D::AREA_PARAM_GRAVITY_OVERRIDE_MODE:
			return gravity_override_mode;
		case PhysicsServer2D::AREA_PARAM_GRAVITY:
			return gravity;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_VECTOR:
			return gravity_vector;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_IS_POINT:
			return gravity_is_point;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE:
			return gravity_point_unit_distance;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE:
			return linear_damping_override_mode;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP:
			return linear_damp;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE:
			return angular_damping_override_mode;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP:
			return angular_damp;
		case PhysicsServer2D::AREA_PARAM_PRIORITY:
			return priority;
	}

	return Variant();
}

void RapierArea2D::set_monitorable(bool p_monitorable) {
	if (monitorable == p_monitorable) {
		return;
	}

	monitorable = p_monitorable;
}

void RapierArea2D::call_queries() {
	if (monitored_objects.is_empty()) {
		return;
	}

	static Array arg_array = []() {
		Array array;
		array.resize(5);
		return array;
	}();

	for (auto E = monitored_objects.begin(); E != monitored_objects.end(); ++E) {
		const MonitorInfo &monitor_info = E->value;
		ERR_CONTINUE(monitor_info.state == 0);

		arg_array[0] = monitor_info.state > 0 ? PhysicsServer2D::AREA_BODY_ADDED : PhysicsServer2D::AREA_BODY_REMOVED;
		arg_array[1] = monitor_info.rid;
		arg_array[2] = monitor_info.instance_id;
		arg_array[3] = monitor_info.object_shape_index;
		arg_array[4] = monitor_info.area_shape_index;

		if (monitor_info.type == Type::TYPE_BODY) {
			ERR_CONTINUE(monitor_callback.is_null());
			monitor_callback.callv(arg_array);
		} else {
			ERR_CONTINUE(area_monitor_callback.is_null());
			area_monitor_callback.callv(arg_array);
		}
	}

	monitored_objects.clear();
}

void RapierArea2D::compute_gravity(const Vector2 &p_position, Vector2 &r_gravity) const {
	if (is_gravity_point()) {
		const real_t gr_unit_dist = get_gravity_point_unit_distance();
		Vector2 v = get_transform().xform(get_gravity_vector()) - p_position;
		if (gr_unit_dist > 0) {
			const real_t v_length_sq = v.length_squared();
			if (v_length_sq > 0) {
				const real_t gravity_strength = get_gravity() * gr_unit_dist * gr_unit_dist / v_length_sq;
				r_gravity = v.normalized() * gravity_strength;
			} else {
				r_gravity = Vector2();
			}
		} else {
			r_gravity = v.normalized() * get_gravity();
		}
	} else {
		r_gravity = get_gravity_vector() * get_gravity();
	}
}

RapierArea2D::RapierArea2D() :
		RapierCollisionObject2D(TYPE_AREA),
		monitor_query_list(this),
		area_override_update_list(this) {
	_set_static(true); //areas are not active by default
}

RapierArea2D::~RapierArea2D() {
}
