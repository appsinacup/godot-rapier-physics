#include "rapier_body_2d.h"

#include "../spaces/rapier_space_2d.h"
#include "rapier_direct_body_state_2d.h"

void RapierBody2D::_mass_properties_changed() {
	if (!get_space()) {
		return;
	}

	if (mode < PhysicsServer2D::BODY_MODE_RIGID) {
		return;
	}

	if (calculate_inertia || calculate_center_of_mass) {
		if (!mass_properties_update_list.in_list()) {
			get_space()->body_add_to_mass_properties_update_list(&mass_properties_update_list);
		}
	} else {
		_apply_mass_properties();
	}
}

void RapierBody2D::_apply_mass_properties(bool force_update) {
	ERR_FAIL_COND(!get_space());

	ERR_FAIL_COND(mode < PhysicsServer2D::BODY_MODE_RIGID);
	real_t inertia_value = (mode == PhysicsServer2D::BODY_MODE_RIGID_LINEAR) ? 0.0 : inertia;

	rapier2d::Vector com = { center_of_mass.x, center_of_mass.y };

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));

	// Force update means local properties will be re-calculated internally,
	// it's needed for applying forces right away (otherwise it's updated on next step)
	rapier2d::body_set_mass_properties(space_handle, body_handle, mass, inertia_value, &com, false, force_update);
}

void RapierBody2D::update_mass_properties(bool force_update) {
	mass_properties_update_list.remove_from_list();
	if (mode < PhysicsServer2D::BODY_MODE_RIGID) {
		return;
	}

	real_t total_area = 0;
	for (int i = 0; i < get_shape_count(); i++) {
		if (is_shape_disabled(i)) {
			continue;
		}
		total_area += get_shape(i)->get_aabb().get_area();
	}

	if (calculate_center_of_mass) {
		center_of_mass = Vector2();

		if (total_area != 0.0) {
			for (int i = 0; i < get_shape_count(); i++) {
				if (is_shape_disabled(i)) {
					continue;
				}

				const RapierShape2D *shape = get_shape(i);

				real_t shape_area = shape->get_aabb().get_area();
				if (shape_area == 0.0 || mass == 0.0) {
					continue;
				}

				real_t shape_mass = shape_area * mass / total_area;

				// NOTE: we assume that the shape origin is also its center of mass.
				center_of_mass += shape_mass * get_shape_transform(i).get_origin();
			}

			center_of_mass /= mass;
		}
	}

	if (calculate_inertia) {
		inertia = 0.0;

		if (total_area != 0.0) {
			for (int i = 0; i < get_shape_count(); i++) {
				if (is_shape_disabled(i)) {
					continue;
				}

				const RapierShape2D *shape = get_shape(i);

				real_t shape_area = shape->get_aabb().get_area();
				if (shape_area == 0.0 || mass == 0.0) {
					continue;
				}

				real_t shape_mass = shape_area * mass / total_area;

				Transform2D mtx = get_shape_transform(i);
				Vector2 scale = mtx.get_scale();
				Vector2 shape_origin = mtx.get_origin() - center_of_mass;
				inertia += shape->get_moment_of_inertia(shape_mass, scale) + shape_mass * shape_origin.length_squared();
			}
		}
	}

	_apply_mass_properties(force_update);
}

void RapierBody2D::reset_mass_properties() {
	if (calculate_inertia && calculate_center_of_mass) {
		// Nothing to do, already calculated
		return;
	}

	calculate_inertia = true;
	calculate_center_of_mass = true;
	_mass_properties_changed();
}

void RapierBody2D::set_active(bool p_active) {
	if (active == p_active) {
		return;
	}

	active = p_active;

	if (active) {
		if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
			// Static bodies can't be active.
			active = false;
		} else if (get_space()) {
			get_space()->body_add_to_active_list(&active_list);
		}
	} else if (get_space()) {
		ERR_FAIL_COND(marked_active);
		active_list.remove_from_list();
	}
}

void RapierBody2D::set_can_sleep(bool p_can_sleep) {
	ERR_FAIL_COND(!get_space());

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	rapier2d::body_set_can_sleep(space_handle, body_handle, p_can_sleep);
}

void RapierBody2D::on_marked_active() {
	ERR_FAIL_COND(marked_active);
	ERR_FAIL_COND(!get_space());

	// TODO: Check how that happens, probably not good for performance
	//ERR_FAIL_COND(mode == PhysicsServer2D::BODY_MODE_STATIC);
	if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
		return;
	}

	marked_active = true;
	if (!active) {
		active = true;
		get_space()->body_add_to_active_list(&active_list);
	} else {
		ERR_FAIL_COND(!active_list.in_list());
	}
}

void RapierBody2D::on_update_active() {
	if (!marked_active) {
		set_active(false);
		return;
	}
	marked_active = false;

	_update_transform();

	if (!direct_state_query_list.in_list()) {
		get_space()->body_add_to_state_query_list(&direct_state_query_list);
	}
	if (mode >= PhysicsServer2D::BODY_MODE_RIGID) {
		if (to_add_angular_velocity != 0.0) {
			set_angular_velocity(to_add_angular_velocity);
			to_add_angular_velocity = 0.0;
		}
		if (to_add_linear_velocity != Vector2()) {
			set_linear_velocity(to_add_linear_velocity);
			to_add_linear_velocity = Vector2();
		}
	}
}

void RapierBody2D::set_param(PhysicsServer2D::BodyParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer2D::BODY_PARAM_BOUNCE:
		case PhysicsServer2D::BODY_PARAM_FRICTION: {
			if (p_param == PhysicsServer2D::BODY_PARAM_BOUNCE) {
				bounce = p_value;
			} else {
				friction = p_value;
			}

			if (!get_space()) {
				return;
			}

			rapier2d::Handle space_handle = get_space()->get_handle();
			ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
			ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
			rapier2d::Material mat;
			mat.friction = friction;
			mat.restitution = bounce;
			rapier2d::body_update_material(space_handle, body_handle, &mat);
		} break;
		case PhysicsServer2D::BODY_PARAM_MASS: {
			real_t mass_value = p_value;
			ERR_FAIL_COND(mass_value <= 0);
			mass = mass_value;
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID) {
				_mass_properties_changed();
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_INERTIA: {
			real_t inertia_value = p_value;
			if (inertia_value <= 0.0) {
				calculate_inertia = true;
			} else {
				calculate_inertia = false;
				inertia = inertia_value;
			}
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID) {
				_mass_properties_changed();
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_CENTER_OF_MASS: {
			center_of_mass = p_value;
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID) {
				_mass_properties_changed();
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_GRAVITY_SCALE: {
			real_t new_gravity_scale = p_value;
			if (gravity_scale != new_gravity_scale) {
				gravity_scale = new_gravity_scale;
				if (!using_area_gravity) {
					_apply_gravity_scale(gravity_scale);
				}
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_LINEAR_DAMP_MODE: {
			int mode_value = p_value;
			if (linear_damping_mode != mode_value) {
				linear_damping_mode = (PhysicsServer2D::BodyDampMode)mode_value;
				if (linear_damping_mode == PhysicsServer2D::BODY_DAMP_MODE_REPLACE) {
					using_area_linear_damping = false;
				}
				if (using_area_linear_damping) {
					// Update linear damping from areas
				} else {
					_apply_linear_damping(linear_damping);
				}
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_ANGULAR_DAMP_MODE: {
			int mode_value = p_value;
			if (angular_damping_mode != mode_value) {
				angular_damping_mode = (PhysicsServer2D::BodyDampMode)mode_value;
				if (angular_damping_mode == PhysicsServer2D::BODY_DAMP_MODE_REPLACE) {
					using_area_angular_damping = false;
				}
				if (using_area_angular_damping) {
					// Update angular damping from areas
				} else {
					_apply_angular_damping(angular_damping);
				}
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_LINEAR_DAMP: {
			real_t new_value = p_value;
			if (new_value != linear_damping) {
				linear_damping = new_value;
				if (!using_area_linear_damping) {
					_apply_linear_damping(linear_damping);
				}
			}
		} break;
		case PhysicsServer2D::BODY_PARAM_ANGULAR_DAMP: {
			real_t new_value = p_value;
			if (new_value != angular_damping) {
				angular_damping = new_value;
				if (!using_area_angular_damping) {
					_apply_angular_damping(angular_damping);
				}
			}
		} break;
		default: {
		}
	}
}

Variant RapierBody2D::get_param(PhysicsServer2D::BodyParameter p_param) const {
	switch (p_param) {
		case PhysicsServer2D::BODY_PARAM_BOUNCE: {
			return bounce;
		}
		case PhysicsServer2D::BODY_PARAM_FRICTION: {
			return friction;
		}
		case PhysicsServer2D::BODY_PARAM_MASS: {
			return mass;
		}
		case PhysicsServer2D::BODY_PARAM_INERTIA: {
			return inertia;
		}
		case PhysicsServer2D::BODY_PARAM_CENTER_OF_MASS: {
			return center_of_mass;
		}
		case PhysicsServer2D::BODY_PARAM_GRAVITY_SCALE: {
			return gravity_scale;
		}
		case PhysicsServer2D::BODY_PARAM_LINEAR_DAMP_MODE: {
			return linear_damping_mode;
		}
		case PhysicsServer2D::BODY_PARAM_ANGULAR_DAMP_MODE: {
			return angular_damping_mode;
		}
		case PhysicsServer2D::BODY_PARAM_LINEAR_DAMP: {
			return linear_damping;
		}
		case PhysicsServer2D::BODY_PARAM_ANGULAR_DAMP: {
			return angular_damping;
		}
		default: {
		}
	}

	return 0;
}

void RapierBody2D::set_mode(PhysicsServer2D::BodyMode p_mode) {
	if (mode == p_mode) {
		return;
	}
	PhysicsServer2D::BodyMode prev_mode = mode;
	mode = p_mode;
	if (get_space()) {
		switch (p_mode) {
			case PhysicsServer2D::BODY_MODE_KINEMATIC: {
				rapier2d::body_change_mode(get_space()->get_handle(), get_body_handle(), rapier2d::BodyType::Kinematic, true);
			} break;
			case PhysicsServer2D::BODY_MODE_STATIC: {
				rapier2d::body_change_mode(get_space()->get_handle(), get_body_handle(), rapier2d::BodyType::Static, true);
			} break;
			case PhysicsServer2D::BODY_MODE_RIGID:
			case PhysicsServer2D::BODY_MODE_RIGID_LINEAR: {
				rapier2d::body_change_mode(get_space()->get_handle(), get_body_handle(), rapier2d::BodyType::Dynamic, true);
			} break;
		}
	}
	if (p_mode == PhysicsServer2D::BODY_MODE_STATIC) {
		force_sleep();

		ERR_FAIL_COND(marked_active);
		active_list.remove_from_list();
		mass_properties_update_list.remove_from_list();
		gravity_update_list.remove_from_list();
		area_override_update_list.remove_from_list();

		return;
	}

	if (active && (prev_mode == PhysicsServer2D::BODY_MODE_STATIC)) {
		if (get_space()) {
			get_space()->body_add_to_active_list(&active_list);
		}
	}

	if (p_mode >= PhysicsServer2D::BODY_MODE_RIGID) {
		_mass_properties_changed();

		if (get_space()) {
			update_area_override();

			_apply_gravity_scale(gravity_scale);
		}
	}
}

PhysicsServer2D::BodyMode RapierBody2D::get_mode() const {
	return mode;
}

void RapierBody2D::_shapes_changed() {
	_mass_properties_changed();
	wakeup();
}

void RapierBody2D::set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			set_transform(p_variant, true);
		} break;
		case PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY: {
			set_linear_velocity(p_variant);
		} break;
		case PhysicsServer2D::BODY_STATE_ANGULAR_VELOCITY: {
			set_angular_velocity(p_variant);
		} break;
		case PhysicsServer2D::BODY_STATE_SLEEPING: {
			if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
				break;
			}
			sleep = p_variant;

			if (sleep) {
				if (can_sleep) {
					force_sleep();
					set_active(false);
				}
			} else if (mode != PhysicsServer2D::BODY_MODE_STATIC) {
				wakeup();
				set_active(true);
			}
		} break;
		case PhysicsServer2D::BODY_STATE_CAN_SLEEP: {
			can_sleep = p_variant;
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID) {
				if (get_space()) {
					set_can_sleep(can_sleep);
				}
				if (!active && !can_sleep) {
					wakeup();
					set_active(true);
				}
			}
		} break;
	}
}

Variant RapierBody2D::get_state(PhysicsServer2D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			return get_transform();
		}
		case PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY: {
			return get_linear_velocity();
		}
		case PhysicsServer2D::BODY_STATE_ANGULAR_VELOCITY: {
			return get_angular_velocity();
		}
		case PhysicsServer2D::BODY_STATE_SLEEPING: {
			return !active;
		}
		case PhysicsServer2D::BODY_STATE_CAN_SLEEP: {
			return can_sleep;
		}
	}

	return Variant();
}

void RapierBody2D::set_continuous_collision_detection_mode(PhysicsServer2D::CCDMode p_mode) {
	if (ccd_mode == p_mode) {
		return;
	}

	ERR_FAIL_COND(mode < PhysicsServer2D::BODY_MODE_RIGID);

	ccd_mode = p_mode;

	if (get_space()) {
		rapier2d::Handle space_handle = get_space()->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

		rapier2d::body_set_ccd_enabled(space_handle, body_handle, ccd_mode != PhysicsServer2D::CCD_MODE_DISABLED);
	}
}

void RapierBody2D::_init_material(rapier2d::Material &mat) const {
	mat.friction = friction;
	mat.restitution = bounce;
}

void RapierBody2D::_init_collider(rapier2d::Handle collider_handle) const {
	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	// Send contact infos for dynamic bodies
	if (mode >= PhysicsServer2D::BODY_MODE_KINEMATIC) {
#ifdef DEBUG_ENABLED
		// Always send contacts in case debug contacts is enabled
		bool send_contacts = true;
#else
		// Only send contacts if contact monitor is enabled
		bool send_contacts = can_report_contacts();
#endif
		rapier2d::collider_set_contact_force_events_enabled(space_handle, collider_handle, send_contacts);
	}
}

/**
* Create or remove a body in the simulation.
* @Param p_space: if null, it removes the body from the simulation in RapierCollisionObject2D::_set_space,
	else add it.
*/
void RapierBody2D::set_space(RapierSpace2D *p_space) {
	if (p_space == get_space()) {
		return;
	}

	// Reset properties if the body is already in a simulation
	if (get_space()) {
		// wakeup_neighbours();

		ERR_FAIL_COND(marked_active);

		mass_properties_update_list.remove_from_list();
		gravity_update_list.remove_from_list();
		active_list.remove_from_list();
		direct_state_query_list.remove_from_list();
		area_override_update_list.remove_from_list();
	}

	_set_space(p_space);

	if (get_space()) {
		if (mode >= PhysicsServer2D::BODY_MODE_KINEMATIC) {
			if (!can_sleep) {
				set_can_sleep(false);
			}

			if (active || !sleep) {
				wakeup();
				get_space()->body_add_to_active_list(&active_list);
			} else if (can_sleep && sleep) {
				force_sleep();
			}
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID) {
				_apply_gravity_scale(gravity_scale);
				if (linear_damping_mode == PhysicsServer2D::BODY_DAMP_MODE_COMBINE) {
					_apply_linear_damping(linear_damping);
				}
				if (angular_damping_mode == PhysicsServer2D::BODY_DAMP_MODE_COMBINE) {
					_apply_angular_damping(angular_damping);
				}

				_mass_properties_changed();
				if (linear_velocity != Vector2()) {
					set_linear_velocity(linear_velocity);
				}
				if (angular_velocity != 0.0) {
					set_angular_velocity(angular_velocity);
				}
				if (constant_force != Vector2()) {
					set_constant_force(constant_force);
				}
				if (constant_torque != 0.0) {
					set_constant_torque(constant_torque);
				}
				if (impulse != Vector2()) {
					apply_impulse(impulse);
				}
				if (torque != 0.0) {
					apply_torque_impulse(torque);
				}
				rapier2d::Handle space_handle = get_space()->get_handle();
				rapier2d::Material mat;
				mat.friction = friction;
				mat.restitution = bounce;
				rapier2d::body_update_material(space_handle, body_handle, &mat);
			}
		}
	}
}
// Synchronize Rapier and Godot with the direct state.
void RapierBody2D::call_queries() {
	Variant direct_state_variant = get_direct_state();

	// If a custom behaviour is set.
	if (fi_callback_data) {
		if (!fi_callback_data->callable.get_object()) {
			set_force_integration_callback(Callable());
		} else {
			static Array arg_array = []() {
				Array array;
				array.resize(2);
				return array;
			}();

			arg_array[0] = get_direct_state();
			arg_array[1] = fi_callback_data->udata;

			body_state_callback.callv(arg_array);
		}
	}

	//  Sync body server with Godot by sending body direct state
	if (body_state_callback.get_object()) {
		static Array arg_array = []() {
			Array array;
			array.resize(1);
			return array;
		}();

		arg_array[0] = get_direct_state();

		body_state_callback.callv(arg_array);
	}

	if (!active) {
		direct_state_query_list.remove_from_list();
	}
}

void RapierBody2D::set_state_sync_callback(const Callable &p_callable) {
	body_state_callback = p_callable;
}

void RapierBody2D::set_force_integration_callback(const Callable &p_callable, const Variant &p_udata) {
	if (p_callable.get_object()) {
		if (!fi_callback_data) {
			fi_callback_data = memnew(ForceIntegrationCallbackData);
		}
		fi_callback_data->callable = p_callable;
		fi_callback_data->udata = p_udata;
	} else if (fi_callback_data) {
		memdelete(fi_callback_data);
		fi_callback_data = nullptr;
	}
}

void RapierBody2D::apply_central_impulse(const Vector2 &p_impulse) {
	impulse += p_impulse;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector impulse = { p_impulse.x, p_impulse.y };
	rapier2d::body_apply_impulse(space_handle, body_handle, &impulse);
}

void RapierBody2D::apply_impulse(const Vector2 &p_impulse, const Vector2 &p_position) {
	impulse += p_impulse;
	torque += (p_position - get_center_of_mass()).cross(p_impulse);
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector impulse = { p_impulse.x, p_impulse.y };
	rapier2d::Vector pos = { p_position.x, p_position.y };
	rapier2d::body_apply_impulse_at_point(space_handle, body_handle, &impulse, &pos);
}

void RapierBody2D::apply_torque_impulse(real_t p_torque) {
	torque += p_torque;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_apply_torque_impulse(space_handle, body_handle, p_torque);
}

void RapierBody2D::apply_central_force(const Vector2 &p_force) {
	// Note: using last delta assuming constant physics time
	real_t last_delta = get_space()->get_last_step();
	impulse += p_force * last_delta;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector force = { p_force.x * last_delta, p_force.y * last_delta };
	rapier2d::body_apply_impulse(space_handle, body_handle, &force);
}

void RapierBody2D::apply_force(const Vector2 &p_force, const Vector2 &p_position) {
	// Note: using last delta assuming constant physics time
	real_t last_delta = get_space()->get_last_step();
	impulse += p_force * last_delta;
	torque += (p_position - get_center_of_mass()).cross(p_force) * last_delta;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector force = { p_force.x * last_delta, p_force.y * last_delta };
	rapier2d::Vector pos = { p_position.x, p_position.y };
	rapier2d::body_apply_impulse_at_point(space_handle, body_handle, &force, &pos);
}

void RapierBody2D::apply_torque(real_t p_torque) {
	// Note: using last delta assuming constant physics time
	real_t last_delta = get_space()->get_last_step();
	torque += p_torque * last_delta;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_apply_torque_impulse(space_handle, body_handle, p_torque * last_delta);
}

void RapierBody2D::add_constant_central_force(const Vector2 &p_force) {
	constant_force += p_force;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector force = { p_force.x, p_force.y };
	rapier2d::body_add_force(space_handle, body_handle, &force);
}

void RapierBody2D::add_constant_force(const Vector2 &p_force, const Vector2 &p_position) {
	constant_torque += (p_position - get_center_of_mass()).cross(p_force);
	constant_force += p_force;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector force = { p_force.x, p_force.y };
	rapier2d::Vector pos = { p_position.x, p_position.y };
	rapier2d::body_add_force_at_point(space_handle, body_handle, &force, &pos);
}

void RapierBody2D::add_constant_torque(real_t p_torque) {
	constant_torque += p_torque;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_add_torque(space_handle, body_handle, p_torque);
}

void RapierBody2D::set_constant_force(const Vector2 &p_force) {
	constant_force = p_force;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector force = { p_force.x, p_force.y };
	rapier2d::body_reset_forces(space_handle, body_handle);
	rapier2d::body_add_force(space_handle, body_handle, &force);
}

Vector2 RapierBody2D::get_constant_force() const {
	if (!get_space()) {
		return constant_force;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), Vector2());

	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(body_handle), Vector2());

	rapier2d::Vector force = rapier2d::body_get_constant_force(space_handle, body_handle);

	return Vector2(force.x, force.y);
}

void RapierBody2D::set_constant_torque(real_t p_torque) {
	constant_torque = p_torque;
	if (!get_space()) {
		return;
	}

	if (mass_properties_update_list.in_list()) {
		// Force update internal mass properties to calculate proper impulse
		update_mass_properties(true);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_reset_torques(space_handle, body_handle);
	rapier2d::body_add_torque(space_handle, body_handle, p_torque);
}

real_t RapierBody2D::get_constant_torque() const {
	if (!get_space()) {
		return constant_torque;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), 0.0);

	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(body_handle), 0.0);

	return rapier2d::body_get_constant_torque(space_handle, body_handle);
}

void RapierBody2D::wakeup() {
	sleep = false;
	if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
		return;
	}

	if (!get_space()) {
		return;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_wake_up(space_handle, body_handle, true);
}

void RapierBody2D::force_sleep() {
	sleep = true;
	if (!get_space()) {
		return;
	}

	ERR_FAIL_COND(!can_sleep);

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_force_sleep(space_handle, body_handle);
}

RapierDirectBodyState2D *RapierBody2D::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(RapierDirectBodyState2D);
		direct_state->body = this;
	}
	return direct_state;
}

void RapierBody2D::add_area(RapierArea2D *p_area) {
	++area_detection_counter;
	if (p_area->has_any_space_override()) {
		areas.ordered_insert(AreaCMP(p_area));
		on_area_updated(p_area);
	}
}

void RapierBody2D::remove_area(RapierArea2D *p_area) {
	ERR_FAIL_COND(area_detection_counter == 0);
	--area_detection_counter;
	if (p_area->has_any_space_override()) {
		int index = areas.find(AreaCMP(p_area));
		ERR_FAIL_COND(index < 0);
		areas.remove_at(index);
		on_area_updated(p_area);
	}
}

void RapierBody2D::on_area_updated(RapierArea2D *p_area) {
	// TODO: Check if uses any override (damp mode vs. area damp override)

	if (!area_override_update_list.in_list()) {
		get_space()->body_add_to_area_update_list(&area_override_update_list);
	}
}

void RapierBody2D::set_linear_velocity(const Vector2 &p_linear_velocity) {
	linear_velocity = p_linear_velocity;
	if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
		return;
	}

	if (!get_space()) {
		return;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	// TODO: apply delta
	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector velocity = { linear_velocity.x, linear_velocity.y };
	rapier2d::body_set_linear_velocity(space_handle, body_handle, &velocity);
}

Vector2 RapierBody2D::get_linear_velocity() const {
	if (!get_space()) {
		return linear_velocity;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), Vector2());

	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(body_handle), Vector2());
	rapier2d::Vector vel = rapier2d::body_get_linear_velocity(space_handle, body_handle);
	return Vector2(vel.x, vel.y);
}

Vector2 RapierBody2D::get_static_linear_velocity() const {
	return linear_velocity;
}

void RapierBody2D::set_angular_velocity(real_t p_angular_velocity) {
	angular_velocity = p_angular_velocity;
	if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
		return;
	}

	if (!get_space()) {
		return;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	// TODO: apply delta
	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_set_angular_velocity(space_handle, body_handle, angular_velocity);
}

real_t RapierBody2D::get_angular_velocity() const {
	if (!get_space()) {
		return angular_velocity;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(space_handle), 0.0f);

	ERR_FAIL_COND_V(!rapier2d::is_handle_valid(body_handle), 0.0f);
	return rapier2d::body_get_angular_velocity(space_handle, body_handle);
}

real_t RapierBody2D::get_static_angular_velocity() const {
	return angular_velocity;
}

void RapierBody2D::_apply_linear_damping(real_t new_value, bool apply_default) {
	if (!get_space()) {
		return;
	}

	// Combine damping with default value from space.
	total_linear_damping = new_value;
	if (apply_default) {
		total_linear_damping += (real_t)get_space()->get_default_area_param(PhysicsServer2D::AREA_PARAM_LINEAR_DAMP);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_set_linear_damping(space_handle, body_handle, total_linear_damping);
}

void RapierBody2D::_apply_angular_damping(real_t new_value, bool apply_default) {
	if (!get_space()) {
		return;
	}

	// Combine damping with default value from space.
	total_angular_damping = new_value;
	if (apply_default) {
		total_angular_damping += (real_t)get_space()->get_default_area_param(PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP);
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_set_angular_damping(space_handle, body_handle, total_angular_damping);
}

void RapierBody2D::_apply_gravity_scale(real_t new_value) {
	if (!get_space()) {
		return;
	}

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::body_set_gravity_scale(space_handle, body_handle, new_value, true);
}

void RapierBody2D::update_area_override() {
	area_override_update_list.remove_from_list();

	if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
		return;
	}

	ERR_FAIL_COND(!get_space());

	// Reset area override flags.
	using_area_gravity = false;
	using_area_linear_damping = false;
	using_area_angular_damping = false;

	// Start with no effect.
	total_gravity = Vector2();
	real_t total_linear_damping = 0.0;
	real_t total_angular_damping = 0.0;

	// Combine gravity and damping from overlapping areas in priority order.
	int ac = areas.size();
	bool gravity_done = false; // always calculate to be able to change scale on area gravity
	bool linear_damping_done = (linear_damping_mode == PhysicsServer2D::BODY_DAMP_MODE_REPLACE);
	bool angular_damping_done = (angular_damping_mode == PhysicsServer2D::BODY_DAMP_MODE_REPLACE);
	if (ac > 0) {
		const AreaCMP *aa = &areas[0];
		for (int i = ac - 1; i >= 0; i--) {
			if (!gravity_done) {
				PhysicsServer2D::AreaSpaceOverrideMode area_gravity_mode = (PhysicsServer2D::AreaSpaceOverrideMode)(int)aa[i].area->get_param(PhysicsServer2D::AREA_PARAM_GRAVITY_OVERRIDE_MODE);
				if (area_gravity_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					Vector2 area_gravity;
					aa[i].area->compute_gravity(get_transform().get_origin(), area_gravity);
					switch (area_gravity_mode) {
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE:
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
							using_area_gravity = true;
							total_gravity += area_gravity;
							gravity_done = area_gravity_mode == PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE;
						} break;
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE:
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
							using_area_gravity = true;
							total_gravity = area_gravity;
							gravity_done = area_gravity_mode == PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE;
						} break;
						default: {
						}
					}
				}
			}
			if (!linear_damping_done) {
				PhysicsServer2D::AreaSpaceOverrideMode area_linear_damping_mode = (PhysicsServer2D::AreaSpaceOverrideMode)(int)aa[i].area->get_param(PhysicsServer2D::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE);
				if (area_linear_damping_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					real_t area_linear_damping = aa[i].area->get_linear_damp();
					switch (area_linear_damping_mode) {
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE:
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
							using_area_linear_damping = true;
							total_linear_damping += area_linear_damping;
							linear_damping_done = area_linear_damping_mode == PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE;
						} break;
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE:
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
							using_area_linear_damping = true;
							total_linear_damping = area_linear_damping;
							linear_damping_done = area_linear_damping_mode == PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE;
						} break;
						default: {
						}
					}
				}
			}
			if (!angular_damping_done) {
				PhysicsServer2D::AreaSpaceOverrideMode area_angular_damping_mode = (PhysicsServer2D::AreaSpaceOverrideMode)(int)aa[i].area->get_param(PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE);
				if (area_angular_damping_mode != PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED) {
					real_t area_angular_damping = aa[i].area->get_angular_damp();
					switch (area_angular_damping_mode) {
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE:
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
							using_area_angular_damping = true;
							total_angular_damping += area_angular_damping;
							angular_damping_done = area_angular_damping_mode == PhysicsServer2D::AREA_SPACE_OVERRIDE_COMBINE_REPLACE;
						} break;
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE:
						case PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
							using_area_angular_damping = true;
							total_angular_damping = area_angular_damping;
							angular_damping_done = area_angular_damping_mode == PhysicsServer2D::AREA_SPACE_OVERRIDE_REPLACE;
						} break;
						default: {
						}
					}
				}
			}
			if (gravity_done && linear_damping_done && angular_damping_done) {
				break;
			}
		}
	}

	// Override or combine damping with body's values.
	total_linear_damping += linear_damping;
	total_angular_damping += angular_damping;

	// Apply to the simulation.
	_apply_linear_damping(total_linear_damping, !linear_damping_done);
	_apply_angular_damping(total_angular_damping, !angular_damping_done);

	if (using_area_gravity) {
		// Add default gravity from space.
		if (!gravity_done) {
			total_gravity += get_space()->get_default_area_param(PhysicsServer2D::AREA_PARAM_GRAVITY);
		}

		// Apply gravity scale to computed value.
		total_gravity *= gravity_scale;

		// Disable simulation gravity and apply it manually instead.
		_apply_gravity_scale(0.0);
		if (!gravity_update_list.in_list()) {
			get_space()->body_add_to_gravity_update_list(&gravity_update_list);
		}
	} else {
		// Enable simulation gravity.
		_apply_gravity_scale(gravity_scale);
		gravity_update_list.remove_from_list();
	}
}

void RapierBody2D::update_gravity(real_t p_step) {
	ERR_FAIL_COND(!using_area_gravity);
	ERR_FAIL_COND(!get_space());

	Vector2 gravity_impulse = total_gravity * mass * p_step;

	rapier2d::Handle space_handle = get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));
	rapier2d::Vector impulse = { gravity_impulse.x, gravity_impulse.y };
	rapier2d::body_apply_impulse(space_handle, body_handle, &impulse);
}

Rect2 RapierBody2D::get_aabb() {
	bool shapes_found = false;
	Rect2 body_aabb;
	for (int i = 0; i < get_shape_count(); ++i) {
		if (is_shape_disabled(i)) {
			continue;
		}
		if (!shapes_found) {
			// TODO not 100% correct, we don't take into consideration rotation here.
			body_aabb = get_shape(i)->get_aabb(get_shape_transform(i).get_origin());
			shapes_found = true;
		} else {
			// TODO not 100% correct, we don't take into consideration rotation here.
			body_aabb = body_aabb.merge(get_shape(i)->get_aabb(get_shape_transform(i).get_origin()));
		}
	}
	return body_aabb;
}

RapierBody2D::RapierBody2D() :
		RapierCollisionObject2D(TYPE_BODY),
		active_list(this),
		mass_properties_update_list(this),
		gravity_update_list(this),
		area_override_update_list(this),
		direct_state_query_list(this) {
}

RapierBody2D::~RapierBody2D() {
	if (fi_callback_data) {
		memdelete(fi_callback_data);
	}
	if (direct_state) {
		memdelete(direct_state);
	}
}
