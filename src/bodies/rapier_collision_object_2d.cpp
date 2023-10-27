#include "rapier_collision_object_2d.h"

#include "../servers/rapier_physics_server_2d.h"
#include "../spaces/rapier_space_2d.h"

void RapierCollisionObject2D::add_shape(RapierShape2D *p_shape, const Transform2D &p_transform, bool p_disabled) {
	Shape shape;
	shape.shape = p_shape;
	shape.xform = p_transform;
	shape.disabled = p_disabled;
	shape.one_way_collision = false;
	shape.one_way_collision_margin = 0;

	if (!shape.disabled) {
		_create_shape(shape, shapes.size());
		_update_shape_transform(shape);
	}

	shapes.push_back(shape);
	p_shape->add_owner(this);

	//if (!pending_shape_update_list.in_list()) {
	//	RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void RapierCollisionObject2D::set_shape(int p_index, RapierShape2D *p_shape) {
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Shape &shape = shapes[p_index];

	_destroy_shape(shape, p_index);

	shape.shape->remove_owner(this);
	shape.shape = p_shape;

	p_shape->add_owner(this);

	if (!shape.disabled) {
		_create_shape(shape, p_index);
		_update_shape_transform(shape);
	}

	//if (!pending_shape_update_list.in_list()) {
	//	RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void RapierCollisionObject2D::set_shape_transform(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Shape &shape = shapes[p_index];
	shape.xform = p_transform;

	_update_shape_transform(shape);

	//if (!pending_shape_update_list.in_list()) {
	//	RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void RapierCollisionObject2D::set_shape_disabled(int p_index, bool p_disabled) {
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	RapierCollisionObject2D::Shape &shape = shapes[p_index];
	if (shape.disabled == p_disabled) {
		return;
	}

	shape.disabled = p_disabled;

	if (shape.disabled) {
		_destroy_shape(shape, p_index);
	} else {
		_create_shape(shape, p_index);
		_update_shape_transform(shape);
	}

	// if (p_disabled && shape.bpid != 0) {
	// 	space->get_broadphase()->remove(shape.bpid);
	// 	shape.bpid = 0;
	// 	if (!pending_shape_update_list.in_list()) {
	// 		RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	// 	}
	// } else if (!p_disabled && shape.bpid == 0) {
	// 	if (!pending_shape_update_list.in_list()) {
	// 		RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	// 	}
	// }

	//if (!pending_shape_update_list.in_list()) {
	//	RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void RapierCollisionObject2D::remove_shape(RapierShape2D *p_shape) {
	//remove a shape, all the times it appears
	for (uint32_t i = 0; i < shapes.size(); i++) {
		if (shapes[i].shape == p_shape) {
			remove_shape(i);
			i--;
		}
	}
}

void RapierCollisionObject2D::remove_shape(int p_index) {
	//remove anything from shape to be erased to end, so subindices don't change
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Shape &shape = shapes[p_index];

	if (!shape.disabled) {
		_destroy_shape(shape, p_index);
	}

	shape.shape->remove_owner(this);
	shapes.remove_at(p_index);

	//if (!pending_shape_update_list.in_list()) {
	//	RapierPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void RapierCollisionObject2D::_set_static(bool p_static) {
	if (_static == p_static) {
		return;
	}
	_static = p_static;

	if (!space) {
		return;
	}
}

void RapierCollisionObject2D::_unregister_shapes() {
}

void RapierCollisionObject2D::_update_transform() {
	if (!space) {
		return;
	}

	rapier2d::Handle space_handle = space->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));

	rapier2d::Vector position = rapier2d::body_get_position(space_handle, body_handle);
	real_t angle = rapier2d::body_get_angle(space_handle, body_handle);

	transform.set_origin(Vector2(position.x, position.y));
	transform.set_rotation(angle);

	inv_transform = transform.affine_inverse();
}

void RapierCollisionObject2D::set_transform(const Transform2D &p_transform, bool wake_up) {
	transform = p_transform;
	inv_transform = transform.affine_inverse();

	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

		ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));

		const Vector2 &origin = transform.get_origin();
		rapier2d::Vector position = { origin.x, origin.y };
		real_t rotation = transform.get_rotation();

		rapier2d::body_set_transform(space_handle, body_handle, &position, rotation, wake_up);
	}
}

void RapierCollisionObject2D::_create_shape(Shape &shape, uint32_t p_shape_index) {
	if (!space) {
		return;
	}

	rapier2d::Handle space_handle = space->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(rapier2d::is_handle_valid(shape.collider_handle));

	rapier2d::Material mat = rapier2d::default_material();
	_init_material(mat);

	rapier2d::Handle shape_handle = shape.shape->get_rapier_shape();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(shape_handle));

	rapier2d::UserData user_data;
	set_collider_user_data(user_data, p_shape_index);

	switch (type) {
		case TYPE_BODY: {
			shape.collider_handle = rapier2d::collider_create_solid(space_handle, shape_handle, &mat, body_handle, &user_data);
		} break;
		case TYPE_AREA: {
			shape.collider_handle = rapier2d::collider_create_sensor(space_handle, shape_handle, body_handle, &user_data);
		} break;
	}

	ERR_FAIL_COND(!rapier2d::is_handle_valid(shape.collider_handle));
	_init_collider(shape.collider_handle);
}

void RapierCollisionObject2D::_destroy_shape(Shape &shape, uint32_t p_shape_index) {
	if (!space) {
		return;
	}

	rapier2d::Handle space_handle = space->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(shape.collider_handle));

	if (area_detection_counter > 0) {
		// Keep track of body information for delayed removal
		space->add_removed_collider(shape.collider_handle, this, p_shape_index);
	}

	rapier2d::collider_destroy(space_handle, shape.collider_handle);
	shape.collider_handle = rapier2d::invalid_handle(); // collider_handle = rapier ID
}

void RapierCollisionObject2D::_update_shape_transform(const Shape &shape) {
	if (!space) {
		return;
	}

	rapier2d::Handle space_handle = space->get_handle();

	const Vector2 &origin = shape.xform.get_origin();
	rapier2d::Vector position = { origin.x, origin.y };
	real_t angle = shape.xform.get_rotation();

	shape.shape->apply_rapier_transform(position, angle);
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!rapier2d::is_handle_valid(shape.collider_handle));
	ERR_FAIL_COND(!rapier2d::is_handle_valid(shape.shape->get_rapier_shape()));
	rapier2d::ShapeInfo shape_info{
		shape.shape->get_rapier_shape(),
		position,
		angle,
		rapier2d::Vector{ shape.xform.get_scale().x, shape.xform.get_scale().y }
	};
	rapier2d::collider_set_transform(space_handle, shape.collider_handle, shape_info);
}

void RapierCollisionObject2D::_set_space(RapierSpace2D *p_space) {
	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

		ERR_FAIL_COND(!rapier2d::is_handle_valid(body_handle));

		// This call also destroys the colliders
		rapier2d::body_destroy(space_handle, body_handle);
		body_handle = rapier2d::invalid_handle();

		for (uint32_t i = 0; i < shapes.size(); i++) {
			Shape &shape = shapes[i];
			if (shape.disabled) {
				continue;
			}

			_destroy_shape(shape, i);
		}

		// Reset area detection counter to keep it consistent for new detections
		area_detection_counter = 0;
	}

	space = p_space;

	if (space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

		ERR_FAIL_COND(rapier2d::is_handle_valid(body_handle));

		rapier2d::UserData user_data;
		set_body_user_data(user_data);

		rapier2d::Vector position = { transform.get_origin().x, transform.get_origin().y };
		real_t angle = transform.get_rotation();

		if (_static) {
			body_handle = rapier2d::body_create_fixed(space_handle, &position, angle, &user_data);
		} else {
			body_handle = rapier2d::body_create_dynamic(space_handle, &position, angle, &user_data);
		}

		for (uint32_t i = 0; i < shapes.size(); i++) {
			Shape &shape = shapes[i];
			if (shape.disabled) {
				continue;
			}

			_create_shape(shape, i);
			_update_shape_transform(shape);
		}
	}
}

void RapierCollisionObject2D::set_body_user_data(rapier2d::UserData &r_user_data) const {
	r_user_data.part1 = (uint64_t)this;
}

RapierCollisionObject2D *RapierCollisionObject2D::get_body_user_data(const rapier2d::UserData &p_user_data) {
	return (RapierCollisionObject2D *)p_user_data.part1;
}

void RapierCollisionObject2D::set_collider_user_data(rapier2d::UserData &r_user_data, uint32_t p_shape_index) const {
	r_user_data.part1 = (uint64_t)this;
	r_user_data.part2 = p_shape_index;
}

RapierCollisionObject2D *RapierCollisionObject2D::get_collider_user_data(const rapier2d::UserData &p_user_data, uint32_t &r_shape_index) {
	r_shape_index = (uint32_t)p_user_data.part2;
	return (RapierCollisionObject2D *)p_user_data.part1;
}

void RapierCollisionObject2D::_shape_changed(RapierShape2D *p_shape) {
	if (!space) {
		return;
	}

	for (uint32_t i = 0; i < shapes.size(); i++) {
		Shape &shape = shapes[i];
		if (shape.shape != p_shape) {
			continue;
		}
		if (shape.disabled) {
			continue;
		}

		_destroy_shape(shape, i);

		_create_shape(shape, i);
		_update_shape_transform(shape);
	}

	_shapes_changed();
}

RapierCollisionObject2D::RapierCollisionObject2D(Type p_type) {
	//: pending_shape_update_list(this) {
	type = p_type;
}
