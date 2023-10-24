#include "rapier_capsule_shape_2d.h"

rapier2d::Handle RapierCapsuleShape2D::create_rapier_shape() const {
	return rapier2d::shape_create_capsule((height / 2.0) - radius, radius);
}

void RapierCapsuleShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY && p_data.get_type() != Variant::VECTOR2);

	if (p_data.get_type() == Variant::ARRAY) {
		Array arr = p_data;
		ERR_FAIL_COND(arr.size() != 2);
		height = arr[0];
		radius = arr[1];
	} else {
		Point2 p = p_data;
		radius = p.x;
		height = p.y;
	}

	Point2 he(radius, height * 0.5);
	configure(Rect2(-he, he * 2.0));
}

Variant RapierCapsuleShape2D::get_data() const {
	return Point2(height, radius);
}

real_t RapierCapsuleShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	Vector2 he2 = Vector2(radius * 2.0, height) * p_scale;
	return p_mass * he2.dot(he2) / 12.0;
}
