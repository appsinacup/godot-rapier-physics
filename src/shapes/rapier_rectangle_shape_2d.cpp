#include "rapier_rectangle_shape_2d.h"

rapier2d::Handle RapierRectangleShape2D::create_rapier_shape() const {
	rapier2d::Vector v = { half_extents.x * 2.0f, half_extents.y * 2.0f };
	return rapier2d::shape_create_box(&v);
}

void RapierRectangleShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);

	half_extents = p_data;
	configure(Rect2(-half_extents, half_extents * 2.0));
}

Variant RapierRectangleShape2D::get_data() const {
	return half_extents;
}

real_t RapierRectangleShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	Vector2 he2 = half_extents * 2.0 * p_scale;
	return p_mass * he2.dot(he2) / 12.0;
}
