#include "rapier_circle_shape_2d.h"

rapier2d::Handle RapierCircleShape2D::create_rapier_shape() const {
	return rapier2d::shape_create_circle(radius);
}

void RapierCircleShape2D::set_data(const Variant &p_data) {
	radius = p_data;
	configure(Rect2(-radius, -radius, radius * 2, radius * 2));
}

Variant RapierCircleShape2D::get_data() const {
	return radius;
}

real_t RapierCircleShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	real_t a = radius * p_scale.x;
	real_t b = radius * p_scale.y;
	return p_mass * (a * a + b * b) / 4.0;
}
