#include "rapier_segment_shape_2d.h"

rapier2d::Handle RapierSegmentShape2D::create_rapier_shape() const {
	Vector2 direction = b - a;
	direction.normalize();

	Vector2 perpendicular = Vector2(-direction.y, direction.x);
	double height = 0.1;

	Vector2 p1 = a + perpendicular * height / 2.0;
	Vector2 p2 = a - perpendicular * height / 2.0;
	Vector2 p3 = b + perpendicular * height / 2.0;
	Vector2 p4 = b - perpendicular * height / 2.0;

	rapier2d::Vector rapier_points[4];
	rapier_points[0] = rapier2d::Vector{ p1.x, p1.y };
	rapier_points[1] = rapier2d::Vector{ p2.x, p2.y };
	rapier_points[2] = rapier2d::Vector{ p3.x, p3.y };
	rapier_points[3] = rapier2d::Vector{ p4.x, p4.y };

	return rapier2d::shape_create_convex_polyline(rapier_points, 4);
}

void RapierSegmentShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::RECT2);

	Rect2 r = p_data;
	a = r.position;
	b = r.size;
	n = (b - a).orthogonal();

	Rect2 aabb;
	aabb.position = a;
	aabb.expand_to(b);
	if (aabb.size.x == 0) {
		aabb.size.x = 0.001;
	}
	if (aabb.size.y == 0) {
		aabb.size.y = 0.001;
	}
	configure(aabb);
}

Variant RapierSegmentShape2D::get_data() const {
	Rect2 r;
	r.position = a;
	r.size = b;
	return r;
}

real_t RapierSegmentShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	return p_mass * ((a * p_scale).distance_squared_to(b * p_scale)) / 12.0;
}
