#include "rapier_world_boundary_shape_2d.h"

rapier2d::Handle RapierWorldBoundaryShape2D::create_rapier_shape() const {
	rapier2d::Vector v = { normal.x, -normal.y };
	return rapier2d::shape_create_halfspace(&v, -d);
}

void RapierWorldBoundaryShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY);
	Array arr = p_data;
	ERR_FAIL_COND(arr.size() != 2);
	normal = arr[0];
	d = arr[1];
	configure(Rect2(Vector2(-1e4, -1e4), Vector2(1e4 * 2.0, 1e4 * 2.0)));
}

Variant RapierWorldBoundaryShape2D::get_data() const {
	Array arr;
	arr.resize(2);
	arr[0] = normal;
	arr[1] = d;
	return arr;
}
