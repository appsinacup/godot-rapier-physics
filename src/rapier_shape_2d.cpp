#include "rapier_shape_2d.h"

void RapierShape2D::configure(const Rect2 &p_aabb) {
	aabb = p_aabb;
	configured = true;
	for (const KeyValue<RapierShapeOwner2D *, int> &E : owners) {
		RapierShapeOwner2D *co = const_cast<RapierShapeOwner2D *>(E.key);
		co->_shape_changed(this);
	}
}

void RapierShape2D::destroy_rapier_shape() {
	if (rapier2d::is_handle_valid(shape_handle)) {
		rapier2d::shape_destroy(shape_handle);
		shape_handle = rapier2d::invalid_handle();
	}
}

rapier2d::Handle RapierShape2D::get_rapier_shape() {
	if (!rapier2d::is_handle_valid(shape_handle)) {
		shape_handle = create_rapier_shape();
	}

	return shape_handle;
}

void RapierShape2D::add_owner(RapierShapeOwner2D *p_owner) {
	HashMap<RapierShapeOwner2D *, int>::Iterator E = owners.find(p_owner);
	if (E) {
		E->value++;
	} else {
		owners[p_owner] = 1;
	}
}

void RapierShape2D::remove_owner(RapierShapeOwner2D *p_owner) {
	HashMap<RapierShapeOwner2D *, int>::Iterator E = owners.find(p_owner);
	ERR_FAIL_COND(!E);
	E->value--;
	if (E->value == 0) {
		owners.remove(E);
	}
}

bool RapierShape2D::is_owner(RapierShapeOwner2D *p_owner) const {
	return owners.has(p_owner);
}

const HashMap<RapierShapeOwner2D *, int> &RapierShape2D::get_owners() const {
	return owners;
}

RapierShape2D::~RapierShape2D() {
	ERR_FAIL_COND(owners.size());
	destroy_rapier_shape();
}

/*********************************************************/
/*********************************************************/
/*********************************************************/

rapier2d::Handle RapierWorldBoundaryShape2D::create_rapier_shape() const {
	rapier2d::Vector v = { normal.x, normal.y };
	return rapier2d::shape_create_halfspace(&v);
}

void RapierWorldBoundaryShape2D::apply_rapier_transform(rapier2d::Vector &position, real_t &angle) const {
	position.x += normal.x * d;
	position.y += normal.y * d;
}

void RapierWorldBoundaryShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY);
	Array arr = p_data;
	ERR_FAIL_COND(arr.size() != 2);
	normal = arr[0];
	d = arr[1];
	configure(Rect2(Vector2(-1e4, -1e4), Vector2(1e4 * 2, 1e4 * 2)));
}

Variant RapierWorldBoundaryShape2D::get_data() const {
	Array arr;
	arr.resize(2);
	arr[0] = normal;
	arr[1] = d;
	return arr;
}

/*********************************************************/
/*********************************************************/
/*********************************************************/

// void RapierSeparationRayShape2D::set_data(const Variant &p_data) {
// 	Dictionary d = p_data;
// 	length = d["length"];
// 	slide_on_slope = d["slide_on_slope"];
// 	configure(Rect2(0, 0, 0.001, length));
// }

// Variant RapierSeparationRayShape2D::get_data() const {
// 	Dictionary d;
// 	d["length"] = length;
// 	d["slide_on_slope"] = slide_on_slope;
// 	return d;
// }

/*********************************************************/
/*********************************************************/
/*********************************************************/

rapier2d::Handle RapierSegmentShape2D::create_rapier_shape() const {
	Vector2 direction = b - a;
	direction.normalize();

	Vector2 perpendicular = Vector2(-direction.y, direction.x);
	float height = 0.1;

	Vector2 p1 = a + perpendicular * height / 2;
	Vector2 p2 = a - perpendicular * height / 2;
	Vector2 p3 = b + perpendicular * height / 2;
	Vector2 p4 = b - perpendicular * height / 2;

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
	return p_mass * ((a * p_scale).distance_squared_to(b * p_scale)) / 12;
}

/*********************************************************/
/*********************************************************/
/*********************************************************/

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

/*********************************************************/
/*********************************************************/
/*********************************************************/

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

/*********************************************************/
/*********************************************************/
/*********************************************************/

rapier2d::Handle RapierConvexPolygonShape2D::create_rapier_shape() const {
	ERR_FAIL_COND_V(point_count < 3, rapier2d::invalid_handle());
	rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(point_count * sizeof(rapier2d::Vector));
	for (int i = 0; i < point_count; i++) {
		rapier_points[i] = rapier2d::Vector{ (points[i].pos.x), (points[i].pos.y) };
	}
	return rapier2d::shape_create_convex_polyline(rapier_points, point_count);
}

void RapierConvexPolygonShape2D::set_data(const Variant &p_data) {
#ifdef REAL_T_IS_DOUBLE
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT64_ARRAY);
#else
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT32_ARRAY);
#endif

	if (points) {
		memdelete_arr(points);
	}
	points = nullptr;
	point_count = 0;

	if (p_data.get_type() == Variant::PACKED_VECTOR2_ARRAY) {
		PackedVector2Array arr = p_data;
		ERR_FAIL_COND(arr.size() == 0);
		point_count = arr.size();
		points = memnew_arr(Point, point_count);
		const Vector2 *r = arr.ptr();

		for (int i = 0; i < point_count; i++) {
			points[i].pos = r[i];
		}

		for (int i = 0; i < point_count; i++) {
			Vector2 p = points[i].pos;
			Vector2 pn = points[(i + 1) % point_count].pos;
			points[i].normal = (pn - p).orthogonal().normalized();
		}
	} else {
#ifdef REAL_T_IS_DOUBLE
		PackedFloat64Array dvr = p_data;
#else
		PackedFloat32Array dvr = p_data;
#endif
		point_count = dvr.size() / 4;
		ERR_FAIL_COND(point_count == 0);

		points = memnew_arr(Point, point_count);
		const real_t *r = dvr.ptr();

		for (int i = 0; i < point_count; i++) {
			int idx = i << 2;
			points[i].pos.x = r[idx + 0];
			points[i].pos.y = r[idx + 1];
			points[i].normal.x = r[idx + 2];
			points[i].normal.y = r[idx + 3];
		}
	}

	ERR_FAIL_COND(point_count == 0);
	Rect2 aabb;
	aabb.position = points[0].pos;
	for (int i = 1; i < point_count; i++) {
		aabb.expand_to(points[i].pos);
	}

	configure(aabb);
}

Variant RapierConvexPolygonShape2D::get_data() const {
	PackedVector2Array dvr;

	dvr.resize(point_count);

	for (int i = 0; i < point_count; i++) {
		dvr.set(i, points[i].pos);
	}

	return dvr;
}

real_t RapierConvexPolygonShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	ERR_FAIL_COND_V_MSG(point_count == 0, 0, "Convex polygon shape has no points.");
	Rect2 aabb_new;
	aabb_new.position = points[0].pos * p_scale;
	for (int i = 0; i < point_count; i++) {
		aabb_new.expand_to(points[i].pos * p_scale);
	}

	return p_mass * aabb_new.size.dot(aabb_new.size) / 12.0;
}

RapierConvexPolygonShape2D::~RapierConvexPolygonShape2D() {
	if (points) {
		memdelete_arr(points);
	}
}

/*********************************************************/
/*********************************************************/
/*********************************************************/

rapier2d::Handle RapierCapsuleShape2D::create_rapier_shape() const {
	return rapier2d::shape_create_capsule((height / 2) - radius, radius);
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
	configure(Rect2(-he, he * 2));
}

Variant RapierCapsuleShape2D::get_data() const {
	return Point2(height, radius);
}

real_t RapierCapsuleShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	Vector2 he2 = Vector2(radius * 2.0, height) * p_scale;
	return p_mass * he2.dot(he2) / 12.0;
}

// /*********************************************************/
// /*********************************************************/
// /*********************************************************/

rapier2d::Handle RapierConcavePolygonShape2D::create_rapier_shape() const {
	int point_count = points.size();
	ERR_FAIL_COND_V(point_count < 3, rapier2d::invalid_handle());
	rapier2d::Vector *rapier_points = (rapier2d::Vector *)alloca(point_count * sizeof(rapier2d::Vector));
	for (int i = 0; i < point_count; i++) {
		rapier_points[i] = rapier2d::Vector{ (points[i].x), (points[i].y) };
	}
	return rapier2d::shape_create_convave_polyline(rapier_points, point_count);
}

void RapierConcavePolygonShape2D::set_data(const Variant &p_data) {
#ifdef REAL_T_IS_DOUBLE
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT64_ARRAY);
#else
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT32_ARRAY);
#endif

	Rect2 aabb;

	if (p_data.get_type() == Variant::PACKED_VECTOR2_ARRAY) {
		PackedVector2Array p2arr = p_data;
		int len = p2arr.size();
		ERR_FAIL_COND(len % 2);

		segments.clear();
		points.clear();

		if (len == 0) {
			configure(aabb);
			return;
		}

		const Vector2 *arr = p2arr.ptr();

		HashMap<Point2, int> pointmap;
		for (int i = 0; i < len; i += 2) {
			Point2 p1 = arr[i];
			Point2 p2 = arr[i + 1];
			int idx_p1, idx_p2;

			if (pointmap.has(p1)) {
				idx_p1 = pointmap[p1];
			} else {
				idx_p1 = pointmap.size();
				pointmap[p1] = idx_p1;
			}

			if (pointmap.has(p2)) {
				idx_p2 = pointmap[p2];
			} else {
				idx_p2 = pointmap.size();
				pointmap[p2] = idx_p2;
			}

			Segment s;
			s.points[0] = idx_p1;
			s.points[1] = idx_p2;
			segments.push_back(s);
		}

		points.resize(pointmap.size());
		aabb.position = pointmap.begin()->key;
		for (const KeyValue<Point2, int> &E : pointmap) {
			aabb.expand_to(E.key);
			points[E.value] = E.key;
		}
	} else {
		//dictionary with arrays
	}

	configure(aabb);
}

Variant RapierConcavePolygonShape2D::get_data() const {
	PackedVector2Array rsegments;
	int len = segments.size();
	if (len == 0) {
		return rsegments;
	}

	rsegments.resize(len * 2);
	Vector2 *w = rsegments.ptrw();
	for (int i = 0; i < len; i++) {
		w[(i << 1) + 0] = points[segments[i].points[0]];
		w[(i << 1) + 1] = points[segments[i].points[1]];
	}

	return rsegments;
}
