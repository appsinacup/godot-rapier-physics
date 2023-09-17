#ifndef RAPIER_SHAPE_2D_H
#define RAPIER_SHAPE_2D_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/local_vector.hpp>

#include "rapier_include.h"

using namespace godot;

class RapierShape2D;

class RapierShapeOwner2D {
public:
	virtual void _shape_changed(RapierShape2D* p_shape) = 0;
	virtual void remove_shape(RapierShape2D *p_shape) = 0;

	virtual ~RapierShapeOwner2D() {}
};

class RapierShape2D {
	RID rid;
	Rect2 aabb;
	bool configured = false;
	real_t custom_bias = 0.0;

	HashMap<RapierShapeOwner2D *, int> owners;

	rapier2d::Handle shape_handle = rapier2d::invalid_handle();

protected:
	void configure(const Rect2 &p_aabb);

	virtual rapier2d::Handle create_rapier_shape() const = 0;
	void destroy_rapier_shape();

public:
	_FORCE_INLINE_ void set_rid(const RID & p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	virtual PhysicsServer2D::ShapeType get_type() const = 0;

	virtual void apply_rapier_transform(rapier2d::Vector& position, real_t& angle) const {}

	rapier2d::Handle get_rapier_shape();

	_FORCE_INLINE_ Rect2 get_aabb() const { return aabb; }
	_FORCE_INLINE_ bool is_configured() const { return configured; }

	void add_owner(RapierShapeOwner2D *p_owner);
	void remove_owner(RapierShapeOwner2D *p_owner);
	bool is_owner(RapierShapeOwner2D *p_owner) const;
	const HashMap<RapierShapeOwner2D *, int> &get_owners() const;

	virtual void set_data(const Variant &p_data) = 0;
	virtual Variant get_data() const = 0;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const = 0;

	RapierShape2D() {}
	virtual ~RapierShape2D();
};

class RapierWorldBoundaryShape2D : public RapierShape2D {
	Vector2 normal;
	real_t d = 0.0;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_WORLD_BOUNDARY; }

	virtual void apply_rapier_transform(rapier2d::Vector& position, real_t& angle) const override;

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override { return 0.0; }
};

// class RapierSeparationRayShape2D : public RapierShape2D {
// 	real_t length = 0.0;
// 	bool slide_on_slope = false;

// public:

// 	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_SEPARATION_RAY; }

// 	virtual bool allows_one_way_collision() const override { return false; }

// 	virtual void set_data(const Variant &p_data) override;
// 	virtual Variant get_data() const override;

//	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override;

// 	DEFAULT_PROJECT_RANGE_CAST

// 	_FORCE_INLINE_ RapierSeparationRayShape2D() {}
// 	_FORCE_INLINE_ RapierSeparationRayShape2D(real_t p_length) { length = p_length; }
// };

class RapierSegmentShape2D : public RapierShape2D {
	Vector2 a;
	Vector2 b;
	Vector2 n;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:
	_FORCE_INLINE_ const Vector2 &get_a() const { return a; }
	_FORCE_INLINE_ const Vector2 &get_b() const { return b; }
	_FORCE_INLINE_ const Vector2 &get_normal() const { return n; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_SEGMENT; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override;

	_FORCE_INLINE_ RapierSegmentShape2D() {}
	_FORCE_INLINE_ RapierSegmentShape2D(const Vector2 &p_a, const Vector2 &p_b, const Vector2 &p_n) {
		a = p_a;
		b = p_b;
		n = p_n;
	}
};

class RapierCircleShape2D : public RapierShape2D {
	real_t radius;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:
	_FORCE_INLINE_ const real_t &get_radius() const { return radius; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CIRCLE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override;
};

class RapierRectangleShape2D : public RapierShape2D {
	Vector2 half_extents;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:
	_FORCE_INLINE_ const Vector2 &get_half_extents() const { return half_extents; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_RECTANGLE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override;
};

class RapierConvexPolygonShape2D : public RapierShape2D {
	struct Point {
		Vector2 pos;
		Vector2 normal; //normal to next segment
	};

	Point *points = nullptr;
	int point_count = 0;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CONVEX_POLYGON; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override;

	~RapierConvexPolygonShape2D();

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;
};

class RapierCapsuleShape2D : public RapierShape2D {
	real_t radius = 0.0;
	real_t height = 0.0;

public:
	_FORCE_INLINE_ const real_t &get_radius() const { return radius; }
	_FORCE_INLINE_ const real_t &get_height() const { return height; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CAPSULE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;
};

class RapierConcavePolygonShape2D : public RapierShape2D {
	struct Segment {
		int points[2] = {};
	};

	LocalVector<Segment> segments;
	LocalVector<Point2> points;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CONCAVE_POLYGON; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2& p_scale) const override { return 0.0; }
};

#endif // RAPIER_SHAPE_2D_H
