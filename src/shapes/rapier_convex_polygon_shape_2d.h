#ifndef RAPIER_CONVEX_POLYGON_SHAPE_2D_H
#define RAPIER_CONVEX_POLYGON_SHAPE_2D_H

#include "rapier_shape_2d.h"

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

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;

	~RapierConvexPolygonShape2D();

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;
};

#endif // RAPIER_CONVEX_POLYGON_SHAPE_2D_H
