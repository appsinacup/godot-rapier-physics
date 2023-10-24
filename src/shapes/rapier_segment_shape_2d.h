#ifndef RAPIER_SEGMENT_SHAPE_2D_H
#define RAPIER_SEGMENT_SHAPE_2D_H

#include "rapier_shape_2d.h"

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

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // RAPIER_SEGMENT_SHAPE_2D_H
