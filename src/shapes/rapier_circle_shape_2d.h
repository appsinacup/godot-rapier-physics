#ifndef RAPIER_CIRCLE_SHAPE_2D_H
#define RAPIER_CIRCLE_SHAPE_2D_H

#include "rapier_shape_2d.h"

class RapierCircleShape2D : public RapierShape2D {
	real_t radius;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:
	_FORCE_INLINE_ const real_t &get_radius() const { return radius; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CIRCLE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // RAPIER_CIRCLE_SHAPE_2D_H
