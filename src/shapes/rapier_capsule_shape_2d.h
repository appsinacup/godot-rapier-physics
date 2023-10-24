#ifndef RAPIER_CAPSULE_SHAPE_2D_H
#define RAPIER_CAPSULE_SHAPE_2D_H

#include "rapier_shape_2d.h"

class RapierCapsuleShape2D : public RapierShape2D {
	real_t radius = 0.0;
	real_t height = 0.0;

public:
	_FORCE_INLINE_ const real_t &get_radius() const { return radius; }
	_FORCE_INLINE_ const real_t &get_height() const { return height; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CAPSULE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;
};

#endif // RAPIER_CAPSULE_SHAPE_2D_H
