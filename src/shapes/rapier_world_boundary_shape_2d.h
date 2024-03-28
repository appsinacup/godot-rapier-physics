#ifndef RAPIER_WORLD_BOUNDARY_SHAPE_2D_H
#define RAPIER_WORLD_BOUNDARY_SHAPE_2D_H

#include "rapier_shape_2d.h"

class RapierWorldBoundaryShape2D : public RapierShape2D {
	Vector2 normal;
	real_t d = 0.0;

protected:
	virtual rapier2d::Handle create_rapier_shape() const override;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_WORLD_BOUNDARY; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override { return 0.0; }
};

#endif // RAPIER_WORLD_BOUNDARY_SHAPE_2D_H
