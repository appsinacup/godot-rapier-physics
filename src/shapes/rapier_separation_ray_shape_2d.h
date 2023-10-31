#ifndef RAPIER_SEPARATION_RAY_SHAPE_2D_H
#define RAPIER_SEPARATION_RAY_SHAPE_2D_H

#include "rapier_segment_shape_2d.h"

class RapierSeparationRayShape2D : public RapierSegmentShape2D {
	real_t length = 0.0;
	bool slide_on_slope = false;

protected:
	//virtual rapier2d::Handle create_rapier_shape() const override;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_SEPARATION_RAY; }

	//virtual void apply_rapier_transform(rapier2d::Vector &position, real_t &angle) const override;

	virtual bool allows_one_way_collision() const override { return false; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	//virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // RAPIER_SEPARATION_RAY_SHAPE_2D_H
