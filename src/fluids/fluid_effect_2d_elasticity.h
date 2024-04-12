#ifndef LIQUID_EFFECT_2D_ELASTICITY_H
#define LIQUID_EFFECT_2D_ELASTICITY_H

#include "fluid_effect_2d.h"

using namespace godot;

class FluidEffect2DElasticity : public FluidEffect2D {
	GDCLASS(FluidEffect2DElasticity, FluidEffect2D);
	real_t young_modulus = 1000.0;
	real_t poisson_ratio = 0.3;
	bool nonlinear_strain = true;

protected:
	static void _bind_methods();

public:
	real_t get_young_modulus() const;
	void set_young_modulus(real_t p_young_modulus);

	real_t get_poisson_ratio() const;
	void set_poisson_ratio(real_t p_poisson_ratio);

	bool get_nonlinear_strain() const;
	void set_nonlinear_strain(bool p_nonlinear_strain);
};

#endif // LIQUID_EFFECT_2D_ELASTICITY_H
