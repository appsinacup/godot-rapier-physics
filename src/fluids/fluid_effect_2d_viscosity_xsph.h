#ifndef LIQUID_EFFECT_2D_VISCOSITY_XSPH_H
#define LIQUID_EFFECT_2D_VISCOSITY_XSPH_H

#include "fluid_effect_2d.h"

using namespace godot;

class FluidEffect2DViscosityXSPH : public FluidEffect2D {
	GDCLASS(FluidEffect2DViscosityXSPH, FluidEffect2D);
	real_t fluid_viscosity_coefficient = 1.0;
	real_t boundary_viscosity_coefficient = 0.0;

protected:
	static void _bind_methods();

public:
	void set_fluid_viscosity_coefficient(real_t p_fluid_viscosity_coefficient);
	real_t get_fluid_viscosity_coefficient() const;
	void set_boundary_viscosity_coefficient(real_t p_boundary_viscosity_coefficient);
	real_t get_boundary_viscosity_coefficient() const;
	FluidEffect2DViscosityXSPH() {
		fluid_effect_type = FluidEffect2D::FLUID_EFFECT_VISCOSITY_XSPH;
	}
};

#endif // LIQUID_EFFECT_2D_VISCOSITY_XSPH_H
