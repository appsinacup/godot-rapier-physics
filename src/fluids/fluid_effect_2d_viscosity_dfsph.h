#ifndef LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H
#define LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H

#include "fluid_effect_2d.h"

using namespace godot;

class FluidEffect2DViscosityDFSPH : public FluidEffect2D {
	GDCLASS(FluidEffect2DViscosityDFSPH, FluidEffect2D);
	real_t fluid_viscosity_coefficient = 1.0;

protected:
	static void _bind_methods();

public:
	void set_fluid_viscosity_coefficient(real_t p_fluid_viscosity_coefficient);
	real_t get_fluid_viscosity_coefficient() const;
	FluidEffect2DViscosityDFSPH() {
		fluid_effect_type = FluidEffect2D::FLUID_EFFECT_VISCOSITY_DFSPH;
	}
};

#endif // LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H
