#ifndef LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H
#define LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DViscosityDFSPH : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DViscosityDFSPH, LiquidEffect2D);
	real_t fluid_viscosity_coefficient = 1.0;

protected:
	static void _bind_methods();

public:
	void set_fluid_viscosity_coefficient(real_t p_fluid_viscosity_coefficient);
	real_t get_fluid_viscosity_coefficient() const;
};

#endif // LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H
