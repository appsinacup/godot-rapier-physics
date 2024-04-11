#ifndef LIQUID_EFFECT_2D_SURFACE_TENSION_HE_H
#define LIQUID_EFFECT_2D_SURFACE_TENSION_HE_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DSurfaceTensionHE : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DSurfaceTensionHE, LiquidEffect2D);
	real_t fluid_tension_coefficient = 1.0;
	real_t boundary_adhesion_coefficient = 0.0;

protected:
	static void _bind_methods();

public:
	void set_fluid_tension_coefficient(real_t p_fluid_tension_coefficient);
	real_t get_fluid_tension_coefficient() const;
	void set_boundary_adhesion_coefficient(real_t p_boundary_adhesion_coefficient);
	real_t get_boundary_adhesion_coefficient() const;
};

#endif // LIQUID_EFFECT_2D_SURFACE_TENSION_HE_H
