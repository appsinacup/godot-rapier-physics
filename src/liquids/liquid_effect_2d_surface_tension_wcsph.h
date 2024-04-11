#ifndef LIQUID_EFFECT_2D_SURFACE_TENSION_WCSPH_H
#define LIQUID_EFFECT_2D_SURFACE_TENSION_WCSPH_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DSurfaceTensionWCSPH : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DSurfaceTensionWCSPH, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DSurfaceTensionWCSPH(){}
};

#endif // LIQUID_EFFECT_2D_SURFACE_TENSION_WCSPH_H
