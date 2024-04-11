#ifndef LIQUID_EFFECT_2D_SURFACE_TENSION_HE_H
#define LIQUID_EFFECT_2D_SURFACE_TENSION_HE_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DSurfaceTensionHE : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DSurfaceTensionHE, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DSurfaceTensionHE(){}
};

#endif // LIQUID_EFFECT_2D_SURFACE_TENSION_HE_H
