#ifndef LIQUID_EFFECT_2D_SURFACE_TENSION_AKINCI_H
#define LIQUID_EFFECT_2D_SURFACE_TENSION_AKINCI_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DSurfaceTensionAKINCI : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DSurfaceTensionAKINCI, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DSurfaceTensionAKINCI(){}
};

#endif // LIQUID_EFFECT_2D_SURFACE_TENSION_AKINCI_H
