#ifndef LIQUID_EFFECT_2D_VISCOSITY_XSPH_H
#define LIQUID_EFFECT_2D_VISCOSITY_XSPH_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DViscosityXSPH : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DViscosityXSPH, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DViscosityXSPH(){}
};

#endif // LIQUID_EFFECT_2D_VISCOSITY_XSPH_H
