#ifndef LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H
#define LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DViscosityDFSPH : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DViscosityDFSPH, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DViscosityDFSPH(){}
};

#endif // LIQUID_EFFECT_2D_VISCOSITY_DFSPH_H
