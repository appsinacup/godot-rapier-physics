#ifndef LIQUID_EFFECT_2D_PRESSURE_DFSPH_H
#define LIQUID_EFFECT_2D_PRESSURE_DFSPH_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DPressureDFSPH : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DPressureDFSPH, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DPressureDFSPH(){}
};

#endif // LIQUID_EFFECT_2D_PRESSURE_DFSPH_H
