#ifndef LIQUID_EFFECT_2D_PRESSURE_IISPH_H
#define LIQUID_EFFECT_2D_PRESSURE_IISPH_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DPressureIISPH : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DPressureIISPH, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DPressureIISPH(){}
};

#endif // LIQUID_EFFECT_2D_PRESSURE_IISPH_H
