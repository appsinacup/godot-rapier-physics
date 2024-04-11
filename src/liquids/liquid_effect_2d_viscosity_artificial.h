#ifndef LIQUID_EFFECT_2D_VISCOSITY_ARTIFICIAL_H
#define LIQUID_EFFECT_2D_VISCOSITY_ARTIFICIAL_H

#include "liquid_effect_2d.h"

using namespace godot;

class LiquidEffect2DViscosityArtificial : public LiquidEffect2D {
	GDCLASS(LiquidEffect2DViscosityArtificial, LiquidEffect2D);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2DViscosityArtificial(){}
};

#endif // LIQUID_EFFECT_2D_VISCOSITY_ARTIFICIAL_H
