#ifndef LIQUID_EFFECT_2D_H
#define LIQUID_EFFECT_2D_H

#include <godot_cpp/classes/resource.hpp>

using namespace godot;

class FluidEffect2D : public Resource {
	GDCLASS(FluidEffect2D, Resource);

public:
	enum FluidEffectType {
		FLUID_EFFECT_ELASTICITY = 0,
		FLUID_EFFECT_SURFACE_TENSION_AKINCI = 1,
		FLUID_EFFECT_SURFACE_TENSION_HE = 2,
		FLUID_EFFECT_SURFACE_TENSION_WCSPH = 3,
		FLUID_EFFECT_VISCOSITY_ARTIFICIAL = 4,
		FLUID_EFFECT_VISCOSITY_DFSPH = 5,
		FLUID_EFFECT_VISCOSITY_XSPH = 6
	};

protected:
	static void _bind_methods() {}
	FluidEffectType fluid_effect_type;

public:
	_FORCE_INLINE_ FluidEffectType get_fluid_effect_type() { return fluid_effect_type; }
	~FluidEffect2D() {}
};

#endif // LIQUID_EFFECT_2D_H
