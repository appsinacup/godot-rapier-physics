#ifndef LIQUID_EFFECT_2D_H
#define LIQUID_EFFECT_2D_H

#include <godot_cpp/classes/resource.hpp>

using namespace godot;

class FluidEffect2D : public Resource {
	GDCLASS(FluidEffect2D, Resource);

protected:
	static void _bind_methods() {}

public:
	~FluidEffect2D() {}
};

#endif // LIQUID_EFFECT_2D_H
