#ifndef LIQUID_EFFECT_2D_H
#define LIQUID_EFFECT_2D_H

#include <godot_cpp/classes/resource.hpp>

using namespace godot;

class LiquidEffect2D : public Resource {
	GDCLASS(LiquidEffect2D, Resource);

protected:
	static void _bind_methods(){}
public:
	~LiquidEffect2D(){}
};

#endif // LIQUID_EFFECT_2D_H
