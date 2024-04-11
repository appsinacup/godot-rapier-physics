#ifndef LIQUID_2D_H
#define LIQUID_2D_H

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/templates/vector.hpp>
#include "liquid_effect_2d.h"

using namespace godot;

class Liquid2D : public Node2D {
	GDCLASS(Liquid2D, Node2D)

private:
	real_t density = 1.0;
	Vector<Ref<LiquidEffect2D>> effects = Vector<Ref<LiquidEffect2D>>();

protected:
	static void _bind_methods();

public:
	real_t get_density() const;
	void set_density(real_t p_density);

	void set_effect(const Ref<LiquidEffect2D> &p_shape);
	Ref<LiquidEffect2D> get_effect() const;

	Liquid2D(){}
	~Liquid2D(){}

	//void _process(double delta) override;
};

#endif // LIQUID_2D_H
