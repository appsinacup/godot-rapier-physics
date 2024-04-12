#ifndef LIQUID_2D_H
#define LIQUID_2D_H

#include "fluid_effect_2d.h"
#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/templates/vector.hpp>

using namespace godot;

class Fluid2D : public Node2D {
	GDCLASS(Fluid2D, Node2D)

private:
	RID rid;
	bool enabled = true;
	real_t density = 1.0;
	Vector<Ref<FluidEffect2D>> effects = Vector<Ref<FluidEffect2D>>();
	PackedVector2Array points;

protected:
	static void _bind_methods();

	void _notification(int p_what);

public:
	RID get_rid() const;

	real_t get_density() const;
	void set_density(real_t p_density);

	PackedVector2Array get_points() const;
	void set_points(PackedVector2Array p_density);

	PackedVector2Array get_points() const;
	void set_points(PackedVector2Array p_points);

	void set_effect(const Ref<FluidEffect2D> &p_shape);
	Ref<FluidEffect2D> get_effect() const;

	Fluid2D();
	~Fluid2D();

	//void _process(double delta) override;
};

#endif // LIQUID_2D_H
