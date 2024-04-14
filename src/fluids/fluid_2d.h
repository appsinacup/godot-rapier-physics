#ifndef LIQUID_2D_H
#define LIQUID_2D_H

#include "fluid_effect_2d.h"
#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/variant/array.hpp>

using namespace godot;

class Fluid2D : public Node2D {
	GDCLASS(Fluid2D, Node2D)

private:
	RID rid;
	real_t radius;
	bool enabled = true;
	bool debug_draw = false;
	real_t density = 1.0;
	Color color;
	TypedArray<FluidEffect2D> effects;

	PackedVector2Array points;

protected:
	static void _bind_methods();

	void _notification(int p_what);

public:
	RID get_rid() const;

	real_t get_density() const;
	void set_density(real_t p_density);

	PackedVector2Array get_accelerations() const;
	PackedVector2Array get_velocities() const;
	PackedVector2Array get_points() const;
	PackedVector2Array create_rectangle_points(int width, int height, Vector2 origin = Vector2());
	void set_points(PackedVector2Array p_points);

	void set_effects(const TypedArray<FluidEffect2D> &p_effects);
	TypedArray<FluidEffect2D> get_effects() const;

	Fluid2D();
	~Fluid2D();

	//void _process(double delta) override;
};

#endif // LIQUID_2D_H
