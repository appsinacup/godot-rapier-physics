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
	real_t lifetime = 0.0;
	TypedArray<FluidEffect2D> effects;

	PackedVector2Array points;
	PackedFloat32Array create_times;

protected:
	static void _bind_methods();

	void _notification(int p_what);

	void _delete_old_particles();

public:
	RID get_rid() const;

	real_t get_density() const;
	void set_density(real_t p_density);

	real_t get_lifetime() const;
	void set_lifetime(real_t p_lifetime);

	PackedVector2Array get_accelerations() const;
	PackedVector2Array get_velocities() const;
	PackedVector2Array get_points() const;
	PackedFloat32Array get_create_times() const;
	PackedVector2Array create_rectangle_points(int width, int height);
	PackedVector2Array create_circle_points(int p_radius);
	void set_points(PackedVector2Array p_points);
	void add_points_and_velocities(PackedVector2Array p_points, PackedVector2Array p_velocities);
	void set_points_and_velocities(PackedVector2Array p_points, PackedVector2Array p_velocities);

	void delete_points(PackedInt32Array p_indices);

	void set_effects(const TypedArray<FluidEffect2D> &p_effects);
	TypedArray<FluidEffect2D> get_effects() const;

	void set_debug_draw(bool p_debug_draw);
	bool get_debug_draw() const;

	Fluid2D();
	~Fluid2D();

	//void _process(double delta) override;
};

#endif // LIQUID_2D_H
