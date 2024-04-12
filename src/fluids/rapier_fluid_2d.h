#ifndef RAPIER_LIQUID_2D_H
#define RAPIER_LIQUID_2D_H

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/templates/vector.hpp>

#include "../rapier_include.h"
#include "../spaces/rapier_space_2d.h"
#include "fluid_effect_2d.h"

using namespace godot;

class RapierFluid2D {
private:
	RID rid;
	bool enabled = true;
	real_t density = 1.0;
	RapierSpace2D *space;
	TypedArray<FluidEffect2D> effects = TypedArray<FluidEffect2D>();
	rapier2d::HandleDouble fluid_handle = rapier2d::invalid_handle_double();
	PackedVector2Array points;
	PackedVector2Array velocities;
	PackedVector2Array accelerations;

public:
	void set_points(PackedVector2Array p_points);
	PackedVector2Array get_points();
	PackedVector2Array get_velocities();
	PackedVector2Array get_accelerations();

	void set_effects(const TypedArray<FluidEffect2D> &params);

	real_t get_density() const;
	void set_density(real_t p_density);

	RID get_rid() const;
	void set_rid(RID p_rid);

	void set_space(RapierSpace2D *p_space);
	RapierSpace2D *get_space() const;

	//void _process(double delta) override;
};

#endif // RAPIER_LIQUID_2D_H
