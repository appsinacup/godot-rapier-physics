#ifndef RAPIER_LIQUID_2D_H
#define RAPIER_LIQUID_2D_H

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/templates/vector.hpp>

#include "../spaces/rapier_space_2d.h"
#include "../rapier_include.h"

using namespace godot;

class RapierFluid2D {
private:
	RID rid;
	bool enabled = true;
	real_t density = 1.0;
    RapierSpace2D *space;
    rapier2d::Handle fluid_handle  = rapier2d::invalid_handle();

public:
	real_t get_density() const;
	void set_density(real_t p_density);

	RID get_rid() const;
	void set_rid(RID p_rid);

	void set_space(RapierSpace2D *p_space);
	RapierSpace2D* get_space() const;

	//void _process(double delta) override;
};

#endif // RAPIER_LIQUID_2D_H
