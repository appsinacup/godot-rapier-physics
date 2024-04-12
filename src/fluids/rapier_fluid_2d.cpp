#include "fluid_2d.h"
#include "../servers/rapier_physics_server_2d.h"
#include <godot_cpp/classes/world2d.hpp>

real_t RapierFluid2D::get_density() const {
	return density;
}

void RapierFluid2D::set_density(real_t p_density) {
	density = p_density;
    if (space) {
        rapier2d::Handle space_handle = space->get_handle();
        ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
        ERR_FAIL_COND(!rapier2d::is_handle_valid(fluid_handle));

        rapier2d::fluid_change_density(space_handle, fluid_handle, density);
    }
}

void RapierFluid2D::set_space(RapierSpace2D *p_space){
    space = p_space;
    if(space) {
		rapier2d::Handle space_handle = space->get_handle();
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

        fluid_handle = rapier2d::fluid_create(space_handle, density);
		ERR_FAIL_COND(!rapier2d::is_handle_valid(fluid_handle));
    }
}

RapierSpace2D * RapierFluid2D::get_space() const{
    return space;
}

RID RapierFluid2D::get_rid() const {
    return rid;
}
void RapierFluid2D::set_rid(RID p_rid) {
    rid = p_rid;
}
