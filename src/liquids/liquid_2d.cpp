#include "liquid_2d.h"
#include "../servers/rapier_physics_server_2d.h"

real_t Liquid2D::get_density() const {
    return density;
}

void Liquid2D::set_density(real_t p_density) {
    density = p_density;
}

void Liquid2D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_density"), &Liquid2D::get_density);
    ClassDB::bind_method(D_METHOD("set_density", "density"), &Liquid2D::set_density);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_density", "get_density");
}
