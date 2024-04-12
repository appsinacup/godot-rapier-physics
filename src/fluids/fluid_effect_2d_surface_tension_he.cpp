#include "fluid_effect_2d_surface_tension_he.h"

real_t FluidEffect2DSurfaceTensionHE::get_fluid_tension_coefficient() const {
	return fluid_tension_coefficient;
}

void FluidEffect2DSurfaceTensionHE::set_fluid_tension_coefficient(real_t p_fluid_tension_coefficient) {
	fluid_tension_coefficient = p_fluid_tension_coefficient;
}

real_t FluidEffect2DSurfaceTensionHE::get_boundary_adhesion_coefficient() const {
	return boundary_adhesion_coefficient;
}

void FluidEffect2DSurfaceTensionHE::set_boundary_adhesion_coefficient(real_t p_boundary_adhesion_coefficient) {
	boundary_adhesion_coefficient = p_boundary_adhesion_coefficient;
}

void FluidEffect2DSurfaceTensionHE::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_fluid_tension_coefficient"), &FluidEffect2DSurfaceTensionHE::get_fluid_tension_coefficient);
	ClassDB::bind_method(D_METHOD("set_fluid_tension_coefficient", "fluid_tension_coefficient"), &FluidEffect2DSurfaceTensionHE::set_fluid_tension_coefficient);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fluid_tension_coefficient", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_fluid_tension_coefficient", "get_fluid_tension_coefficient");

	ClassDB::bind_method(D_METHOD("get_boundary_adhesion_coefficient"), &FluidEffect2DSurfaceTensionHE::get_boundary_adhesion_coefficient);
	ClassDB::bind_method(D_METHOD("set_boundary_adhesion_coefficient", "boundary_adhesion_coefficient"), &FluidEffect2DSurfaceTensionHE::set_boundary_adhesion_coefficient);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "boundary_adhesion_coefficient", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_boundary_adhesion_coefficient", "get_boundary_adhesion_coefficient");
}
