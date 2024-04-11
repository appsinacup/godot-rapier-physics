#include "liquid_effect_2d_viscosity_dfsph.h"

real_t LiquidEffect2DViscosityDFSPH::get_fluid_viscosity_coefficient() const {
    return fluid_viscosity_coefficient;
}

void LiquidEffect2DViscosityDFSPH::set_fluid_viscosity_coefficient(real_t p_fluid_viscosity_coefficient) {
    fluid_viscosity_coefficient = p_fluid_viscosity_coefficient;
}

void LiquidEffect2DViscosityDFSPH::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_fluid_viscosity_coefficient"), &LiquidEffect2DViscosityDFSPH::get_fluid_viscosity_coefficient);
    ClassDB::bind_method(D_METHOD("set_fluid_viscosity_coefficient", "fluid_viscosity_coefficient"), &LiquidEffect2DViscosityDFSPH::set_fluid_viscosity_coefficient);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fluid_viscosity_coefficient", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_fluid_viscosity_coefficient", "get_fluid_viscosity_coefficient");
}
