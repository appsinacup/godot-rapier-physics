#include "liquid_effect_2d_elasticity.h"

real_t LiquidEffect2DElasticity::get_young_modulus() const {
	return young_modulus;
}

void LiquidEffect2DElasticity::set_young_modulus(real_t p_young_modulus) {
	young_modulus = p_young_modulus;
}

real_t LiquidEffect2DElasticity::get_poisson_ratio() const {
	return poisson_ratio;
}

void LiquidEffect2DElasticity::set_poisson_ratio(real_t p_poisson_ratio) {
	poisson_ratio = p_poisson_ratio;
}

bool LiquidEffect2DElasticity::get_nonlinear_strain() const {
	return nonlinear_strain;
}

void LiquidEffect2DElasticity::set_nonlinear_strain(bool p_nonlinear_strain) {
	nonlinear_strain = p_nonlinear_strain;
}

void LiquidEffect2DElasticity::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_young_modulus"), &LiquidEffect2DElasticity::get_young_modulus);
	ClassDB::bind_method(D_METHOD("set_young_modulus", "young_modulus"), &LiquidEffect2DElasticity::set_young_modulus);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "young_modulus", PROPERTY_HINT_RANGE, U"0,500000,or_greater"), "set_young_modulus", "get_young_modulus");

	ClassDB::bind_method(D_METHOD("get_poisson_ratio"), &LiquidEffect2DElasticity::get_poisson_ratio);
	ClassDB::bind_method(D_METHOD("set_poisson_ratio", "poisson_ratio"), &LiquidEffect2DElasticity::set_poisson_ratio);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "poisson_ratio", PROPERTY_HINT_RANGE, U"0,1,or_greater"), "set_poisson_ratio", "get_poisson_ratio");

	ClassDB::bind_method(D_METHOD("get_nonlinear_strain"), &LiquidEffect2DElasticity::get_nonlinear_strain);
	ClassDB::bind_method(D_METHOD("set_nonlinear_strain", "nonlinear_strain"), &LiquidEffect2DElasticity::set_nonlinear_strain);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "nonlinear_strain"), "set_nonlinear_strain", "get_nonlinear_strain");
}
