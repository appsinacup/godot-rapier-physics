#include "liquid_effect_2d_elasticity.h"

	static void _bind_methods();
public:
	real_t get_young_modulus() const;
	void set_young_modulus(real_t p_young_modulus);

	real_t get_poisson_ratio() const;
	void set_poisson_ratio(real_t p_poisson_ratio);

	bool get_nonlinear_strain() const;
	void set_nonlinear_strain(bool p_nonlinear_strain);
