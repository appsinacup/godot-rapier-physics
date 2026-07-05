use godot::prelude::*;

fluid_effect!(FluidEffect3DElasticity {
    young_modulus: real = 100.0,
    poisson_ratio: real = 0.3,
    nonlinear_strain: bool = true,
});
