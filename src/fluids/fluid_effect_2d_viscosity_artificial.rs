use godot::prelude::*;

fluid_effect!(FluidEffect2DViscosityArtificial {
    fluid_viscosity_coefficient: real = 200.0,
    boundary_adhesion_coefficient: real = 0.0,
});
