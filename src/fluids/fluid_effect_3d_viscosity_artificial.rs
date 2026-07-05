use godot::prelude::*;

fluid_effect!(FluidEffect3DViscosityArtificial {
    fluid_viscosity_coefficient: real = 1.0,
    boundary_adhesion_coefficient: real = 0.0,
});
