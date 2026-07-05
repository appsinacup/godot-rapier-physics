use godot::prelude::*;

fluid_effect!(FluidEffect3DSurfaceTensionWCSPH {
    fluid_tension_coefficient: real = 1.0,
    boundary_adhesion_coefficient: real = 0.0,
});
