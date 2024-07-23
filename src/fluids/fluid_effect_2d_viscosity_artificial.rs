use godot::prelude::*;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DViscosityArtificial {
    #[export]
    fluid_viscosity_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
#[godot_api]
impl IResource for FluidEffect2DViscosityArtificial {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 200.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
