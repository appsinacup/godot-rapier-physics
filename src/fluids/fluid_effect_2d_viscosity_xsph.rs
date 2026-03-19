use godot::prelude::*;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DViscosityXSPH {
    #[export]
    fluid_viscosity_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
#[godot_api]
impl IResource for FluidEffect2DViscosityXSPH {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
