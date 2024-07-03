use godot::prelude::*;

use super::fluid_effect_2d::FluidEffect2DType;
use super::fluid_effect_2d::IFluidEffect2D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DViscosityArtificial {
    #[export]
    fluid_viscosity_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect2D for FluidEffect2DViscosityArtificial {
    fn get_fluid_effect_type(&self) -> FluidEffect2DType {
        FluidEffect2DType::FluidEffect2DViscosityArtificial
    }
}
#[godot_api]
impl IResource for FluidEffect2DViscosityArtificial {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
