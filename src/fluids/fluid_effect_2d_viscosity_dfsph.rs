use godot::prelude::*;

use super::fluid_effect_2d::FluidEffect2DType;
use super::fluid_effect_2d::IFluidEffect2D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DViscosityDFSPH {
    #[export]
    fluid_viscosity_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect2D for FluidEffect2DViscosityDFSPH {
    fn get_fluid_effect_type(&self) -> FluidEffect2DType {
        FluidEffect2DType::FluidEffect2DViscosityDfsph
    }
}
#[godot_api]
impl IResource for FluidEffect2DViscosityDFSPH {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            base,
        }
    }
}
