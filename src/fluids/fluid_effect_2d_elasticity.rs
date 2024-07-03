use godot::prelude::*;

use super::fluid_effect_2d::FluidEffect2DType;
use super::fluid_effect_2d::IFluidEffect2D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DElasticity {
    #[export]
    young_modulus: real,
    #[export]
    poisson_ratio: real,
    #[export]
    nonlinear_strain: bool,

    base: Base<Resource>,
}
impl IFluidEffect2D for FluidEffect2DElasticity {
    fn get_fluid_effect_type(&self) -> FluidEffect2DType {
        FluidEffect2DType::FluidEffect2DElasticity
    }
}
#[godot_api]
impl IResource for FluidEffect2DElasticity {
    fn init(base: Base<Resource>) -> Self {
        Self {
            young_modulus: 100.0,
            poisson_ratio: 0.3,
            nonlinear_strain: true,
            base,
        }
    }
}
