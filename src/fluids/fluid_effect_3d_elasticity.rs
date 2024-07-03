use godot::prelude::*;

use super::fluid_effect_3d::FluidEffect3DType;
use super::fluid_effect_3d::IFluidEffect3D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3DElasticity {
    #[export]
    young_modulus: real,
    #[export]
    poisson_ratio: real,
    #[export]
    nonlinear_strain: bool,

    base: Base<Resource>,
}
impl IFluidEffect3D for FluidEffect3DElasticity {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType {
        FluidEffect3DType::FluidEffect3DElasticity
    }
}
#[godot_api]
impl IResource for FluidEffect3DElasticity {
    fn init(base: Base<Resource>) -> Self {
        Self {
            young_modulus: 100.0,
            poisson_ratio: 0.3,
            nonlinear_strain: true,
            base,
        }
    }
}
