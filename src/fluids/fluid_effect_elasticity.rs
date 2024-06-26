use godot::prelude::*;

use super::fluid_effect::FluidEffectType;
use super::fluid_effect::IFluidEffect;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffectElasticity {
    #[export]
    young_modulus: real,
    #[export]
    poisson_ratio: real,
    #[export]
    nonlinear_strain: bool,

    base: Base<Resource>,
}
impl IFluidEffect for FluidEffectElasticity {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        FluidEffectType::FluidEffectElasticity
    }
}
#[godot_api]
impl IResource for FluidEffectElasticity {
    fn init(base: Base<Resource>) -> Self {
        Self {
            young_modulus: 100.0,
            poisson_ratio: 0.3,
            nonlinear_strain: true,
            base,
        }
    }
}
