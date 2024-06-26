use godot::prelude::*;

use super::fluid_effect::FluidEffectType;
use super::fluid_effect::IFluidEffect;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffectViscosityXSPH {
    #[export]
    fluid_viscosity_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect for FluidEffectViscosityXSPH {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        FluidEffectType::FluidEffectViscosityXsph
    }
}
#[godot_api]
impl IResource for FluidEffectViscosityXSPH {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
