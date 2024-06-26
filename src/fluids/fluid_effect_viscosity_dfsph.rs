use godot::prelude::*;

use super::fluid_effect::FluidEffectType;
use super::fluid_effect::IFluidEffect;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffectViscosityDFSPH {
    #[export]
    fluid_viscosity_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect for FluidEffectViscosityDFSPH {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        FluidEffectType::FluidEffectViscosityDfsph
    }
}
#[godot_api]
impl IResource for FluidEffectViscosityDFSPH {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            base,
        }
    }
}
