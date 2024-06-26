use godot::prelude::*;

use super::fluid_effect::FluidEffectType;
use super::fluid_effect::IFluidEffect;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffectViscosityArtificial {
    #[export]
    fluid_viscosity_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect for FluidEffectViscosityArtificial {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        FluidEffectType::FluidEffectViscosityArtificial
    }
}
#[godot_api]
impl IResource for FluidEffectViscosityArtificial {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
