use godot::prelude::*;

use super::fluid_effect::FluidEffectType;
use super::fluid_effect::IFluidEffect;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffectTensionHE {
    #[export]
    fluid_tension_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect for FluidEffectTensionHE {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        FluidEffectType::FluidEffectSurfaceTensionHe
    }
}
#[godot_api]
impl IResource for FluidEffectTensionHE {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_tension_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
