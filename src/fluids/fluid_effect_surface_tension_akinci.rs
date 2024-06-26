use godot::prelude::*;

use super::fluid_effect::FluidEffectType;
use super::fluid_effect::IFluidEffect;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffectTensionAkinci {
    #[export]
    fluid_tension_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect for FluidEffectTensionAkinci {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        FluidEffectType::FluidEffectSurfaceTensionAkinci
    }
}
#[godot_api]
impl IResource for FluidEffectTensionAkinci {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_tension_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
