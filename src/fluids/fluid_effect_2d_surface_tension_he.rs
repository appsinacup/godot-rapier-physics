use godot::prelude::*;

use super::fluid_effect_2d::FluidEffect2DType;
use super::fluid_effect_2d::IFluidEffect2D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DTensionHE {
    #[export]
    fluid_tension_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect2D for FluidEffect2DTensionHE {
    fn get_fluid_effect_type(&self) -> FluidEffect2DType {
        FluidEffect2DType::FluidEffect2DSurfaceTensionHe
    }
}
#[godot_api]
impl IResource for FluidEffect2DTensionHE {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_tension_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
