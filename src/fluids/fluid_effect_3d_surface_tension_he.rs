use godot::prelude::*;

use super::fluid_effect_3d::FluidEffect3DType;
use super::fluid_effect_3d::IFluidEffect3D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3DSurfaceTensionHE {
    #[export]
    fluid_tension_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect3D for FluidEffect3DSurfaceTensionHE {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType {
        FluidEffect3DType::FluidEffect3DSurfaceTensionHe
    }
}
#[godot_api]
impl IResource for FluidEffect3DSurfaceTensionHE {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_tension_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
