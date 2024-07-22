use godot::prelude::*;

use super::fluid_effect_3d::FluidEffect3DType;
use super::fluid_effect_3d::IFluidEffect3D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3DSurfaceTensionAKINCI {
    #[export]
    fluid_tension_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect3D for FluidEffect3DSurfaceTensionAKINCI {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType {
        FluidEffect3DType::FluidEffect3DSurfaceTensionAkinci
    }
}
#[godot_api]
impl IResource for FluidEffect3DSurfaceTensionAKINCI {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_tension_coefficient: 1000.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
