use godot::prelude::*;

use super::fluid_effect_3d::FluidEffect3DType;
use super::fluid_effect_3d::IFluidEffect3D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3DViscosityXSPH {
    #[export]
    fluid_viscosity_coefficient: real,
    #[export]
    boundary_adhesion_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect3D for FluidEffect3DViscosityXSPH {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType {
        FluidEffect3DType::FluidEffect3DViscosityXsph
    }
}
#[godot_api]
impl IResource for FluidEffect3DViscosityXSPH {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            boundary_adhesion_coefficient: 0.0,
            base,
        }
    }
}
