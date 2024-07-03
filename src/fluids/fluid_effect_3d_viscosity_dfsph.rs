use godot::prelude::*;

use super::fluid_effect_3d::FluidEffect3DType;
use super::fluid_effect_3d::IFluidEffect3D;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3DViscosityDFSPH {
    #[export]
    fluid_viscosity_coefficient: real,

    base: Base<Resource>,
}
impl IFluidEffect3D for FluidEffect3DViscosityDFSPH {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType {
        FluidEffect3DType::FluidEffect3DViscosityDfsph
    }
}
#[godot_api]
impl IResource for FluidEffect3DViscosityDFSPH {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_viscosity_coefficient: 1.0,
            base,
        }
    }
}
