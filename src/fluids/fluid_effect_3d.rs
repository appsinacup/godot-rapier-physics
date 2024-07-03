use godot::prelude::*;
pub trait IFluidEffect3D {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType;
}
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3D {
    fluid_effect_type: FluidEffect3DType,

    base: Base<Resource>,
}
#[derive(GodotConvert, Var, Export, Clone)]
#[godot(via = GString)]
pub enum FluidEffect3DType {
    FluidEffect3DElasticity = 0,
    FluidEffect3DSurfaceTensionAkinci = 1,
    FluidEffect3DSurfaceTensionHe = 2,
    FluidEffect3DSurfaceTensionWcsph = 3,
    FluidEffect3DViscosityArtificial = 4,
    FluidEffect3DViscosityDfsph = 5,
    FluidEffect3DViscosityXsph = 6,
    None = 7,
}
#[godot_api]
impl IResource for FluidEffect3D {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_effect_type: FluidEffect3DType::None,
            base,
        }
    }
}
impl IFluidEffect3D for FluidEffect3D {
    fn get_fluid_effect_type(&self) -> FluidEffect3DType {
        self.fluid_effect_type.clone()
    }
}
