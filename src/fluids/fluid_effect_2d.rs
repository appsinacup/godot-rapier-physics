use godot::prelude::*;
pub trait IFluidEffect2D {
    fn get_fluid_effect_type(&self) -> FluidEffect2DType;
}
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2D {
    fluid_effect_type: FluidEffect2DType,

    base: Base<Resource>,
}
#[derive(GodotConvert, Var, Export, Clone)]
#[godot(via = GString)]
pub enum FluidEffect2DType {
    FluidEffect2DElasticity = 0,
    FluidEffect2DSurfaceTensionAKINCI = 1,
    FluidEffect2DSurfaceTensionHe = 2,
    FluidEffect2DSurfaceTensionWcsph = 3,
    FluidEffect2DViscosityArtificial = 4,
    FluidEffect2DViscosityDfsph = 5,
    FluidEffect2DViscosityXsph = 6,
    None = 7,
}
#[godot_api]
impl IResource for FluidEffect2D {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_effect_type: FluidEffect2DType::None,
            base,
        }
    }
}
impl IFluidEffect2D for FluidEffect2D {
    fn get_fluid_effect_type(&self) -> FluidEffect2DType {
        self.fluid_effect_type.clone()
    }
}
