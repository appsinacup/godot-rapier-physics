use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect {
    fluid_effect_type: FluidEffectType,

    base: Base<Resource>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FluidEffectType {
    FluidEffectElasticity = 0,
    FluidEffectSurfaceTensionAkinci = 1,
    FluidEffectSurfaceTensionHe = 2,
    FluidEffectSurfaceTensionWcsph = 3,
    FluidEffectViscosityArtificial = 4,
    FluidEffectViscosityDfsph = 5,
    FluidEffectViscosityXsph = 6,
}

#[godot_api]
impl IResource for FluidEffect {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_effect_type: FluidEffectType::FluidEffectElasticity,
            base,
        }
    }
}

impl FluidEffect {
    fn get_fluid_effect_type(&self) -> FluidEffectType {
        self.fluid_effect_type
    }
}
