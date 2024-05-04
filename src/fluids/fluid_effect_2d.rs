use godot::{builtin::{meta::{FromGodot, ToGodot}, Variant}, engine::{IResource, Resource}, obj::Base, register::{godot_api, GodotClass}};
use godot::builtin::meta::GodotConvert;

#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2D {
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
impl IResource for FluidEffect2D {
    fn init(base: Base<Resource>) -> Self {
        Self {
            fluid_effect_type: FluidEffectType::FluidEffectElasticity,
            base,
        }
    }
}


