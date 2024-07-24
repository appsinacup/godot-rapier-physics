#[cfg(feature = "dim2")]
pub type FluidEffectElasticity = super::fluid_effect_2d_elasticity::FluidEffect2DElasticity;
#[cfg(feature = "dim3")]
pub type FluidEffectElasticity = super::fluid_effect_3d_elasticity::FluidEffect3DElasticity;
#[cfg(feature = "dim2")]
pub type FluidEffectSurfaceTensionAKINCI =
    super::fluid_effect_2d_surface_tension_akinci::FluidEffect2DSurfaceTensionAKINCI;
#[cfg(feature = "dim3")]
pub type FluidEffectSurfaceTensionAKINCI =
    super::fluid_effect_3d_surface_tension_akinci::FluidEffect3DSurfaceTensionAKINCI;
#[cfg(feature = "dim2")]
pub type FluidEffectSurfaceTensionHE =
    super::fluid_effect_2d_surface_tension_he::FluidEffect2DSurfaceTensionHE;
#[cfg(feature = "dim3")]
pub type FluidEffectSurfaceTensionHE =
    super::fluid_effect_3d_surface_tension_he::FluidEffect3DSurfaceTensionHE;
#[cfg(feature = "dim2")]
pub type FluidEffectSurfaceTensionWCSPH =
    super::fluid_effect_2d_surface_tension_wcsph::FluidEffect2DSurfaceTensionWCSPH;
#[cfg(feature = "dim3")]
pub type FluidEffectSurfaceTensionWCSPH =
    super::fluid_effect_3d_surface_tension_wcsph::FluidEffect3DSurfaceTensionWCSPH;
#[cfg(feature = "dim2")]
pub type FluidEffectViscosityArtificial =
    super::fluid_effect_2d_viscosity_artificial::FluidEffect2DViscosityArtificial;
#[cfg(feature = "dim3")]
pub type FluidEffectViscosityArtificial =
    super::fluid_effect_3d_viscosity_artificial::FluidEffect3DViscosityArtificial;
#[cfg(feature = "dim2")]
pub type FluidEffectViscosityDFSPH =
    super::fluid_effect_2d_viscosity_dfsph::FluidEffect2DViscosityDFSPH;
#[cfg(feature = "dim3")]
pub type FluidEffectViscosityDFSPH =
    super::fluid_effect_3d_viscosity_dfsph::FluidEffect3DViscosityDFSPH;
#[cfg(feature = "dim2")]
pub type FluidEffectViscosityXSPH =
    super::fluid_effect_2d_viscosity_xsph::FluidEffect2DViscosityXSPH;
#[cfg(feature = "dim3")]
pub type FluidEffectViscosityXSPH =
    super::fluid_effect_3d_viscosity_xsph::FluidEffect3DViscosityXSPH;
#[cfg(feature = "dim2")]
pub type Fluid = super::fluid_2d::Fluid2D;
#[cfg(feature = "dim3")]
pub type Fluid = super::fluid_3d::Fluid3D;
