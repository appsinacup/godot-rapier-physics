use godot::prelude::*;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2DElasticity {
    #[export]
    #[var(pub)]
    young_modulus: real,
    #[export]
    #[var(pub)]
    poisson_ratio: real,
    #[export]
    #[var(pub)]
    nonlinear_strain: bool,

    base: Base<Resource>,
}
#[godot_api]
impl IResource for FluidEffect2DElasticity {
    fn init(base: Base<Resource>) -> Self {
        Self {
            young_modulus: 100.0,
            poisson_ratio: 0.3,
            nonlinear_strain: true,
            base,
        }
    }
}
