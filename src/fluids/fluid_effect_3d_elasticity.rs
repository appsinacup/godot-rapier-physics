use godot::prelude::*;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3DElasticity {
    #[export]
    young_modulus: real,
    #[export]
    poisson_ratio: real,
    #[export]
    nonlinear_strain: bool,

    base: Base<Resource>,
}
#[godot_api]
impl IResource for FluidEffect3DElasticity {
    fn init(base: Base<Resource>) -> Self {
        Self {
            young_modulus: 100.0,
            poisson_ratio: 0.3,
            nonlinear_strain: true,
            base,
        }
    }
}
