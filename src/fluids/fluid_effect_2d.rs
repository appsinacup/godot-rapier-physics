use godot::prelude::*;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect2D {
    base: Base<Resource>,
}
#[godot_api]
impl IResource for FluidEffect2D {
    fn init(base: Base<Resource>) -> Self {
        Self { base }
    }
}
