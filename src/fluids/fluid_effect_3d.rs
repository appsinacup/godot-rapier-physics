use godot::prelude::*;
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct FluidEffect3D {
    base: Base<Resource>,
}
#[godot_api]
impl IResource for FluidEffect3D {
    fn init(base: Base<Resource>) -> Self {
        Self { base }
    }
}
