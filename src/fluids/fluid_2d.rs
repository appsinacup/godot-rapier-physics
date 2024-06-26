use godot::prelude::*;

use crate::servers::rapier_project_settings::RapierProjectSettings;
use crate::types::*;
#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Fluid2D {
    rid: Rid,
    #[export]
    radius: real,
    #[export]
    debug_draw: bool,
    #[export]
    density: real,
    #[export]
    lifetime: real,
    #[export]
    effects: Array<Option<Gd<Resource>>>,

    points: PackedVectorArray,
    create_times: PackedFloat32Array,
    base: Base<Node2D>,
}
#[godot_api]
impl INode2D for Fluid2D {
    fn init(base: Base<Node2D>) -> Self {
        Self {
            rid: Rid::Invalid,
            radius: RapierProjectSettings::get_fluid_particle_radius(),
            debug_draw: false,
            density: 1.0,
            lifetime: 0.0,
            effects: Array::new(),
            points: PackedVectorArray::new(),
            create_times: PackedFloat32Array::new(),
            base,
        }
    }
}
