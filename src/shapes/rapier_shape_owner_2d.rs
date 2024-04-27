use godot::builtin::Rid;

pub trait RapierShapeOwner2D {
    fn shape_changed(&self, p_shape: Rid);
    fn remove_shape(&self, p_shape: Rid);
}