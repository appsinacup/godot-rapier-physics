use rapier::prelude::*;

use crate::rapier_wrapper::prelude::*;
pub fn shape_create_box(size: Vector<Real>) -> Handle {
    let shape = SharedShape::cuboid(0.5 * size.x, 0.5 * size.y, 0.5 * size.z);
    physics_engine().insert_shape(shape)
}
