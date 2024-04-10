use salva2d::integrations::rapier::FluidsPipeline;
use rapier2d::prelude::*;
use salva2d::object::*;
use salva2d::*;
use crate::convert::*;
use crate::handle::*;
use crate::vector::Vector;
use crate::user_data::UserData;
use crate::physics_world::*;
use crate::shape::*;
use crate::collider::*;
use crate::physics_world::*;

#[no_mangle]
pub extern "C" fn fluid_create(world_handle : Handle, density: Real, pixel_points : &Vector, point_count : usize) -> FluidHandle {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let particle_radius = physics_world.fluids_pipeline.liquid_world.particle_radius();
    let points = pixel_point_array_to_vec(pixel_points, point_count);
    let fluid = Fluid::new(points, particle_radius, density);
	return physics_world.fluids_pipeline.liquid_world.add_fluid(fluid);
}

#[no_mangle]
pub extern "C" fn fluid_change(world_handle : Handle, density: Real, pixel_points : &Vector, point_count : usize) -> FluidHandle {
	let mut physics_engine = SINGLETON.lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let particle_radius = physics_world.fluids_pipeline.liquid_world.particle_radius();
    let points = pixel_point_array_to_vec(pixel_points, point_count);
    let fluid = Fluid::new(points, particle_radius, density);
	return physics_world.fluids_pipeline.liquid_world.add_fluid(fluid);
}

