use std::borrow::BorrowMut;

use salva2d::integrations::rapier::FluidsPipeline;
use rapier2d::prelude::*;
use salva2d::object::*;
use salva2d::solver::Becker2009Elasticity;
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
pub extern "C" fn fluid_create(world_handle : Handle, density: Real) -> Handle {
	let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let particle_radius = physics_world.fluids_pipeline.liquid_world.particle_radius();
    let fluid = Fluid::new(Vec::new(), particle_radius, density);
	return fluid_handle_to_handle(physics_world.fluids_pipeline.liquid_world.add_fluid(fluid));
}

#[no_mangle]
pub extern "C" fn fluid_change_density(world_handle : Handle, fluid_handle: Handle, density: Real) {
	let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let fluid = physics_world.fluids_pipeline.liquid_world.fluids_mut().get_mut(handle_to_fluid_handle(fluid_handle));
    assert!(fluid.is_some());
    let fluid = fluid.unwrap();
    fluid.density0 = density;
}

#[no_mangle]
pub extern "C" fn fluid_change_points(world_handle : Handle, fluid_handle: Handle, pixel_points : &Vector, point_count : usize) {
	let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let points = pixel_point_array_to_vec(pixel_points, point_count);
    let fluid = physics_world.fluids_pipeline.liquid_world.fluids_mut().get_mut(handle_to_fluid_handle(fluid_handle));
    assert!(fluid.is_some());
    let fluid = fluid.unwrap();
    let velocities: Vec<_> = std::iter::repeat(salva2d::math::Vector::zeros())
        .take(point_count)
        .collect();
    fluid.positions = points;
    fluid.velocities = velocities;
}

#[no_mangle]
pub extern "C" fn fluid_add_effect_elasticity(world_handle : Handle, fluid_handle: Handle, young_modulus: Real, poisson_ratio: Real, nonlinear_strain: bool) {
	let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let fluid = physics_world.fluids_pipeline.liquid_world.fluids_mut().get_mut(handle_to_fluid_handle(fluid_handle));
    assert!(fluid.is_some());
    let fluid = fluid.unwrap();
    let elasticity: Becker2009Elasticity = Becker2009Elasticity::new(young_modulus, poisson_ratio, nonlinear_strain);
    fluid.nonpressure_forces.push(Box::new(elasticity));
}

#[no_mangle]
pub extern "C" fn fluid_change_effect_elasticity(world_handle : Handle, index: usize, fluid_handle: Handle, young_modulus: Real, poisson_ratio: Real, nonlinear_strain: bool) {
	let mut physics_engine = singleton().lock().unwrap();
	let physics_world = physics_engine.get_world(world_handle);
    let fluid = physics_world.fluids_pipeline.liquid_world.fluids_mut().get_mut(handle_to_fluid_handle(fluid_handle));
    assert!(fluid.is_some());
    let fluid = fluid.unwrap();
    let elasticity: Becker2009Elasticity = Becker2009Elasticity::new(young_modulus, poisson_ratio, nonlinear_strain);
    //let force: &mut Becker2009Elasticity = fluid.nonpressure_forces[index].borrow_mut();
    //let
}
