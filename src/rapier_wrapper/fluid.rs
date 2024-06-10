use rapier::prelude::*;
use salva::math::Vector as SalvaVector;
use salva::object::*;
use salva::solver::*;

use super::shape::pixel_point_array_to_vec;
use crate::rapier_wrapper::prelude::*;
pub fn pixel_vector_array_to_vec(pixel_data: Vec<Vector<Real>>) -> Vec<SalvaVector<Real>> {
    let mut vec = Vec::<SalvaVector<Real>>::with_capacity(pixel_data.len());
    for pixel_point in pixel_data {
        let point = &vector_pixels_to_meters(pixel_point);
        let salva_vector = SalvaVector::<Real>::from_data(point.data);
        vec.push(salva_vector);
    }
    vec
}
pub fn fluid_create(world_handle: Handle, density: Real) -> HandleDouble {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        let particle_radius = physics_world.fluids_pipeline.liquid_world.particle_radius();
        let fluid = Fluid::new(Vec::new(), particle_radius, density);
        return fluid_handle_to_handle(physics_world.fluids_pipeline.liquid_world.add_fluid(fluid));
    }
    invalid_handle_double()
}
pub fn fluid_change_density(world_handle: Handle, fluid_handle: HandleDouble, density: Real) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            fluid.density0 = density;
        }
    }
}
pub fn fluid_change_points_and_velocities(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    pixel_points: Vec<Vector<Real>>,
    velocity_points: Vec<Vector<Real>>,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        let points = pixel_point_array_to_vec(pixel_points);
        let velocities = pixel_vector_array_to_vec(velocity_points);
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let points_len = points.len();
            let mut accelerations: Vec<_> = std::iter::repeat(SalvaVector::zeros())
                .take(points_len)
                .collect();
            fluid.positions = points;
            // copy back the accelerations that were before, if they exist
            for i in 0..fluid.accelerations.len() {
                if fluid.accelerations.len() > i {
                    accelerations[i] = fluid.accelerations[i];
                }
            }
            fluid.velocities = velocities;
            fluid.accelerations = accelerations;
            fluid.volumes = std::iter::repeat(fluid.default_particle_volume())
                .take(points_len)
                .collect();
        }
    }
}
pub fn fluid_change_points(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    pixel_points: Vec<Vector<Real>>,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        let points = pixel_point_array_to_vec(pixel_points);
        let point_count = points.len();
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let mut velocities: Vec<_> = std::iter::repeat(SalvaVector::zeros())
                .take(point_count)
                .collect();
            let mut accelerations: Vec<_> = std::iter::repeat(SalvaVector::zeros())
                .take(point_count)
                .collect();
            fluid.positions = points;
            // copy back the velocities and accelerations that were before, if they exist
            for i in 0..point_count {
                if fluid.velocities.len() > i {
                    velocities[i] = fluid.velocities[i];
                }
                if fluid.accelerations.len() > i {
                    accelerations[i] = fluid.accelerations[i];
                }
            }
            fluid.velocities = velocities;
            fluid.accelerations = accelerations;
            fluid.volumes = std::iter::repeat(fluid.default_particle_volume())
                .take(point_count)
                .collect();
        }
    }
}
pub fn fluid_delete_points(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    indexes: &usize,
    indexes_count: usize,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            unsafe {
                let indexes_raw = std::slice::from_raw_parts(indexes, indexes_count);
                // create mask from array of indexes
                let mut mask = vec![false; fluid.positions.len()];
                for i in 0..indexes_count {
                    if fluid.positions.len() <= indexes_raw[i] {
                        continue;
                    }
                    mask[indexes_raw[i]] = true;
                }
                let mut i = 0;
                // remove all points that are not in the mask
                fluid.positions.retain(|_| {
                    let delete = mask[i];
                    i += 1;
                    !delete
                });
                let mut i = 0;
                fluid.velocities.retain(|_| {
                    let delete = mask[i];
                    i += 1;
                    !delete
                });
                let mut i = 0;
                fluid.accelerations.retain(|_| {
                    let delete = mask[i];
                    i += 1;
                    !delete
                });
                let mut i = 0;
                fluid.volumes.retain(|_| {
                    let delete = mask[i];
                    i += 1;
                    !delete
                });
            }
        }
    }
}
pub fn fluid_add_points_and_velocities(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    pixel_points: Vec<Vector<Real>>,
    velocity_points: Vec<Vector<Real>>,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let points = pixel_point_array_to_vec(pixel_points);
            let velocities = pixel_vector_array_to_vec(velocity_points);
            // add old positions from fluid
            fluid.positions.extend_from_slice(&points);
            fluid.velocities.extend_from_slice(&velocities);
            let new_point_count = fluid.positions.len();
            let mut accelerations: Vec<_> = std::iter::repeat(SalvaVector::zeros())
                .take(new_point_count)
                .collect();
            // copy back the accelerations that were before, if they exist
            for i in 0..fluid.accelerations.len() {
                accelerations[i] = fluid.accelerations[i];
            }
            fluid.accelerations = accelerations;
            fluid.volumes = std::iter::repeat(fluid.default_particle_volume())
                .take(new_point_count)
                .collect();
        }
    }
}
pub fn fluid_get_points(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    pixel_points: &mut Vector<Real>,
    point_count: usize,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            unsafe {
                let points_raw = std::slice::from_raw_parts_mut(pixel_points, point_count);
                for i in 0..points_raw.len() {
                    if fluid.positions.len() <= i {
                        return;
                    }
                    points_raw[i].x = meters_to_pixels(fluid.positions[i].x);
                    points_raw[i].y = meters_to_pixels(fluid.positions[i].y);
                }
            }
        }
    }
}
pub fn fluid_get_velocities(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    pixel_velocities: &mut Vector<Real>,
    velocity_count: usize,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            unsafe {
                let velocity_raw = std::slice::from_raw_parts_mut(pixel_velocities, velocity_count);
                for i in 0..velocity_raw.len() {
                    if fluid.velocities.len() <= i {
                        return;
                    }
                    velocity_raw[i].x = meters_to_pixels(fluid.velocities[i].x);
                    velocity_raw[i].y = meters_to_pixels(fluid.velocities[i].y);
                }
            }
        }
    }
}
pub fn fluid_get_accelerations(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    pixel_acceleration: &mut Vector<Real>,
    acceleration_count: usize,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            unsafe {
                let acceleration_raw =
                    std::slice::from_raw_parts_mut(pixel_acceleration, acceleration_count);
                for i in 0..acceleration_raw.len() {
                    if fluid.accelerations.len() <= i {
                        return;
                    }
                    acceleration_raw[i].x = meters_to_pixels(fluid.accelerations[i].x);
                    acceleration_raw[i].y = meters_to_pixels(fluid.accelerations[i].y);
                }
            }
        }
    }
}
pub fn fluid_clear_effects(world_handle: Handle, fluid_handle: HandleDouble) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            fluid.nonpressure_forces.clear();
        }
    }
}
pub fn fluid_add_effect_elasticity(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    young_modulus: Real,
    poisson_ratio: Real,
    nonlinear_strain: bool,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: Becker2009Elasticity =
                Becker2009Elasticity::new(young_modulus, poisson_ratio, nonlinear_strain);
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_add_effect_surface_tension_akinci(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    fluid_tension_coefficient: Real,
    boundary_adhesion_coefficient: Real,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: Akinci2013SurfaceTension = Akinci2013SurfaceTension::new(
                fluid_tension_coefficient,
                boundary_adhesion_coefficient,
            );
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_add_effect_surface_tension_he(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    fluid_tension_coefficient: Real,
    boundary_adhesion_coefficient: Real,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: He2014SurfaceTension =
                He2014SurfaceTension::new(fluid_tension_coefficient, boundary_adhesion_coefficient);
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_add_effect_surface_tension_wcsph(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    fluid_tension_coefficient: Real,
    boundary_adhesion_coefficient: Real,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: WCSPHSurfaceTension =
                WCSPHSurfaceTension::new(fluid_tension_coefficient, boundary_adhesion_coefficient);
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_add_effect_viscosity_artificial(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    fluid_viscosity_coefficient: Real,
    boundary_viscosity_coefficient: Real,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: ArtificialViscosity = ArtificialViscosity::new(
                fluid_viscosity_coefficient,
                boundary_viscosity_coefficient,
            );
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_add_effect_viscosity_dfsph(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    fluid_viscosity_coefficient: Real,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: DFSPHViscosity = DFSPHViscosity::new(fluid_viscosity_coefficient);
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_add_effect_viscosity_xsph(
    world_handle: Handle,
    fluid_handle: HandleDouble,
    fluid_viscosity_coefficient: Real,
    boundary_viscosity_coefficient: Real,
) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: XSPHViscosity =
                XSPHViscosity::new(fluid_viscosity_coefficient, boundary_viscosity_coefficient);
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }
}
pub fn fluid_destroy(world_handle: Handle, fluid_handle: HandleDouble) {
    let physics_engine = physics_engine();
    if let Some(physics_world) = physics_engine.get_world(world_handle) {
        if let Some(_fluid) = physics_world
            .fluids_pipeline
            .liquid_world
            .fluids_mut()
            .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            // TODO reenable after the function is fixed https://github.com/dimforge/salva/pull/37/files
            //physics_world.fluids_pipeline.liquid_world.remove_fluid(handle_to_fluid_handle(fluid_handle));
        }
    }
}
