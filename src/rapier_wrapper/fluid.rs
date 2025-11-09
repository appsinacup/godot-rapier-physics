use rapier::prelude::*;
use salva::object::*;
use salva::solver::*;

use super::shape::point_array_to_vec;
use crate::rapier_wrapper::prelude::*;
impl PhysicsEngine {
    pub fn fluid_create(
        &mut self,
        world_handle: WorldHandle,
        density: Real,
        interaction_groups: salva::object::interaction_groups::InteractionGroups,
    ) -> HandleDouble {
        if let Some(physics_world) = self.get_mut_world(world_handle) {
            let particle_radius = physics_world.fluids_pipeline.liquid_world.particle_radius();
            let fluid = Fluid::new(Vec::new(), particle_radius, density, interaction_groups);
            return fluid_handle_to_handle(
                physics_world.fluids_pipeline.liquid_world.add_fluid(fluid),
            );
        }
        invalid_handle_double()
    }

    pub fn fluid_change_density(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        density: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            fluid.density0 = density;
        }
    }

    pub fn fluid_change_interaction_groups(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        interaction_groups: salva::object::interaction_groups::InteractionGroups,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            fluid.interaction_groups = interaction_groups;
        }
    }

    pub fn fluid_change_points_and_velocities(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        points: &Vec<Vector<Real>>,
        velocity_points: &[Vector<Real>],
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let points = point_array_to_vec(points);
            // 1. Mark all existing particles for deletion.
            for i in 0..fluid.num_particles() {
                fluid.delete_particle_at_next_timestep(i);
            }
            // 2. Add the new particles.
            fluid.add_particles(&points, Some(velocity_points));
        }
    }

    pub fn fluid_change_points(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        points: &Vec<Vector<Real>>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let points = point_array_to_vec(points);
            // 1. Mark all existing particles for deletion.
            for i in 0..fluid.num_particles() {
                fluid.delete_particle_at_next_timestep(i);
            }
            // 2. Add the new particles with zero velocity.
            fluid.add_particles(&points, None);
        }
    }

    pub fn fluid_delete_points(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        indices: Vec<i32>,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            for index in indices {
                if index >= 0 && (index as usize) < fluid.num_particles() {
                    fluid.delete_particle_at_next_timestep(index as usize);
                }
            }
        }
    }

    pub fn fluid_add_points_and_velocities(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        points: &Vec<Vector<Real>>,
        velocity_points: &[Vector<Real>],
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let points = point_array_to_vec(points);
            fluid.add_particles(&points, Some(velocity_points));
        }
    }

    pub fn fluid_get_points(
        &self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
    ) -> Vec<crate::types::Vector> {
        let mut array = Vec::new();
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids()
                .get(handle_to_fluid_handle(fluid_handle))
        {
            for i in 0..fluid.positions.len() {
                array.push(vector_to_godot(fluid.positions[i].coords));
            }
        }
        array
    }

    pub fn fluid_get_velocities(
        &self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
    ) -> Vec<crate::types::Vector> {
        let mut array = Vec::new();
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids()
                .get(handle_to_fluid_handle(fluid_handle))
        {
            for i in 0..fluid.positions.len() {
                array.push(vector_to_godot(fluid.velocities[i]));
            }
        }
        array
    }

    pub fn fluid_get_accelerations(
        &self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
    ) -> Vec<crate::types::Vector> {
        let mut array = Vec::new();
        if let Some(physics_world) = self.get_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids()
                .get(handle_to_fluid_handle(fluid_handle))
        {
            for i in 0..fluid.positions.len() {
                array.push(vector_to_godot(fluid.accelerations[i]));
            }
        }
        array
    }

    pub fn fluid_get_particles_in_aabb(
        &self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        aabb: crate::types::Rect,
    ) -> Vec<i32> {
        let mut indices = Vec::new();
        if let Some(physics_world) = self.get_world(world_handle) {
            let salva_aabb = crate::rapier_wrapper::convert::aabb_to_salva_aabb(aabb);
            let liquid_world = &physics_world.fluids_pipeline.liquid_world;
            let r_fluid_handle = handle_to_fluid_handle(fluid_handle);
            for particle in liquid_world.particles_intersecting_aabb(salva_aabb) {
                match particle {
                    salva::object::ParticleId::FluidParticle(
                        found_fluid_handle,
                        particle_index,
                    ) => {
                        if found_fluid_handle == r_fluid_handle {
                            indices.push(particle_index as i32);
                        }
                    }
                    // We are only interested in fluid particles for this function.
                    salva::object::ParticleId::BoundaryParticle(..) => {}
                }
            }
        }
        indices
    }

    pub fn fluid_get_particles_in_ball(
        &self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        center: crate::types::Vector,
        radius: Real,
    ) -> Vec<i32> {
        let mut indices = Vec::new();
        if let Some(physics_world) = self.get_world(world_handle) {
            let godot_aabb = crate::types::Rect::new(
                center - crate::types::Vector::splat(radius),
                crate::types::Vector::splat(radius * 2.0),
            );
            let salva_aabb = crate::rapier_wrapper::convert::aabb_to_salva_aabb(godot_aabb);
            let liquid_world = &physics_world.fluids_pipeline.liquid_world;
            let r_fluid_handle = handle_to_fluid_handle(fluid_handle);
            let radius_sq = radius * radius;
            let r_center = vector_to_rapier(center);
            for particle in liquid_world.particles_intersecting_aabb(salva_aabb) {
                if let salva::object::ParticleId::FluidParticle(found_fluid_handle, particle_index) =
                    particle
                    && found_fluid_handle == r_fluid_handle
                    && let Some(fluid) = liquid_world.fluids().get(found_fluid_handle)
                {
                    let particle_pos = fluid.positions[particle_index].coords;
                    if (particle_pos - r_center).norm_squared() <= radius_sq {
                        indices.push(particle_index as i32);
                    }
                }
            }
        }
        indices
    }

    pub fn fluid_clear_effects(&mut self, world_handle: WorldHandle, fluid_handle: HandleDouble) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            fluid.nonpressure_forces.clear();
        }
    }

    pub fn fluid_add_effect_elasticity(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        young_modulus: Real,
        poisson_ratio: Real,
        nonlinear_strain: bool,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
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

    pub fn fluid_add_effect_surface_tension_akinci(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        fluid_tension_coefficient: Real,
        boundary_adhesion_coefficient: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
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

    pub fn fluid_add_effect_surface_tension_he(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        fluid_tension_coefficient: Real,
        boundary_adhesion_coefficient: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
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

    pub fn fluid_add_effect_surface_tension_wcsph(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        fluid_tension_coefficient: Real,
        boundary_adhesion_coefficient: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
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

    pub fn fluid_add_effect_viscosity_artificial(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        fluid_viscosity_coefficient: Real,
        boundary_viscosity_coefficient: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
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

    pub fn fluid_add_effect_viscosity_dfsph(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        fluid_viscosity_coefficient: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            let effect: DFSPHViscosity = DFSPHViscosity::new(fluid_viscosity_coefficient);
            fluid.nonpressure_forces.push(Box::new(effect));
        }
    }

    pub fn fluid_add_effect_viscosity_xsph(
        &mut self,
        world_handle: WorldHandle,
        fluid_handle: HandleDouble,
        fluid_viscosity_coefficient: Real,
        boundary_viscosity_coefficient: Real,
    ) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(fluid) = physics_world
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

    pub fn fluid_destroy(&mut self, world_handle: WorldHandle, fluid_handle: HandleDouble) {
        if let Some(physics_world) = self.get_mut_world(world_handle)
            && let Some(_fluid) = physics_world
                .fluids_pipeline
                .liquid_world
                .fluids_mut()
                .get_mut(handle_to_fluid_handle(fluid_handle))
        {
            physics_world
                .fluids_pipeline
                .liquid_world
                .remove_fluid(handle_to_fluid_handle(fluid_handle));
        }
    }
}
