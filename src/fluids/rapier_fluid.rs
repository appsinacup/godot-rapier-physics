use std::collections::VecDeque;

use godot::prelude::*;

use super::types::*;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsSpaces;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::Vector;
//#[derive(Serialize, Deserialize, Debug)]
pub struct RapierFluid {
    id: RapierId,
    density: real,
    space: Rid,
    space_id: WorldHandle,
    effects: Array<Option<Gd<Resource>>>,
    fluid_handle: HandleDouble,
    points: Vec<Vector>,
    velocities: Vec<Vector>,
    accelerations: Vec<Vector>,
    interaction_groups: salva::object::interaction_groups::InteractionGroups,
}
impl RapierFluid {
    pub fn new(id: RapierId) -> Self {
        Self {
            id,
            density: 1.0,
            space: Rid::Invalid,
            space_id: WorldHandle::default(),
            effects: Array::default(),
            fluid_handle: invalid_handle_double(),
            points: Vec::new(),
            velocities: Vec::new(),
            accelerations: Vec::new(),
            interaction_groups: salva::object::interaction_groups::InteractionGroups::new(
                salva::object::interaction_groups::Group::GROUP_1,
                salva::object::interaction_groups::Group::GROUP_1,
            ),
        }
    }

    pub fn is_valid(&self) -> bool {
        self.fluid_handle.is_valid() && self.space_id != WorldHandle::default()
    }

    pub fn set_points(&mut self, points: Vec<Vector>, physics_engine: &mut PhysicsEngine) {
        self.points = points;
        self.velocities.resize(self.points.len(), Vector::default());
        self.accelerations
            .resize(self.points.len(), Vector::default());
        if self.is_valid() {
            let rapier_points = self
                .points
                .iter()
                .map(|vec: &crate::types::Vector| vector_to_rapier(*vec))
                .collect::<Vec<_>>();
            physics_engine.fluid_change_points(self.space_id, self.fluid_handle, &rapier_points);
        }
    }

    pub fn set_interaction_groups(
        &mut self,
        groups: salva::object::interaction_groups::InteractionGroups,
        physics_engine: &mut PhysicsEngine,
    ) {
        if self.is_valid() {
            physics_engine.fluid_change_interaction_groups(
                self.space_id,
                self.fluid_handle,
                groups,
            );
        }
    }

    pub fn get_interaction_groups(&self) -> salva::object::interaction_groups::InteractionGroups {
        self.interaction_groups
    }

    pub fn set_points_and_velocities(
        &mut self,
        points: Vec<Vector>,
        velocities: Vec<Vector>,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.points = points;
        self.velocities = velocities;
        self.accelerations
            .resize(self.points.len(), Vector::default());
        if self.is_valid() {
            let rapier_points = self
                .points
                .iter()
                .map(|vec: &crate::types::Vector| vector_to_rapier(*vec))
                .collect::<Vec<_>>();
            let rapier_velocities = self
                .velocities
                .iter()
                .map(|vec: &crate::types::Vector| vector_to_rapier(*vec))
                .collect::<Vec<_>>();
            physics_engine.fluid_change_points_and_velocities(
                self.space_id,
                self.fluid_handle,
                &rapier_points,
                &rapier_velocities,
            );
        }
    }

    pub fn add_points_and_velocities(
        &mut self,
        points: Vec<Vector>,
        velocities: Vec<Vector>,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.points.extend(points.clone());
        self.velocities.extend(velocities.clone());
        self.accelerations
            .resize(self.points.len(), Vector::default());
        if self.is_valid() {
            let rapier_points = points
                .iter()
                .map(|vec: &crate::types::Vector| vector_to_rapier(*vec))
                .collect::<Vec<_>>();
            let rapier_velocities = velocities
                .iter()
                .map(|vec: &crate::types::Vector| vector_to_rapier(*vec))
                .collect::<Vec<_>>();
            physics_engine.fluid_add_points_and_velocities(
                self.space_id,
                self.fluid_handle,
                &rapier_points,
                &rapier_velocities,
            );
        }
    }

    pub fn delete_points(&mut self, indices: Vec<i32>, physics_engine: &mut PhysicsEngine) {
        let removals = indices.clone().into_iter().collect::<VecDeque<_>>();
        for index in removals {
            self.points.remove(index as usize);
            self.velocities.remove(index as usize);
            self.accelerations.remove(index as usize);
        }
        physics_engine.fluid_delete_points(self.space_id, self.fluid_handle, indices);
    }

    pub fn get_points(&mut self, physics_engine: &mut PhysicsEngine) -> &Vec<Vector> {
        if self.is_valid() {
            self.points = physics_engine.fluid_get_points(self.space_id, self.fluid_handle);
        }
        &self.points
    }

    pub fn get_velocities(&mut self, physics_engine: &mut PhysicsEngine) -> &Vec<Vector> {
        if self.is_valid() {
            self.velocities = physics_engine.fluid_get_velocities(self.space_id, self.fluid_handle);
        }
        &self.velocities
    }

    pub fn get_accelerations(&mut self, physics_engine: &mut PhysicsEngine) -> &Vec<Vector> {
        if self.is_valid() {
            self.accelerations =
                physics_engine.fluid_get_accelerations(self.space_id, self.fluid_handle);
        }
        &self.accelerations
    }

    fn set_effect(&self, effect: &Gd<Resource>, physics_engine: &mut PhysicsEngine) {
        if let Ok(effect) = effect.clone().try_cast::<FluidEffectElasticity>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_elasticity(
                self.space_id,
                self.fluid_handle,
                effect.get_young_modulus(),
                effect.get_poisson_ratio(),
                effect.get_nonlinear_strain(),
            );
        } else if let Ok(effect) = effect.clone().try_cast::<FluidEffectSurfaceTensionAKINCI>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_surface_tension_akinci(
                self.space_id,
                self.fluid_handle,
                effect.get_fluid_tension_coefficient(),
                effect.get_boundary_adhesion_coefficient(),
            );
        } else if let Ok(effect) = effect.clone().try_cast::<FluidEffectSurfaceTensionHE>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_surface_tension_he(
                self.space_id,
                self.fluid_handle,
                effect.get_fluid_tension_coefficient(),
                effect.get_boundary_adhesion_coefficient(),
            );
        } else if let Ok(effect) = effect.clone().try_cast::<FluidEffectSurfaceTensionWCSPH>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_surface_tension_wcsph(
                self.space_id,
                self.fluid_handle,
                effect.get_fluid_tension_coefficient(),
                effect.get_boundary_adhesion_coefficient(),
            );
        } else if let Ok(effect) = effect.clone().try_cast::<FluidEffectViscosityArtificial>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_viscosity_artificial(
                self.space_id,
                self.fluid_handle,
                effect.get_fluid_viscosity_coefficient(),
                effect.get_boundary_adhesion_coefficient(),
            );
        } else if let Ok(effect) = effect.clone().try_cast::<FluidEffectViscosityDFSPH>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_viscosity_dfsph(
                self.space_id,
                self.fluid_handle,
                effect.get_fluid_viscosity_coefficient(),
            );
        } else if let Ok(effect) = effect.clone().try_cast::<FluidEffectViscosityXSPH>() {
            let effect = effect.bind();
            physics_engine.fluid_add_effect_viscosity_xsph(
                self.space_id,
                self.fluid_handle,
                effect.get_fluid_viscosity_coefficient(),
                effect.get_boundary_adhesion_coefficient(),
            );
        }
    }

    pub fn set_effects(
        &mut self,
        effects: Array<Option<Gd<Resource>>>,
        physics_engine: &mut PhysicsEngine,
    ) {
        if self.effects != effects {
            self.effects = effects.clone();
        }
        if self.is_valid() {
            physics_engine.fluid_clear_effects(self.space_id, self.fluid_handle);
            for effect in self.effects.iter_shared().flatten() {
                let effect = effect.clone();
                self.set_effect(&effect, physics_engine);
            }
        }
    }

    pub fn set_density(&mut self, density: f32, physics_engine: &mut PhysicsEngine) {
        self.density = density;
        if self.is_valid() {
            physics_engine.fluid_change_density(self.space_id, self.fluid_handle, density);
        }
    }

    pub fn set_space(
        &mut self,
        p_space: Rid,
        physics_spaces: &mut PhysicsSpaces,
        physics_engine: &mut PhysicsEngine,
    ) {
        if self.space == p_space {
            return;
        }
        // remove from old space
        if self.is_valid() {
            physics_engine.fluid_destroy(self.space_id, self.fluid_handle);
        }
        self.fluid_handle = invalid_handle_double();
        self.space_id = WorldHandle::default();
        if let Some(space) = physics_spaces.get(&p_space) {
            self.space = p_space;
            self.space_id = space.get_state().get_id();
            if self.space_id != WorldHandle::default() {
                self.fluid_handle = physics_engine.fluid_create(
                    self.space_id,
                    self.density,
                    salva::object::interaction_groups::InteractionGroups::all(),
                );
            }
            self.set_points(self.points.clone(), physics_engine);
            self.set_effects(self.effects.clone(), physics_engine);
        }
    }

    pub fn destroy_fluid(&mut self, physics_engine: &mut PhysicsEngine) {
        if self.is_valid() {
            physics_engine.fluid_destroy(self.space_id, self.fluid_handle);
            self.fluid_handle = invalid_handle_double();
            self.space_id = WorldHandle::default()
        }
    }

    pub fn get_id(&self) -> RapierId {
        self.id
    }
}
