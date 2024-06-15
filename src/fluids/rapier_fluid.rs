use std::collections::VecDeque;

use godot::prelude::*;
use rapier::math::{Real, Vector};

use super::fluid_effect::FluidEffect;
use crate::rapier_wrapper::prelude::*;
pub struct RapierFluid {
    rid: Rid,
    enabled: bool,
    density: f64,
    space: Rid,
    effects: Array<Gd<FluidEffect>>,
    fluid_handle: HandleDouble,
    points: Vec<Vector<Real>>,
    velocities: Vec<Vector<Real>>,
    accelerations: Vec<Vector<Real>>,
}
impl RapierFluid {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            enabled: true,
            density: 1.0,
            space: Rid::Invalid,
            effects: Array::default(),
            fluid_handle: invalid_handle_double(),
            points: Vec::new(),
            velocities: Vec::new(),
            accelerations: Vec::new(),
        }
    }

    pub fn set_points(&mut self, points: Vec<Vector<Real>>) {
        self.points = points;
    }

    pub fn set_points_and_velocities(&mut self, points: Vec<Vector<Real>>, velocities: Vec<Vector<Real>>) {
        self.points = points;
        self.velocities = velocities;
    }

    pub fn add_points_and_velocities(&mut self, points: Vec<Vector<Real>>, velocities: Vec<Vector<Real>>) {
        self.points.extend(points);
        self.velocities.extend(velocities);
    }

    pub fn delete_points(&mut self, indices: Vec<i32>) {
        let mut removals = indices.into_iter().collect::<VecDeque<_>>();
        removals.make_contiguous().sort_unstable();
        removals.make_contiguous().reverse();
        for index in removals {
            self.points.remove(index as usize);
            self.velocities.remove(index as usize);
            self.accelerations.remove(index as usize);
        }
    }

    pub fn get_points(&self) -> Vec<Vector<Real>> {
        self.points.clone()
    }

    pub fn get_velocities(&self) -> Vec<Vector<Real>> {
        self.velocities.clone()
    }

    pub fn get_accelerations(&self) -> Vec<Vector<Real>> {
        self.accelerations.clone()
    }

    pub fn set_effects(&mut self, effects: Array<Gd<FluidEffect>>) {
        self.effects = effects;
    }

    pub fn get_density(&self) -> f64 {
        self.density
    }

    pub fn set_density(&mut self, density: f64) {
        self.density = density;
    }

    pub fn get_rid(&self) -> Rid {
        self.rid
    }

    pub fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
    }

    pub fn get_space(&self) -> Rid {
        self.space
    }
}
impl Drop for RapierFluid {
    fn drop(&mut self) {
        // Cleanup code here
    }
}
