use godot::classes::Time;
use godot::prelude::*;

use super::types::Fluid;
use crate::servers::RapierPhysicsServer;
use crate::types::PackedVectorArray;
pub struct FluidImpl {}
impl FluidImpl {
    pub fn set_points(fluid: &mut Fluid, points: PackedVectorArray) {
        fluid.points = points;
        let old_times = fluid.create_times.len();
        fluid.create_times.resize(fluid.points.len());
        let ticks = Time::singleton().get_ticks_msec();
        for i in old_times..fluid.points.len() {
            fluid.create_times[i] = ticks as f32;
        }
        let gl_transform = fluid.to_gd().get_global_transform();
        let mut rapier_points = fluid.points.clone();
        for i in 0..fluid.points.len() {
            rapier_points[i] = gl_transform * (fluid.points[i]);
        }
        let rid = fluid.rid;
        let guard = fluid.base_mut();
        RapierPhysicsServer::fluid_set_points(rid, rapier_points.clone());
        drop(guard);
    }

    pub fn set_density(fluid: &mut Fluid, density: real) {
        if fluid.density != density {
            fluid.density = density;
            let rid = fluid.rid;
            let density = fluid.density;
            let guard = fluid.base_mut();
            RapierPhysicsServer::fluid_set_density(rid, density);
            drop(guard);
        }
    }

    pub fn set_lifetime(fluid: &mut Fluid, lifetime: real) {
        if fluid.lifetime != lifetime {
            fluid.lifetime = lifetime;
            fluid
                .to_gd()
                .set_process(fluid.debug_draw || fluid.lifetime > 0.0);
        }
    }

    pub fn set_debug_draw(fluid: &mut Fluid, debug_draw: bool) {
        if fluid.debug_draw != debug_draw {
            fluid.debug_draw = debug_draw;
        }
        let mut fluid_gd = fluid.to_gd();
        fluid_gd.set_notify_transform(fluid.debug_draw);
        fluid_gd.set_process(fluid.debug_draw || fluid.lifetime > 0.0);
    }

    pub fn get_accelerations(fluid: &Fluid) -> PackedVectorArray {
        let rid = fluid.rid;
        let guard = fluid.base();
        let accelerations = RapierPhysicsServer::fluid_get_accelerations(rid);
        drop(guard);
        accelerations
    }

    pub fn get_remaining_times(fluid: &Fluid) -> PackedFloat32Array {
        let ticks = Time::singleton().get_ticks_msec() as f32;
        let mut remaining_times = PackedFloat32Array::default();
        remaining_times.resize(fluid.create_times.len());
        let div_1000 = 1.0 / 1000.0;
        for i in 0..fluid.create_times.len() {
            remaining_times[i] = (ticks - fluid.create_times[i]) * div_1000;
        }
        remaining_times
    }

    pub fn get_velocities(fluid: &Fluid) -> PackedVectorArray {
        let rid = fluid.rid;
        let guard = fluid.base();
        let velocities = RapierPhysicsServer::fluid_get_velocities(rid);
        drop(guard);
        velocities
    }

    pub fn get_points(fluid: &Fluid) -> PackedVectorArray {
        let rid = fluid.rid;
        let guard = fluid.base();
        let mut new_points = RapierPhysicsServer::fluid_get_points(rid);
        drop(guard);
        let gl_transform = fluid.to_gd().get_global_transform().affine_inverse();
        for i in 0..new_points.len() {
            new_points[i] = gl_transform * new_points[i];
        }
        new_points
    }

    pub fn add_points_and_velocities(
        fluid: &mut Fluid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        if points.len() != velocities.len() {
            godot_error!("points and velocities must be the same length");
            return;
        }
        fluid.points.extend_array(&points);
        let old_times = fluid.create_times.len();
        let ticks = Time::singleton().get_ticks_msec();
        for _i in old_times..fluid.points.len() {
            fluid.create_times.push(ticks as f32);
        }
        let gl_transform = fluid.to_gd().get_global_transform();
        let mut rapier_points = points.clone();
        for i in 0..points.len() {
            rapier_points[i] = gl_transform * (points[i]);
        }
        let rid = fluid.rid;
        let guard = fluid.base_mut();
        RapierPhysicsServer::fluid_add_points_and_velocities(rid, rapier_points, velocities);
        drop(guard);
    }

    pub fn set_points_and_velocities(
        fluid: &mut Fluid,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        if points.len() != velocities.len() {
            godot_error!("points and velocities must be the same length");
            return;
        }
        fluid.points = points;
        let old_times = fluid.create_times.len();
        fluid.create_times.resize(fluid.points.len());
        let ticks = Time::singleton().get_ticks_msec();
        for _i in old_times..fluid.points.len() {
            fluid.create_times.push(ticks as f32);
        }
        let gl_transform = fluid.to_gd().get_global_transform();
        for i in 0..fluid.points.len() {
            fluid.points[i] = gl_transform * (fluid.points[i]);
        }
        let rid = fluid.rid;
        let points = fluid.points.clone();
        let guard = fluid.base_mut();
        RapierPhysicsServer::fluid_set_points_and_velocities(rid, points, velocities);
        drop(guard);
    }

    pub fn delete_points(fluid: &mut Fluid, indices: PackedInt32Array) {
        let rid = fluid.rid;
        let guard = fluid.base_mut();
        RapierPhysicsServer::fluid_delete_points(rid, indices.clone());
        drop(guard);
        let mut indices = indices.to_vec();
        indices.sort_unstable();
        indices.reverse();
        for index in indices {
            fluid.points.remove(index as usize);
            fluid.create_times.remove(index as usize);
        }
    }

    pub fn set_effects(fluid: &mut Fluid, effects: Array<Option<Gd<Resource>>>) {
        fluid.effects = effects;
        let rid = fluid.rid;
        let effects = fluid.effects.clone();
        let guard = fluid.base_mut();
        RapierPhysicsServer::fluid_set_effects(rid, effects);
        drop(guard);
    }

    pub fn delete_old_particles(fluid: &mut Fluid) {
        if fluid.lifetime <= 0.0 {
            return;
        }
        let ticks = Time::singleton().get_ticks_msec() as f32;
        let mut to_remove = PackedInt32Array::default();
        for i in 0..fluid.create_times.len() {
            if ticks - fluid.create_times[i] > fluid.lifetime * 1000.0 {
                to_remove.push(i as i32);
            }
        }
        if !to_remove.is_empty() {
            FluidImpl::delete_points(fluid, to_remove);
        }
    }
}
