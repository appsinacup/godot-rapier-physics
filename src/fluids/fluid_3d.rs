use godot::classes::notify::Node3DNotification;
use godot::classes::Engine;
use godot::prelude::*;

use super::fluid_impl::FluidImpl;
use crate::servers::rapier_project_settings::RapierProjectSettings;
use crate::servers::RapierPhysicsServer;
use crate::types::*;
#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct Fluid3D {
    #[var(get)]
    pub(crate) rid: Rid,
    #[var(get)]
    pub(crate) radius: real,
    #[export]
    #[var(get, set = set_debug_draw)]
    pub(crate) debug_draw: bool,
    #[export]
    #[var(get, set = set_density)]
    pub(crate) density: real,
    #[export]
    #[var(get, set = set_lifetime)]
    pub(crate) lifetime: real,
    #[export]
    #[var(get, set = set_effects)]
    pub(crate) effects: Array<Option<Gd<Resource>>>,

    #[export]
    #[var(get = get_points, set = set_points)]
    pub(crate) points: PackedVectorArray,
    pub(crate) create_times: PackedFloat32Array,
    base: Base<Node3D>,
}
#[godot_api]
impl Fluid3D {
    #[func]
    fn set_points(&mut self, points: PackedVectorArray) {
        FluidImpl::set_points(self, points);
    }

    #[func]
    fn set_density(&mut self, density: real) {
        FluidImpl::set_density(self, density);
    }

    #[func]
    fn set_lifetime(&mut self, lifetime: real) {
        FluidImpl::set_lifetime(self, lifetime);
    }

    #[func]
    fn set_debug_draw(&mut self, debug_draw: bool) {
        FluidImpl::set_debug_draw(self, debug_draw);
    }

    #[func]
    fn get_accelerations(&self) -> PackedVectorArray {
        FluidImpl::get_accelerations(self)
    }

    #[func]
    fn get_remaining_times(&self) -> PackedFloat32Array {
        FluidImpl::get_remaining_times(self)
    }

    #[func]
    fn get_velocities(&self) -> PackedVectorArray {
        FluidImpl::get_velocities(self)
    }

    #[func]
    fn get_points(&self) -> PackedVectorArray {
        FluidImpl::get_points(self)
    }

    #[func]
    fn create_box_points(&self, width: i32, height: i32, depth: i32) -> PackedVectorArray {
        let mut new_points = PackedVectorArray::default();
        new_points.resize((width * height * depth) as usize);
        for i in 0..width {
            for j in 0..height {
                for k in 0..depth {
                    new_points[(i + j * width + k * width * height) as usize] = Vector::new(
                        i as f32 * self.radius * 2.0,
                        j as f32 * self.radius * 2.0,
                        k as f32 * self.radius * 2.0,
                    );
                }
            }
        }
        new_points
    }

    #[func]
    fn create_sphere_points(&self, radius: i32) -> PackedVectorArray {
        let mut new_points = PackedVectorArray::default();
        for i in -radius..radius {
            for j in -radius..radius {
                for k in -radius..radius {
                    let x = i as f32 * self.radius * 2.0;
                    let y = j as f32 * self.radius * 2.0;
                    let z = k as f32 * self.radius * 2.0;
                    if i * i + j * j * k * k <= radius * radius {
                        new_points.push(Vector::new(x, y, z));
                    }
                }
            }
        }
        new_points
    }

    #[func]
    fn add_points_and_velocities(
        &mut self,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        FluidImpl::add_points_and_velocities(self, points, velocities);
    }

    #[func]
    fn set_points_and_velocities(
        &mut self,
        points: PackedVectorArray,
        velocities: PackedVectorArray,
    ) {
        FluidImpl::set_points_and_velocities(self, points, velocities);
    }

    #[func]
    fn delete_points(&mut self, indices: PackedInt32Array) {
        FluidImpl::delete_points(self, indices);
    }

    #[func]
    fn set_effects(&mut self, effects: Array<Option<Gd<Resource>>>) {
        FluidImpl::set_effects(self, effects);
    }
}
#[godot_api]
impl INode3D for Fluid3D {
    fn init(base: Base<Node3D>) -> Self {
        Self {
            rid: RapierPhysicsServer::fluid_create(),
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

    fn on_notification(&mut self, p_what: Node3DNotification) {
        match p_what {
            Node3DNotification::PROCESS => {
                if self.debug_draw {
                    // TODO
                    //self.to_gd().queue_redraw();
                }
                if !Engine::singleton().is_editor_hint() {
                    FluidImpl::delete_old_particles(self);
                }
            }
            Node3DNotification::ENTER_TREE | Node3DNotification::ENTER_WORLD => {
                let mut space_rid = Rid::Invalid;
                let fluid_gd = self.to_gd();
                if !fluid_gd.is_inside_tree() {
                    return;
                }
                if let Some(space) = fluid_gd.get_world_3d() {
                    space_rid = space.get_space();
                }
                drop(fluid_gd);
                let rid = self.rid;
                let guard = self.base_mut();
                RapierPhysicsServer::fluid_set_space(rid, space_rid);
                drop(guard);
                self.set_points(self.points.clone());
                let mut fluid_gd = self.to_gd();
                fluid_gd.set_notify_transform(self.debug_draw);
            }
            Node3DNotification::TRANSFORM_CHANGED
            | Node3DNotification::LOCAL_TRANSFORM_CHANGED
            | Node3DNotification::TRANSLATION_CHANGED => {
                if !self.to_gd().is_inside_tree() {
                    return;
                }
                self.set_points(self.points.clone());
                let mut fluid_gd = self.to_gd();
                fluid_gd.set_notify_transform(self.debug_draw);
                // TODO
                //fluid_gd.queue_redraw();
            }
            Node3DNotification::EXIT_TREE | Node3DNotification::EXIT_WORLD => {
                let rid = self.rid;
                let guard = self.base_mut();
                RapierPhysicsServer::fluid_set_space(rid, Rid::Invalid);
                drop(guard);
            }
            /*
            Node3DNotification::DRAW => {
                if self.debug_draw {
                    self.points = self.get_points();
                    for point in self.points.as_slice() {
                        let mut color = Color::WHITE;
                        color.a = 0.4;
                        self.to_gd().draw_rect(
                            Rect2::new(
                                *point - Vector2::new(self.radius / 2.0, self.radius / 2.0),
                                Vector2::new(self.radius, self.radius),
                            ),
                            color,
                        );
                    }
                }
            } */
            _ => {}
        }
    }
}
impl Drop for Fluid3D {
    fn drop(&mut self) {
        if self.rid != Rid::Invalid {
            PhysicsServer::singleton().free_rid(self.rid);
        }
    }
}
