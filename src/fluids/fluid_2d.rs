use godot::engine::notify::CanvasItemNotification;
use godot::engine::Engine;
use godot::engine::IPhysicsServer2DExtension;
use godot::engine::Time;
use godot::prelude::*;

use crate::servers::rapier_project_settings::RapierProjectSettings;
use crate::servers::RapierPhysicsServer;
use crate::types::*;
#[derive(GodotClass)]
#[class(base=Node2D)]
pub struct Fluid2D {
    rid: Rid,
    radius: real,
    #[export]
    #[var(get, set = set_debug_draw)]
    debug_draw: bool,
    #[export]
    #[var(get, set = set_density)]
    density: real,
    #[export]
    #[var(get, set = set_lifetime)]
    lifetime: real,
    #[export]
    #[var(get, set = set_effects)]
    effects: Array<Option<Gd<Resource>>>,

    #[export]
    #[var(get = get_points, set = set_points)]
    points: PackedVectorArray,
    create_times: PackedFloat32Array,
    base: Base<Node2D>,
}
#[godot_api]
impl Fluid2D {
    #[func]
    fn set_points(&mut self, p_points: PackedVector2Array) {
        self.points = p_points;
        let old_times = self.create_times.len();
        self.create_times.resize(self.points.len());
        let ticks = Time::singleton().get_ticks_msec();
        for i in old_times..self.points.len() {
            self.create_times[i] = ticks as f32;
        }
        let gl_transform = self.to_gd().get_global_transform();
        for i in 0..self.points.len() {
            self.points[i] = gl_transform * (self.points[i]);
        }
        RapierPhysicsServer::fluid_set_points(self.rid, self.points.clone());
        self.to_gd().queue_redraw();
    }

    #[func]
    fn set_density(&mut self, p_density: real) {
        if self.density != p_density {
            self.density = p_density;
            RapierPhysicsServer::fluid_set_density(self.rid, self.density);
        }
    }

    #[func]
    fn set_lifetime(&mut self, p_lifetime: real) {
        if self.lifetime != p_lifetime {
            self.lifetime = p_lifetime;
            self.to_gd()
                .set_process(self.debug_draw || self.lifetime > 0.0);
        }
    }

    #[func]
    fn set_debug_draw(&mut self, p_debug_draw: bool) {
        if self.debug_draw != p_debug_draw {
            self.debug_draw = p_debug_draw;
        }
        let mut fluid_gd = self.to_gd();
        fluid_gd.set_process(self.debug_draw || self.lifetime > 0.0);
        fluid_gd.set_notify_transform(self.debug_draw);
        fluid_gd.queue_redraw();
    }

    #[func]
    fn get_accelerations(&self) -> PackedVector2Array {
        RapierPhysicsServer::fluid_get_accelerations(self.rid)
    }

    #[func]
    fn get_velocities(&self) -> PackedVector2Array {
        RapierPhysicsServer::fluid_get_velocities(self.rid)
    }

    #[func]
    fn get_points(&self) -> PackedVector2Array {
        let mut new_points = RapierPhysicsServer::fluid_get_points(self.rid);
        let gl_transform = self.to_gd().get_global_transform().affine_inverse();
        for i in 0..new_points.len() {
            new_points[i] = gl_transform * new_points[i];
        }
        new_points
    }

    #[func]
    fn create_rectangle_points(&self, width: i32, height: i32) -> PackedVector2Array {
        let mut new_points = PackedVector2Array::default();
        new_points.resize((width * height) as usize);
        for i in 0..width {
            for j in 0..height {
                new_points[(i + j * width) as usize] = Vector2::new(
                    i as f32 * self.radius * 2.0,
                    j as f32 * self.radius * 2.0,
                );
            }
        }
        new_points
    }

    #[func]
    fn create_circle_points(&self, p_radius: i32) -> PackedVector2Array {
        let mut new_points = PackedVector2Array::default();
        for i in -p_radius..p_radius {
            for j in -p_radius..p_radius {
                let x = i as f32 * self.radius * 2.0;
                let y = j as f32 * self.radius * 2.0;
                if i * i + j * j <= p_radius * p_radius {
                    new_points.push(Vector2::new(x, y));
                }
            }
        }
        new_points
    }

    #[func]
    fn add_points_and_velocities(
        &mut self,
        p_points: PackedVector2Array,
        p_velocities: PackedVector2Array,
    ) {
        let mut p_points = p_points;
        self.points.extend_array(&p_points);
        let old_times = self.create_times.len();
        let ticks = Time::singleton().get_ticks_msec();
        for i in old_times..self.points.len() {
            self.create_times.push(ticks as f32);
        }
        let gl_transform = self.to_gd().get_global_transform();
        for i in 0..self.points.len() {
            p_points[i] = gl_transform * (p_points[i]);
        }
        RapierPhysicsServer::fluid_add_points_and_velocities(self.rid, p_points, p_velocities);
        self.to_gd().queue_redraw();
    }

    #[func]
    fn set_points_and_velocities(
        &mut self,
        p_points: PackedVector2Array,
        p_velocities: PackedVector2Array,
    ) {
        self.points = p_points;
        let old_times = self.create_times.len();
        self.create_times.resize(self.points.len());
        let ticks = Time::singleton().get_ticks_msec();
        for i in old_times..self.points.len() {
            self.create_times.push(ticks as f32);
        }
        let gl_transform = self.to_gd().get_global_transform();
        for i in 0..self.points.len() {
            self.points[i] = gl_transform * (self.points[i]);
        }
        RapierPhysicsServer::fluid_add_points_and_velocities(
            self.rid,
            self.points.clone(),
            p_velocities,
        );
        self.to_gd().queue_redraw();
    }

    #[func]
    fn delete_points(&mut self, p_indices: PackedInt32Array) {
        RapierPhysicsServer::fluid_delete_points(self.rid, p_indices.clone());
        for i in 0..p_indices.len() {
            self.points.remove(p_indices[i] as usize);
            self.create_times.remove(p_indices[i] as usize);
        }
        self.to_gd().queue_redraw();
    }

    #[func]
    fn set_effects(&mut self, p_effects: Array<Option<Gd<Resource>>>) {
        self.effects = p_effects;
        RapierPhysicsServer::fluid_set_effects(self.rid, self.effects.clone());
    }

    fn delete_old_particles(&mut self) {
        if self.lifetime <= 0.0 {
            return;
        }
        let ticks = Time::singleton().get_ticks_msec() as f32;
        let mut to_remove = PackedInt32Array::default();
        for i in 0..self.create_times.len() {
            if ticks - self.create_times[i] > self.lifetime * 1000.0 {
                to_remove.push(i as i32);
            }
        }
        if !to_remove.is_empty() {
            to_remove.reverse();
            self.delete_points(to_remove);
        }
    }
}
#[godot_api]
impl INode2D for Fluid2D {
    fn init(base: Base<Node2D>) -> Self {
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

    fn on_notification(&mut self, p_what: CanvasItemNotification) {
        match p_what {
            CanvasItemNotification::PROCESS => {
                if self.debug_draw {
                    self.to_gd().queue_redraw();
                }
                if !Engine::singleton().is_editor_hint() {
                    self.delete_old_particles();
                }
            }
            CanvasItemNotification::ENTER_TREE
            | CanvasItemNotification::WORLD_2D_CHANGED
            | CanvasItemNotification::TRANSFORM_CHANGED
            | CanvasItemNotification::LOCAL_TRANSFORM_CHANGED
            | CanvasItemNotification::TRANSLATION_CHANGED => {
                let mut space_rid = Rid::Invalid;
                if let Some(space) = self.to_gd().get_world_2d() {
                    space_rid = space.get_rid();
                }
                RapierPhysicsServer::fluid_set_space(space_rid, space_rid);
                self.set_points(self.points.clone());
                if self.debug_draw {
                    self.to_gd().queue_redraw();
                }
            }
            CanvasItemNotification::EXIT_TREE => {
                RapierPhysicsServer::fluid_set_space(self.rid, Rid::Invalid);
            }
            CanvasItemNotification::DRAW => {
                if self.debug_draw {
                    self.points = self.get_points();
                    for point in self.points.as_slice() {
                        self.to_gd().draw_rect(
                            Rect2::new(
                                *point - Vector2::new(self.radius / 2.0, self.radius / 2.0),
                                Vector2::new(self.radius, self.radius),
                            ),
                            Color::WHITE,
                        );
                    }
                }
            }
            _ => {}
        }
    }
}
impl Drop for Fluid2D {
    fn drop(&mut self) {
        if self.rid != Rid::Invalid {
            PhysicsServer::singleton().free_rid(self.rid);
        }
    }
}
