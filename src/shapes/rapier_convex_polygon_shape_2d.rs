use crate::rapier2d::handle::{invalid_handle, Handle};
use crate::rapier2d::shape::shape_create_convex_polyline;
use crate::rapier2d::vector::Vector;
use crate::shapes::rapier_shape_2d::{IRapierShape2D, RapierShapeBase2D};
use godot::engine::physics_server_2d::ShapeType;
use godot::prelude::*;

pub struct RapierConvexPolygonShape2D {
    points: Vec<Point>,
    pub base: RapierShapeBase2D,
}

#[derive(Clone, Copy)]
struct Point {
    pos: Vector2,
    normal: Vector2,
}

impl RapierConvexPolygonShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            points: Vec::new(),
            base: RapierShapeBase2D::new(rid),
        }
    }
}

impl IRapierShape2D for RapierConvexPolygonShape2D {
    fn get_base(&self) -> &RapierShapeBase2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierShapeBase2D {
        &mut self.base
    }
    fn get_type(&self) -> ShapeType {
        ShapeType::CONVEX_POLYGON
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector2) -> f32 {
        if self.points.len() < 3 {
            return 0.0;
        }
        let mut aabb_new = Rect2::new(Vector2::ZERO, Vector2::ZERO);
        for point in self.points.iter() {
            aabb_new = aabb_new.expand(point.pos * scale);
        }
        mass * aabb_new.size.dot(aabb_new.size) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        if self.points.len() >= 3 {
            let mut rapier_points = Vec::with_capacity(self.points.len());
            for point in self.points.iter() {
                rapier_points.push(Vector::new(point.pos.x, point.pos.y));
            }
            shape_create_convex_polyline(rapier_points)
        } else {
            godot_error!("ConvexPolygon2D must have at least three point");
            invalid_handle()
        }
    }

    fn set_data(&mut self, data: Variant) {
        match data.get_type() {
            VariantType::PackedVector2Array => {
                let arr: PackedVector2Array = data.to();
                let size = arr.len();
                assert!(size > 0);
                self.points = Vec::with_capacity(size);

                for i in 0..size {
                    self.points[i] = Point {
                        pos: arr.get(i),
                        normal: Vector2::ZERO,
                    };
                }

                for i in 0..size {
                    let p = self.points[i].pos;
                    let pn = self.points[(i + 1) % size].pos;
                    self.points[i].normal = (pn - p).orthogonal().normalized();
                }
            }
            VariantType::PackedFloat32Array | VariantType::PackedFloat64Array => {
                let arr: PackedFloat32Array = data.to();

                let size = arr.len() / 4;
                assert!(size > 0);
                self.points = Vec::with_capacity(size);

                for i in 0..size {
                    let idx = i << 2;
                    self.points[i] = Point {
                        pos: Vector2::new(arr.get(idx), arr.get(idx + 1)),
                        normal: Vector2::new(arr.get(idx + 2), arr.get(idx + 3)),
                    };
                }
            }
            _ => godot_error!("Invalid data type for RapierConvexPolygonShape2D"),
        }
        if self.points.len() < 3 {
            godot_error!("ConvexPolygon2D must have at least three point");
            return;
        }
        let mut aabb = Rect2::new(Vector2::ZERO, Vector2::ZERO);
        for point in self.points.iter() {
            aabb = aabb.expand(point.pos);
        }

        self.base.configure(aabb);
    }

    fn get_data(&self) -> Variant {
        let mut arr = PackedVector2Array::new();
        for point in self.points.iter() {
            arr.push(point.pos);
        }
        arr.to_variant()
    }

    fn get_rapier_shape(&mut self) -> Handle {
        if !self.base.get_handle().is_valid() {
            let handle = self.create_rapier_shape();
            self.base.set_handle(handle);
        }
        self.base.get_handle()
    }
}
