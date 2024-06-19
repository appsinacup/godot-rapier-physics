use godot::engine::physics_server_2d::ShapeType;
use godot::prelude::*;

use crate::rapier_wrapper::prelude::*;
use crate::shapes::rapier_shape::*;
use crate::Vector;
#[derive(Serialize, Deserialize, Debug)]
pub struct RapierConcavePolygonShape2D {
    points: Vec<Vector2>,
    segments: Vec<[i32; 2]>,
    base: RapierShapeBase,
}
impl RapierConcavePolygonShape2D {
    pub fn new(rid: Rid) -> Self {
        Self {
            points: Vec::new(),
            segments: Vec::new(),
            base: RapierShapeBase::new(rid),
        }
    }
}
fn find_or_insert_point(points: &mut Vec<Vector2>, new_point: Vector2) -> usize {
    for (idx, &point) in points.iter().enumerate() {
        if point.distance_squared_to(new_point) < 1e-4 {
            return idx;
        }
    }
    // If the point is not found, add it to the vector and return its index
    let idx = points.len();
    points.push(new_point);
    idx
}
impl IRapierShape for RapierConcavePolygonShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CONCAVE_POLYGON
    }

    fn get_moment_of_inertia(&self, mass: f32, scale: Vector) -> f32 {
        if self.points.len() < 3 {
            return 0.0;
        }
        let mut aabb_new = Rect2::new(Vector2::ZERO, Vector2::ZERO);
        for point in self.points.iter() {
            aabb_new = aabb_new.expand(*point * scale);
        }
        mass * aabb_new.size.dot(aabb_new.size) / 12.0
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn create_rapier_shape(&mut self) -> Handle {
        if self.points.len() >= 3 {
            let point_count = self.points.len();
            let mut rapier_points = Vec::with_capacity(point_count + 1);
            for i in 0..point_count {
                rapier_points.push(vector_to_rapier(self.points[i]));
            }
            // Close the polyline shape
            rapier_points.push(rapier_points[0]);
            shape_create_concave_polyline(rapier_points)
        } else {
            godot_error!("ConcavePolygon2D must have at least three point");
            invalid_handle()
        }
    }

    fn set_data(&mut self, data: Variant) {
        let mut aabb = Rect2::default();
        match data.get_type() {
            VariantType::PACKED_VECTOR2_ARRAY => {
                let arr: PackedVector2Array = data.to();
                let len = arr.len();
                if len == 0 {
                    return;
                }
                if len % 2 != 0 {
                    godot_error!("ConcavePolygon2D must have an even number of points");
                    return;
                }
                self.segments.clear();
                self.points.clear();
                for i in (0..len).step_by(2) {
                    let p1 = arr[i];
                    let p2 = arr[i + 1];
                    // Find or insert the points into the `points` vector
                    let idx_p1 = find_or_insert_point(&mut self.points, p1);
                    let idx_p2 = find_or_insert_point(&mut self.points, p2);
                    // Create the segment with the indices of the points
                    let s = [idx_p1 as i32, idx_p2 as i32];
                    self.segments.push(s);
                }
                for &p in self.points.iter() {
                    aabb = aabb.expand(p);
                }
            }
            _ => {
                // Handle dictionary with arrays
                godot_error!("Invalid shape data");
                return;
            }
        }
        let handle = self.create_rapier_shape();
        self.base.set_handle(handle, aabb);
    }

    fn get_data(&self) -> Variant {
        let len = self.segments.len();
        if len == 0 {
            return Variant::nil();
        }
        let mut rsegments = PackedVector2Array::new();
        rsegments.resize(len * 2);
        for i in 0..len {
            let idx0 = self.segments[i][0] as usize;
            let idx1 = self.segments[i][1] as usize;
            rsegments[i << 1] = self.points[idx0];
            rsegments[(i << 1) + 1] = self.points[idx1];
        }
        rsegments.to_variant()
    }

    fn get_handle(&mut self) -> Handle {
        self.base.get_handle()
    }
}
