use godot::classes::physics_server_2d::ShapeType;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::PhysicsRids;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;
pub struct RapierSegmentShape2D {
    base: RapierShapeBase,
}
impl RapierSegmentShape2D {
    pub fn create(rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            base: RapierShapeBase::new(rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierSegmentShape2D(shape));
    }
}
impl IRapierShape for RapierSegmentShape2D {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::SEGMENT
    }

    fn allows_one_way_collision(&self) -> bool {
        true
    }

    fn set_data(
        &mut self,
        data: Variant,
        physics_engine: &mut PhysicsEngine,
        physics_rids: &mut PhysicsRids,
    ) {
        if data.get_type() != VariantType::RECT2 {
            godot_error!("RapierSegmentShape data must be a Rect2. Got {}", data);
            return;
        }
        let r: Rect2 = data.try_to().unwrap_or_default();
        let p1 = r.position;
        let p2 = r.size;
        let rapier_points = [vector_to_rapier(p1), vector_to_rapier(p2)];
        let handle = physics_engine.shape_create_concave_polyline(&rapier_points.to_vec(), None);
        self.base
            .set_handle_and_reset_aabb(handle, physics_engine, physics_rids);
    }

    fn get_data(&self, physics_engine: &PhysicsEngine) -> Variant {
        let (points, _) = physics_engine.shape_get_concave_polyline(self.base.get_handle());
        if points.len() != 2 {
            godot_error!(
                "RapierSegmentShape data must be a Rect2. Got length {}",
                points.len()
            );
            return Rect2::default().to_variant();
        }
        let r = Rect2::new(
            vector_to_godot(points[0].coords),
            vector_to_godot(points[1].coords),
        );
        r.to_variant()
    }
}
