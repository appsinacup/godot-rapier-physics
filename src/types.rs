use godot::classes::*;
use godot::prelude::*;
use rapier::math::Real;
use rapier::math::Rotation;
#[cfg(feature = "single")]
pub type PackedFloatArray = PackedFloat32Array;
#[cfg(feature = "double")]
pub type PackedFloatArray = PackedFloat64Array;
#[cfg(feature = "dim3")]
pub type PackedVectorArray = godot::prelude::PackedVector3Array;
#[cfg(feature = "dim2")]
pub type PackedVectorArray = godot::prelude::PackedVector2Array;
#[cfg(feature = "dim3")]
pub type Transform = godot::prelude::Transform3D;
#[cfg(feature = "dim2")]
pub type Transform = godot::prelude::Transform2D;
#[cfg(feature = "dim3")]
pub type Vector = godot::prelude::Vector3;
#[cfg(feature = "dim2")]
pub type Vector = godot::prelude::Vector2;
#[cfg(feature = "dim3")]
pub type Angle = godot::prelude::Vector3;
#[cfg(feature = "dim2")]
pub type Angle = real;
#[cfg(feature = "dim3")]
pub type Rect = godot::prelude::Aabb;
#[cfg(feature = "dim2")]
pub type Rect = godot::prelude::Rect2;
#[cfg(feature = "dim2")]
pub const ANGLE_ZERO: rapier::math::Real = 0.0;
#[cfg(feature = "dim3")]
pub const ANGLE_ZERO: Vector3 = Vector3::new(0.0, 0.0, 0.0);
#[cfg(feature = "dim2")]
pub type PhysicsDirectSpaceState = PhysicsDirectSpaceState2D;
#[cfg(feature = "dim3")]
pub type PhysicsDirectSpaceState = PhysicsDirectSpaceState3D;
#[cfg(feature = "dim2")]
pub type PhysicsServerExtensionMotionResult = native::PhysicsServer2DExtensionMotionResult;
#[cfg(feature = "dim3")]
pub type PhysicsServerExtensionMotionResult = native::PhysicsServer3DExtensionMotionResult;
#[cfg(feature = "dim2")]
pub type RapierDirectBodyState =
    crate::bodies::rapier_direct_body_state_2d::RapierDirectBodyState2D;
#[cfg(feature = "dim2")]
pub type PhysicsDirectBodyState = PhysicsDirectBodyState2D;
#[cfg(feature = "dim3")]
pub type RapierDirectBodyState =
    crate::bodies::rapier_direct_body_state_3d::RapierDirectBodyState3D;
#[cfg(feature = "dim3")]
pub type PhysicsDirectBodyState = PhysicsDirectBodyState3D;
#[cfg(feature = "dim2")]
pub type PhysicsServer = PhysicsServer2D;
#[cfg(feature = "dim3")]
pub type PhysicsServer = PhysicsServer3D;
#[cfg(feature = "dim2")]
pub type PhysicsServerExtensionShapeResult = native::PhysicsServer2DExtensionShapeResult;
#[cfg(feature = "dim2")]
pub type PhysicsServerExtensionRayResult = native::PhysicsServer2DExtensionRayResult;
#[cfg(feature = "dim2")]
pub type PhysicsServerExtensionShapeRestInfo = native::PhysicsServer2DExtensionShapeRestInfo;
#[cfg(feature = "dim3")]
pub type PhysicsServerExtensionShapeResult = native::PhysicsServer3DExtensionShapeResult;
#[cfg(feature = "dim3")]
pub type PhysicsServerExtensionRayResult = native::PhysicsServer3DExtensionRayResult;
#[cfg(feature = "dim3")]
pub type PhysicsServerExtensionShapeRestInfo = native::PhysicsServer3DExtensionShapeRestInfo;
#[cfg(feature = "dim3")]
pub fn transform_scale(transform: &Transform) -> Vector {
    transform.basis.scale()
}
#[cfg(feature = "dim2")]
pub fn transform_scale(transform: &Transform) -> Vector {
    transform.scale()
}
#[cfg(feature = "dim3")]
pub fn transform_inverse(transform: &Transform) -> Transform {
    let determnant = transform.basis.determinant();
    if determnant == 0.0 {
        *transform
    } else {
        transform.affine_inverse()
    }
}
#[cfg(feature = "dim2")]
pub fn transform_inverse(transform: &Transform) -> Transform {
    // todo use determinant
    if transform.a == Vector::ZERO && transform.b == Vector::ZERO {
        *transform
    } else {
        transform.affine_inverse()
    }
}
#[cfg(feature = "dim2")]
pub fn transform_update(
    transform: &Transform,
    rotation: Rotation<Real>,
    origin: Vector,
) -> Transform {
    let mut skew = 0.0;
    if !transform.determinant().is_zero_approx() {
        skew = transform.skew();
    }
    Transform::from_angle_scale_skew_origin(rotation.angle(), transform.scale(), skew, origin)
}
#[cfg(feature = "dim3")]
pub fn transform_update(
    transform: &Transform,
    rotation: Rotation<Real>,
    origin: Vector,
) -> Transform {
    use godot::builtin::Basis;
    let quaternion = rotation.quaternion();
    let new_transform = Transform::new(
        Basis::from_quat(Quaternion::new(
            quaternion.coords.x,
            quaternion.coords.y,
            quaternion.coords.z,
            quaternion.coords.w,
        )),
        origin,
    );
    let scale = transform.basis.scale();
    new_transform.scaled_local(scale)
}
#[cfg(feature = "dim3")]
pub fn transform_rotation_rapier(transform: &godot::builtin::Transform3D) -> Rotation<Real> {
    use rapier::na::Vector4;
    let quaternion = transform.basis.to_quat();
    Rotation::from_quaternion(rapier::na::Quaternion {
        coords: Vector4::new(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    })
}
#[cfg(feature = "dim2")]
pub fn transform_rotation_rapier(transform: &godot::builtin::Transform2D) -> Rotation<Real> {
    let angle = transform.rotation();
    Rotation::from_angle(angle)
}
pub fn vector_normalized(vector: Vector) -> Vector {
    if vector != Vector::ZERO {
        return vector.normalized();
    }
    Vector::ZERO
}
pub fn variant_to_float(variant: &Variant) -> real {
    match variant.get_type() {
        VariantType::FLOAT => variant.to::<real>(),
        VariantType::INT => variant.to::<i32>() as real,
        _ => 0.0,
    }
}
pub fn invalid_rid() -> Rid {
    Rid::Invalid
}
