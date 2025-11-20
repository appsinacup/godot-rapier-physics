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
    transform.basis.get_scale()
}
#[derive(GodotConvert, Var, Export, Debug)]
#[godot(via = GString)]
// An anum to allow easy export into various formats; None outputs a json string (plaintext, good for debugging),
// GodotBase64 uses Godot's Marshalls to produce an encoded Godot String (for if you need to decode the data in gdscript or elsewhere on the Godot side),
// and RustBincode uses, well, Rust's bincode for maximum speed and efficiency.
pub enum SerializationFormat {
    None, // first enumerator is default.
    GodotBase64,
    RustBincode,
}
pub fn bin_to_packed_byte_array(bin: Vec<u8>) -> PackedByteArray {
    let mut pba = PackedByteArray::new();
    pba.resize(bin.len());
    for i in 0..(bin.len()) {
        pba[i] = bin[i];
    }
    pba
}
#[cfg(feature = "dim2")]
pub fn transform_scale(transform: &Transform) -> Vector {
    transform.scale()
}
#[cfg(feature = "dim2")]
pub fn transform_inverse(transform: &Transform) -> Transform {
    let determnant = transform.determinant();
    if determnant == 0.0 {
        *transform
    } else {
        transform.affine_inverse()
    }
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
/// Converts a world position to local position without applying scale.
/// This is needed for joint anchors to work correctly with scaled bodies.
#[cfg(feature = "dim2")]
pub fn world_to_local_no_scale(transform: &Transform, world_pos: Vector) -> Vector {
    // Create a transform without scale for proper anchor conversion
    let scale = transform.scale();
    if scale.x.is_zero_approx() || scale.y.is_zero_approx() {
        // Degenerate scale, return as-is
        return transform.affine_inverse() * world_pos;
    }
    // Remove scale from the transform
    let rotation = transform.rotation();
    let origin = transform.origin;
    let transform_no_scale = Transform::from_angle_scale_skew_origin(
        rotation,
        Vector::new(1.0, 1.0),
        transform.skew(),
        origin,
    );
    transform_no_scale.affine_inverse() * world_pos
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
        Basis::from_quaternion(Quaternion::new(
            quaternion.coords.x,
            quaternion.coords.y,
            quaternion.coords.z,
            quaternion.coords.w,
        )),
        origin,
    );
    let scale = transform.basis.get_scale();
    new_transform.scaled_local(scale)
}
#[cfg(feature = "dim3")]
pub fn transform_rotation_rapier(transform: &godot::builtin::Transform3D) -> Rotation<Real> {
    use rapier::na::Vector4;
    let quaternion = transform.basis.get_quaternion();
    Rotation::from_quaternion(rapier::na::Quaternion {
        coords: Vector4::new(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    })
}
#[cfg(feature = "dim2")]
pub fn transform_rotation_rapier(transform: &godot::builtin::Transform2D) -> Rotation<Real> {
    let angle = transform.rotation();
    Rotation::from_angle(angle)
}
#[cfg(feature = "dim3")]
pub fn basis_to_rapier(basis: godot::builtin::Basis) -> Rotation<Real> {
    use rapier::na::Vector4;
    let quaternion = basis.get_quaternion();
    Rotation::from_quaternion(rapier::na::Quaternion {
        coords: Vector4::new(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    })
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
pub fn variant_to_int(variant: &Variant) -> i32 {
    match variant.get_type() {
        VariantType::INT => variant.to::<i32>(),
        _ => 0,
    }
}
