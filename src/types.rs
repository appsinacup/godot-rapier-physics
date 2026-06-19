use godot::classes::*;
use godot::prelude::*;
use rapier::math::Rotation;
use rapier::na::ComplexField;
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
    // Deterministic scale extraction: compute column lengths using ComplexField::sqrt
    // instead of Godot's platform-dependent get_scale().
    let basis = &transform.basis;
    let col0 = basis.col_a();
    let col1 = basis.col_b();
    let col2 = basis.col_c();
    let sx: real =
        ComplexField::sqrt(col0.x * col0.x + col0.y * col0.y + col0.z * col0.z);
    let sy: real =
        ComplexField::sqrt(col1.x * col1.x + col1.y * col1.y + col1.z * col1.z);
    let sz: real =
        ComplexField::sqrt(col2.x * col2.x + col2.y * col2.y + col2.z * col2.z);
    // Detect negative scale via determinant sign
    let det = basis.determinant();
    if det < 0.0 {
        Vector3::new(-sx, sy, sz)
    } else {
        Vector3::new(sx, sy, sz)
    }
}
#[derive(Clone, GodotConvert, Var, Export, Debug)]
#[godot(via = GString)]
// An enum to allow easy export into various formats; Json outputs a json string (plaintext, good for debugging),
// GodotBase64 uses Godot's Marshalls to produce an encoded Godot String (for if you need to decode the data in gdscript or elsewhere on the Godot side),
// and RustBincode uses, well, Rust's bincode for maximum speed and efficiency.
pub enum SerializationFormat {
    Json, // first enumerator is default.
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
    // Extract rotation deterministically from basis vectors
    let ax = transform.a.x;
    let ay = transform.a.y;
    let len: real = ComplexField::sqrt(ax * ax + ay * ay);
    let (cos_r, sin_r) = if len > 0.0 {
        (ax / len, ay / len)
    } else {
        (1.0, 0.0)
    };
    // Build a unit-scale transform from the deterministic rotation
    let origin = transform.origin;
    let transform_no_scale = Transform2D {
        a: Vector2::new(cos_r, sin_r),
        b: Vector2::new(-sin_r, cos_r),
        origin,
    };
    transform_no_scale.affine_inverse() * world_pos
}
#[cfg(feature = "dim2")]
pub fn transform_update(transform: &Transform, rotation: Rotation, origin: Vector) -> Transform {
    // Use deterministic math to avoid platform-dependent transcendental functions.
    // This is critical for cross-platform determinism with the enhanced-determinism feature.
    //
    // The rotation is a UnitComplex (cos θ, sin θ) from rapier — already deterministic.
    // We need to compute the delta rotation and apply it to the existing basis vectors.
    let cos_new = rotation.re;
    let sin_new = rotation.im;
    // Extract current rotation deterministically from the 'a' basis vector.
    let ax = transform.a.x;
    let ay = transform.a.y;
    let len_a: real = ComplexField::sqrt(ax * ax + ay * ay);
    let (cos_old, sin_old) = if len_a > 0.0 {
        (ax / len_a, ay / len_a)
    } else {
        (1.0, 0.0)
    };
    // Compute delta rotation: new * inverse(old)
    // inverse of (cos, sin) = (cos, -sin) for unit complex
    // (cos_new + i*sin_new) * (cos_old - i*sin_old)
    let cos_delta = cos_new * cos_old + sin_new * sin_old;
    let sin_delta = sin_new * cos_old - cos_new * sin_old;
    // Apply delta rotation to both basis vectors to preserve skew and scale.
    // Rotation of vector (x, y) by angle: (x*cos - y*sin, x*sin + y*cos)
    let mut a = Vector2::new(
        transform.a.x * cos_delta - transform.a.y * sin_delta,
        transform.a.x * sin_delta + transform.a.y * cos_delta,
    );
    let mut b = Vector2::new(
        transform.b.x * cos_delta - transform.b.y * sin_delta,
        transform.b.x * sin_delta + transform.b.y * cos_delta,
    );
    // Re-normalize to prevent scale drift from repeated rotations.
    // Use deterministic sqrt via ComplexField (backed by libm with enhanced-determinism).
    let new_len_a: real = ComplexField::sqrt(a.x * a.x + a.y * a.y);
    if !len_a.is_zero_approx() && !new_len_a.is_zero_approx() {
        let correction_a = len_a / new_len_a;
        a *= correction_a;
    }
    let bx = transform.b.x;
    let by = transform.b.y;
    let len_b: real = ComplexField::sqrt(bx * bx + by * by);
    let new_len_b: real = ComplexField::sqrt(b.x * b.x + b.y * b.y);
    if !len_b.is_zero_approx() && !new_len_b.is_zero_approx() {
        let correction_b = len_b / new_len_b;
        b *= correction_b;
    }
    Transform2D { a, b, origin }
}
#[cfg(feature = "dim3")]
pub fn transform_update(transform: &Transform, rotation: Rotation, origin: Vector) -> Transform {
    // Deterministic basis construction from quaternion.
    // Avoids Godot's Basis::from_quaternion which may use platform-dependent math internally.
    let x = rotation.x;
    let y = rotation.y;
    let z = rotation.z;
    let w = rotation.w;
    // Rotation matrix from quaternion (column-major):
    // col0 = (1-2(y²+z²), 2(xy+wz), 2(xz-wy))
    // col1 = (2(xy-wz), 1-2(x²+z²), 2(yz+wx))
    // col2 = (2(xz+wy), 2(yz-wx), 1-2(x²+y²))
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;
    let col0 = Vector3::new(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy));
    let col1 = Vector3::new(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx));
    let col2 = Vector3::new(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy));
    // Apply scale from the original transform deterministically
    let scale = transform_scale(transform);
    let basis = godot::builtin::Basis::from_cols(
        col0 * scale.x,
        col1 * scale.y,
        col2 * scale.z,
    );
    Transform::new(basis, origin)
}
#[cfg(feature = "dim3")]
pub fn transform_rotation_rapier(transform: &godot::builtin::Transform3D) -> Rotation {
    basis_to_rapier(transform.basis)
}
#[cfg(feature = "dim2")]
pub fn transform_rotation_rapier(transform: &godot::builtin::Transform2D) -> Rotation {
    // Instead of calling transform.rotation() which uses Godot's platform-dependent atan2,
    // extract the rotation directly from the basis vectors using deterministic math.
    // The 'a' column of Transform2D is (cos*scale_x, sin*scale_x).
    // We need the unit vector direction, which gives us (cos, sin) for the Rotation.
    let ax = transform.a.x;
    let ay = transform.a.y;
    let len: real = ComplexField::sqrt(ax * ax + ay * ay);
    if len > 0.0 {
        // Rotation (UnitComplex) stores (cos, sin) = (re, im)
        Rotation::from_cos_sin_unchecked(ax / len, ay / len)
    } else {
        Rotation::identity()
    }
}
#[cfg(feature = "dim3")]
pub fn basis_to_rapier(basis: godot::builtin::Basis) -> Rotation {
    // Deterministic quaternion extraction from a rotation matrix using Shepperd's method.
    // Uses ComplexField::sqrt (backed by libm with enhanced-determinism) instead of
    // platform-dependent sqrt.
    //
    // First, remove scale from the basis to get a pure rotation matrix.
    let col0 = basis.col_a();
    let col1 = basis.col_b();
    let col2 = basis.col_c();
    let sx: real = ComplexField::sqrt(col0.x * col0.x + col0.y * col0.y + col0.z * col0.z);
    let sy: real = ComplexField::sqrt(col1.x * col1.x + col1.y * col1.y + col1.z * col1.z);
    let sz: real = ComplexField::sqrt(col2.x * col2.x + col2.y * col2.y + col2.z * col2.z);
    if sx == 0.0 || sy == 0.0 || sz == 0.0 {
        return Rotation::from_xyzw(0.0, 0.0, 0.0, 1.0);
    }
    // Normalize columns to get rotation matrix, accounting for negative determinant
    let det = basis.determinant();
    let sign = if det < 0.0 { -1.0 } else { 1.0 };
    let inv_sx = sign / sx;
    let inv_sy = 1.0 / sy;
    let inv_sz = 1.0 / sz;
    // Rotation matrix elements (row-major access: m[row][col])
    // Godot's Basis stores columns, so col_a() = first column, etc.
    // m00 = col0.x/sx, m10 = col0.y/sx, m20 = col0.z/sx
    // m01 = col1.x/sy, m11 = col1.y/sy, m21 = col1.z/sy
    // m02 = col2.x/sz, m12 = col2.y/sz, m22 = col2.z/sz
    let m00 = col0.x * inv_sx;
    let m10 = col0.y * inv_sx;
    let m20 = col0.z * inv_sx;
    let m01 = col1.x * inv_sy;
    let m11 = col1.y * inv_sy;
    let m21 = col1.z * inv_sy;
    let m02 = col2.x * inv_sz;
    let m12 = col2.y * inv_sz;
    let m22 = col2.z * inv_sz;
    // Shepperd's method: find the largest quaternion component to avoid division by near-zero.
    let trace = m00 + m11 + m22;
    let (x, y, z, w);
    if trace > 0.0 {
        let s: real = ComplexField::sqrt(trace + 1.0) * 2.0; // s = 4*w
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else if m00 > m11 && m00 > m22 {
        let s: real = ComplexField::sqrt(1.0 + m00 - m11 - m22) * 2.0; // s = 4*x
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if m11 > m22 {
        let s: real = ComplexField::sqrt(1.0 + m11 - m00 - m22) * 2.0; // s = 4*y
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        let s: real = ComplexField::sqrt(1.0 + m22 - m00 - m11) * 2.0; // s = 4*z
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }
    // Normalize the quaternion deterministically
    let len: real = ComplexField::sqrt(x * x + y * y + z * z + w * w);
    if len > 0.0 {
        Rotation::from_xyzw(x / len, y / len, z / len, w / len)
    } else {
        Rotation::from_xyzw(0.0, 0.0, 0.0, 1.0)
    }
}
pub fn vector_length(vector: Vector) -> real {
    #[cfg(feature = "dim2")]
    {
        ComplexField::sqrt(vector.x * vector.x + vector.y * vector.y)
    }
    #[cfg(feature = "dim3")]
    {
        ComplexField::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
    }
}
pub fn vector_normalized(vector: Vector) -> Vector {
    if vector != Vector::ZERO {
        #[cfg(feature = "dim2")]
        {
            let len: real =
                ComplexField::sqrt(vector.x * vector.x + vector.y * vector.y);
            if len > 0.0 {
                return Vector2::new(vector.x / len, vector.y / len);
            }
        }
        #[cfg(feature = "dim3")]
        {
            let len: real = ComplexField::sqrt(
                vector.x * vector.x + vector.y * vector.y + vector.z * vector.z,
            );
            if len > 0.0 {
                return Vector3::new(vector.x / len, vector.y / len, vector.z / len);
            }
        }
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
#[cfg(feature = "dim2")]
#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use godot::builtin::math::assert_eq_approx;

    use super::*;
    #[test]
    fn test_transform_update_does_not_panic() {
        let transform = Transform2D::from_cols(
            Vector2::new(0., 0.),
            Vector2::new(0., 0.),
            Vector2::new(0., 0.),
        );
        let new_transform =
            transform_update(&transform, Rotation::identity(), Vector2::new(-0., 0.));
        assert_eq!(
            new_transform,
            Transform2D::from_cols(
                Vector2::new(0., 0.),
                Vector2::new(0., 0.),
                Vector2::new(0., 0.),
            )
        );
    }
    #[test]
    fn test_transform_update() {
        let transforms = [
            Transform2D::from_cols(
                Vector2::new(1., 1.),
                Vector2::new(0., 1.),
                Vector2::new(0., 0.),
            ),
            Transform2D::from_cols(
                Vector2::new(1., 2.),
                Vector2::new(0., 3.),
                Vector2::new(0., 0.),
            ),
            Transform2D::from_cols(
                Vector2::new(5., 0.),
                Vector2::new(0., 2.),
                Vector2::new(0., 0.),
            ),
            Transform2D::from_cols(
                Vector2::new(5., -5.),
                Vector2::new(0., 2.),
                Vector2::new(0., 0.),
            ),
        ];
        let new_rotations = [0., -PI, PI / 2., PI / 5.];
        for i in 0..transforms.len() {
            let transform = transforms[i];
            let new_rotation = new_rotations[i];
            let new_origin = Vector2::new(-50., 30.);
            let new_transform =
                transform_update(&transform, Rotation::from_angle(new_rotation), new_origin);
            let skew = if !transform.determinant().is_zero_approx() {
                transform.skew()
            } else {
                0.
            };
            assert_eq_approx!(
                new_transform,
                Transform::from_angle_scale_skew_origin(
                    new_rotation,
                    transform.scale(),
                    skew,
                    new_origin
                )
            );
        }
    }
    #[test]
    fn test_transform_update_accuracy() {
        let mut transform = Transform2D::from_cols(
            Vector2::new(1., 0.),
            Vector2::new(0., 1.),
            Vector2::new(0., 0.),
        );
        for i in 0..1000 {
            println!("{}", PI * (3123. * i as f32).cos());
            transform = transform_update(
                &transform,
                Rotation::from_angle(PI * (3123. * i as f32).cos()),
                Vector2::new(0., 0.),
            );
        }
        assert_eq_approx!(transform.scale(), Vector2::ONE);
        assert_eq_approx!(transform.skew(), 0.);
    }
}
