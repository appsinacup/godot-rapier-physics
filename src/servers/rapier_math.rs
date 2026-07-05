use godot::prelude::*;
use rapier::na::ComplexField;
use rapier::na::RealField;
#[derive(GodotClass)]
#[cfg_attr(feature = "dim2", class(base=Object, init, rename=RapierMath2D))]
#[cfg_attr(feature = "dim3", class(base=Object, init, rename=RapierMath3D))]
/// Contains Rapier Deterministic Math functions.
pub struct RapierMath {
    base: Base<Object>,
}
#[godot_api]
impl RapierMath {
    #[func]
    /// Deterministically compute acos.
    fn acos(x: real) -> real {
        ComplexField::acos(x)
    }

    #[func]
    /// Deterministically compute acosh.
    fn acosh(x: real) -> real {
        ComplexField::acosh(x)
    }

    #[func]
    /// Deterministically compute asin.
    fn asin(x: real) -> real {
        ComplexField::asin(x)
    }

    #[func]
    /// Deterministically compute asinh.
    fn asinh(x: real) -> real {
        ComplexField::asinh(x)
    }

    #[func]
    /// Deterministically compute atan.
    fn atan(x: real) -> real {
        ComplexField::atan(x)
    }

    #[func]
    /// Deterministically compute atanh.
    fn atanh(x: real) -> real {
        ComplexField::atanh(x)
    }

    #[func]
    /// Deterministically compute cos.
    fn cos(x: real) -> real {
        ComplexField::cos(x)
    }

    #[func]
    /// Deterministically compute cosh.
    fn cosh(x: real) -> real {
        ComplexField::cosh(x)
    }

    #[func]
    /// Deterministically compute log.
    fn log(x: real) -> real {
        ComplexField::ln(x)
    }

    #[func]
    /// Deterministically compute sin.
    fn sin(x: real) -> real {
        ComplexField::sin(x)
    }

    #[func]
    /// Deterministically compute sinh.
    fn sinh(x: real) -> real {
        ComplexField::sinh(x)
    }

    #[func]
    /// Deterministically compute tan.
    fn tan(x: real) -> real {
        ComplexField::tan(x)
    }

    #[func]
    /// Deterministically compute tanh.
    fn tanh(x: real) -> real {
        ComplexField::tanh(x)
    }

    #[func]
    /// Deterministically compute sqrt.
    fn sqrt(x: real) -> real {
        if let Some(result) = ComplexField::try_sqrt(x) {
            return result;
        }
        real::NAN
    }

    #[func]
    /// Deterministically compute cbrt (cube root).
    fn cbrt(x: real) -> real {
        ComplexField::cbrt(x)
    }

    #[func]
    /// Deterministically compute the four-quadrant arctangent of y and x.
    fn atan2(y: real, x: real) -> real {
        RealField::atan2(y, x)
    }

    #[func]
    /// Deterministically compute base raised to the power of exponent.
    fn pow(base: real, exponent: real) -> real {
        ComplexField::powf(base, exponent)
    }

    #[func]
    /// Deterministically compute the length of the hypotenuse of a right-angle triangle (sqrt(x*x + y*y)).
    fn hypot(x: real, y: real) -> real {
        ComplexField::hypot(x, y)
    }

    #[func]
    /// Deterministically compute e raised to the power of x.
    fn exp(x: real) -> real {
        ComplexField::exp(x)
    }

    #[func]
    /// Deterministically compute 2 raised to the power of x.
    fn exp2(x: real) -> real {
        ComplexField::exp2(x)
    }

    #[func]
    /// Deterministically compute the base-2 logarithm.
    fn log2(x: real) -> real {
        ComplexField::log2(x)
    }

    #[func]
    /// Deterministically compute the base-10 logarithm.
    fn log10(x: real) -> real {
        ComplexField::log10(x)
    }
}
