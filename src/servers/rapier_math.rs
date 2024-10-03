use godot::prelude::*;
use rapier::na::ComplexField;
#[cfg(feature = "dim3")]
#[derive(GodotClass)]
#[class(base=Object, init, rename=RapierMath3D)]
/// Contains Rapier Deterministic Math functions.
pub struct RapierMath {
    base: Base<Object>,
}
#[cfg(feature = "dim2")]
#[derive(GodotClass)]
#[class(base=Object, init, rename=RapierMath2D)]
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
}
