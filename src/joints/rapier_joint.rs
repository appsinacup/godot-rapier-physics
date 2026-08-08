use godot::classes::*;
use joints::rapier_empty_joint::RapierEmptyJoint;
#[cfg(feature = "dim2")]
use joints::rapier_groove_joint_2d::RapierGrooveJoint2D;
#[cfg(feature = "dim2")]
use physics_server_2d::JointParam;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

#[cfg(feature = "dim3")]
use super::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D;
#[cfg(feature = "dim2")]
use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use super::rapier_fixed_joint::RapierFixedJoint;
#[cfg(feature = "dim3")]
use super::rapier_generic_6dof_joint_3d::RapierGeneric6DOFJoint3D;
use super::rapier_joint_base::RapierJointBase;
use super::rapier_revolute_joint::RapierRevoluteJoint;
use super::rapier_rope_joint::RapierRopeJoint;
#[cfg(feature = "dim3")]
use super::rapier_slider_joint_3d::RapierSliderJoint3D;
#[cfg(feature = "dim3")]
use super::rapier_spherical_joint_3d::RapierSphericalJoint3D;
#[cfg(feature = "dim2")]
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::*;
macro_rules! impl_rapier_joint_base {
    ($ty:ident, $type_expr:expr) => {
        impl IRapierJoint for $ty {
            fn get_type(&self) -> JointType {
                $type_expr
            }

            fn get_base(&self) -> &RapierJointBase {
                &self.base
            }

            fn get_mut_base(&mut self) -> &mut RapierJointBase {
                &mut self.base
            }
        }
    };
}
pub(crate) use impl_rapier_joint_base;
#[allow(clippy::large_enum_variant)]
pub enum RapierJoint {
    #[cfg(feature = "dim2")]
    RapierDampedSpringJoint2D(RapierDampedSpringJoint2D),
    #[cfg(feature = "dim2")]
    RapierGrooveJoint2D(RapierGrooveJoint2D),
    RapierEmptyJoint(RapierEmptyJoint),
    RapierFixedJoint(RapierFixedJoint),
    RapierRopeJoint(RapierRopeJoint),
    RapierRevoluteJoint(RapierRevoluteJoint),
    #[cfg(feature = "dim3")]
    RapierSliderJoint3D(RapierSliderJoint3D),
    #[cfg(feature = "dim3")]
    RapierConeTwistJoint3D(RapierConeTwistJoint3D),
    #[cfg(feature = "dim3")]
    RapierSphericalJoint3D(RapierSphericalJoint3D),
    #[cfg(feature = "dim3")]
    RapierGeneric6DOFJoint3D(RapierGeneric6DOFJoint3D),
}
macro_rules! impl_rapier_joint_trait {
    ($enum_name:ident, $($variant:ident),*) => {
        impl IRapierJoint for $enum_name {
            fn get_base(&self) -> &RapierJointBase {
                match self {
                    $(Self::$variant(joint) => joint.get_base(),)*
                }
            }

            fn get_mut_base(&mut self) -> &mut RapierJointBase {
                match self {
                    $(Self::$variant(joint) => joint.get_mut_base(),)*
                }
            }

            fn get_type(&self) -> JointType {
                match self {
                    $(Self::$variant(joint) => joint.get_type(),)*
                }
            }

            #[cfg(feature = "dim2")]
            fn set_joint_param(
                &mut self,
                param: JointParam,
                value: f32,
                physics_engine: &mut PhysicsEngine,
            ) {
                match self {
                    $(Self::$variant(joint) => joint.set_joint_param(param, value, physics_engine),)*
                }
            }

            #[cfg(feature = "dim2")]
            fn get_joint_param(&self, param: JointParam) -> f32 {
                match self {
                    $(Self::$variant(joint) => joint.get_joint_param(param),)*
                }
            }
        }
    };
}
#[cfg(feature = "dim3")]
impl_rapier_joint_trait!(
    RapierJoint,
    RapierEmptyJoint,
    RapierFixedJoint,
    RapierRopeJoint,
    RapierRevoluteJoint,
    RapierSliderJoint3D,
    RapierConeTwistJoint3D,
    RapierSphericalJoint3D,
    RapierGeneric6DOFJoint3D
);
#[cfg(feature = "dim2")]
impl_rapier_joint_trait!(
    RapierJoint,
    RapierDampedSpringJoint2D,
    RapierGrooveJoint2D,
    RapierEmptyJoint,
    RapierFixedJoint,
    RapierRopeJoint,
    RapierRevoluteJoint
);
pub trait IRapierJoint {
    fn get_base(&self) -> &RapierJointBase;
    fn get_mut_base(&mut self) -> &mut RapierJointBase;
    fn get_type(&self) -> JointType;

    /// Named apart from each joint type's own `set_param`, which takes that type's specific
    /// param enum. Joint types that map a generic param onto a rapier joint override this so
    /// they can rebuild the rapier joint when it changes.
    #[cfg(feature = "dim2")]
    fn set_joint_param(
        &mut self,
        param: JointParam,
        value: f32,
        _physics_engine: &mut PhysicsEngine,
    ) {
        if param == JointParam::MAX_FORCE {
            self.get_mut_base().set_max_force(value);
        }
    }

    #[cfg(feature = "dim2")]
    fn get_joint_param(&self, param: JointParam) -> f32 {
        if param == JointParam::MAX_FORCE {
            self.get_base().get_max_force()
        } else {
            0.0
        }
    }
}
