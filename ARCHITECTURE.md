# Architecture

The Godot Rapier data is organized in two layers:
- Godot Data Layer: This maps to what Godot sends to the plugin.
- Rapier Data Layer: This is data converted from Godot to match Rapier Lib data.

This data is held in a **Singleton** for performance reasons.

```mermaid
classDiagram
direction LR
namespace Godot Data Layer {
    class RapierSpace {
        stateless_data: Data
        state: RapierSpaceState
    }
    class RapierSpaceState {
        stateful_data: Data
        handle: Index
    }

    class IRapierShape {
        stateless_data: Data
        base: RapierShapeBase
    }
    class RapierShapeBase {
        stateless_data: Data
        state: RapierSpaceState
    }
    class RapierShapeBaseState {
        stateful_data: Data
        world_handle: Index
        handle: Index
    }

    class IRapierJoint {
        stateless_data: Data
        base: RapierJointBase
    }
    class RapierJointBase {
        stateless_data: Data
        base: RapierJointBase
    }
    class RapierJointBaseState {
        stateful_data: Data
        world_handle: Index
        handle: Index
    }

    class IRapierCollisionObject{
        stateless_data: Data
        base: RapierCollisionObjectBase
        state: RapierCollisionObjectState
    }

    class RapierCollisionObjectBase {
        stateless_data: Data
        base: RapierCollisionObjectBaseState
    }

    class RapierCollisionObjectBaseState {
        stateful_data: Data
        world_handle: Index
        handle: Index
    }

    class RapierFluid {
        effects: Vec[IRapierFluidEffects]
    }
    class IRapierFluidEffects {
    }
}
namespace Rapier Data Layer {
    class PhysicsEngine {
        physics_worlds: Arena[Index, PhysicsWorld]
        shapes: Arena[Index, SharedShape]
    }
    class PhysicsWorld {
        physics_objects: PhysicsObjects
        fluids_pipeline: FluidsPipeline
    }
    class PhysicsObjects {
        impulse_joint_set: Arena[Index, ImpulseJoint]
        multi_body_joint_set: Arena[Index, MultiBodyJoint]
        rigid_body_set: Arena[Index, Rigidbody]
        collider_set: Arena[Index, Collider]
        handle: Index
    }
}
    class Singleton {
        shapes: HashMap[Rid, RapierShape]
        spaces: HashMap[Rid, RapierSpace]
        collision_objects: HashMap[Rid, RapierCollisionObject]
        joints: HashMap[Rid, RapierJoint]
        fluids: HashMap[Rid, RapierFluid]
        physics_engine: PhysicsEngine
        ---
        rids: HashMap[Index, Rid]
    }

    Singleton *-- RapierSpace : rid
    Singleton *-- IRapierShape : rid
    Singleton *-- IRapierJoint : rid
    Singleton *-- IRapierCollisionObject : rid
    Singleton *-- RapierFluid : rid
    Singleton *-- PhysicsEngine
    PhysicsEngine *-- PhysicsWorld : space_handle
    PhysicsWorld *-- PhysicsObjects : space_handle

    RapierSpace *-- RapierSpaceState
    RapierSpaceState ..> PhysicsWorld : space_handle

    IRapierShape *-- RapierShapeBase
    RapierShapeBase *-- RapierShapeBaseState
    RapierShapeBaseState ..> PhysicsEngine : shape_handle

    IRapierJoint *-- RapierJointBase
    RapierJointBase *-- RapierJointBaseState
    RapierJointBaseState ..> PhysicsObjects : impulse_joit_handle

    IRapierCollisionObject *-- RapierCollisionObjectBase
    RapierCollisionObjectBase *-- RapierCollisionObjectBaseState
    RapierCollisionObjectBaseState ..> PhysicsObjects : rigidbody_handle

    RapierFluid *-- IRapierFluidEffects
```

## Spaces

Spaces hold some statelss data, and some data that holds state. The stateful data also holds an index to the **PhysicsWorld** it simulates. The physics world is located on the **Singleton**.

```mermaid
classDiagram
namespace Godot Data Layer {
}
RapierSpace *-- RapierSpaceState
PhysicsWorld <.. RapierSpaceState
```

## Joint

```mermaid
classDiagram
    class RapierJointBase {
        +f32 max_force
        +bool disabled_collisions_between_bodies
        +RapierJointBaseState state
        +new(WorldHandle space_handle, JointHandle handle) RapierJointBase
        +get_handle() JointHandle
        +get_space_handle() WorldHandle
        +get_space(PhysicsRids physics_rids) Rid
        +set_max_force(f32 force)
        +get_max_force() f32
        +is_valid() bool
        +disable_collisions_between_bodies(bool disabled, PhysicsEngine physics_engine)
        +is_disabled_collisions_between_bodies() bool
        +copy_settings_from(RapierJointBase joint, PhysicsEngine physics_engine)
        +destroy_joint(PhysicsEngine physics_engine, PhysicsRids physics_rids)
    }

    class RapierJointBaseState {
        +JointHandle handle
        +WorldHandle space_handle
    }

    class PhysicsEngine {
        +joint_change_disable_collision(WorldHandle space_handle, JointHandle handle, bool disabled)
        +destroy_joint(WorldHandle space_handle, JointHandle handle)
    }

    class PhysicsRids {
    }

    RapierJointBase --> RapierJointBaseState : has
    RapierJointBase --> PhysicsEngine : interacts with
    RapierJointBase --> PhysicsRids : interacts with
```