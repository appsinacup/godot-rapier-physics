# Architecture

The Godot Rapier data is organized in two layers:
- Godot Data Layer: This maps to what Godot sends to the plugin.
- Rapier Data Layer: This is data converted from Godot to match Rapier Lib data.

This data is held in a **Singleton** for performance reasons.

```mermaid
classDiagram
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
    RapierJointBaseState ..> PhysicsObjects : impulse_joint_handle

    IRapierCollisionObject *-- RapierCollisionObjectBase
    RapierCollisionObjectBase *-- RapierCollisionObjectBaseState
    RapierCollisionObjectBaseState ..> PhysicsObjects : rigidbody_handle

    RapierFluid *-- IRapierFluidEffects
```
