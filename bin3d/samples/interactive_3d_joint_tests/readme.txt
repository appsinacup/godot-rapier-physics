These joint tests are minimal scenes set up to
make it quick and easy to toy around with 3D joints 
and verify their behaviour between different engines. 

They are here as a convenience for 
contributors who may be working on the joints
code. They could be deleted if they no 
longer serve any value.

There is a sample scene for every Godot Joint3D.

The compound 6dof sample shows the use of 
two joints compounded into one in order to 
better match the limit behaviour of the Jolt
6dof joint. The Jolt 6dof joint uses swing/twist
axes with a pyramid constraint on the swing.
Rapier can't currently match Jolt's particular
setup with a single joint (as of 0.32.0). 

Another way of matching Jolt 6dof limit behaviour
would be to implement limits using external
colliders to stop the rigid body where you
want it limited.

This behaviour difference, along with workarounds
may be something worth documenting for users in
the future. We could also include workaround 
samples like the one here, if users actually hit
up against these issues in practice.
