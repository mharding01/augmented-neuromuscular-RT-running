The library *contactGeom* can be used to detect and apply contact forces and torques due to contacts between different shapes made of simple primitives.

Currently, these primitives are the following:
* *infinite plane* (like the ground)
* *sphere*
* *cuboid*

However, the algorithm implementation is performed in such a way that its structure can be easily extended with new basic primitives. It is possible to combine different primitive shapes to get more complex shapes.

The basic principle of this library is the following. When two shapes are considered, we compute the penetration volume (i.e. the volume of intersection) and its time derivative, based on the kinematics. These two expressions are exact in all cases, except when two infinite planes are considered (volume and derivative set to 0) or when we compute the contact between a sphere and a cuboid (an approximation is provided). From the penetration volume and its derivative, a normal force is computed. Then, some points at the intersection of the two shapes are selected. Their relative velocity and distributed normal force are computed, resulting in a friction force. Finally, everything is gather in a F sensor, providing force and torque component.

In *contactGeom*, the folder [config](userFiles/simu/forces/contactGeom/config) is used to let the user configure its contact model with the provided library. All the requested information can be found in the manual placed inside each file.

Here is a quick overview of the configuration files in the folder *config* (as previously explained, check the manual inside the different files for more explanation):

* *UserShapeModule.cc* and *UserShapeModule.hh*: This class is not used by the contact library, but is located inside each shape. The user can modify this class to add new fields and new methods specific to some shapes (like friction coefficient).
* *user_contact_force.cc*: In this file, the user can implement its own law to define the normal force computation based on the penetration volume (and its derivative) and to define the friction force based on the relative motion and the normal force.
* *user_contact_kinematics.cc*: This file is only for more advanced users who want to improve the computational speed of the model.
* *user_contact_states.cc*: This file is called by the library each time step for each basic shape, but is empty. There, the user can implement state changes (like stiction vs. sliding) if needed.
* *user_shapes.cc*: This is the main file to define the shapes involved in the simulation for the contact model. Their definition is done in such a way that it is similar to the values introduced inside *MBsysPad* for the visualization of the corresponding primitives.

When using this library, some tuning might be necessary (for the stiffness of the normal force, the friction coefficient, the time step...). In particular, you can easily get a singular matrix in the two following cases:
* The time step is too big and the normal force too important.
* For flying objects, if the configuration *R1-R2-R3* is used, the problem becomes singular in some cases like when R2 is close to -pi/2 or +pi/2.
