# Project: Building a Controller
## Write Up

### Scenario 2

   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

### Scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


### Scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

### Scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds



## Implemented body rate control in C++.

The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

## Implement roll pitch control in C++.

The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

## Implement altitude controller in C++.

The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

## Implement lateral position control in C++.

The controller should use the local NE position and velocity to generate a commanded local acceleration.

## Implement yaw control in C++.

The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

## Implement calculating the motor commands given commanded thrust and moments in C++.

The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.