# Project: Building a Controller
## Write Up

## Scenario 1

### Task

- Make the drone stay stable for at least .8 seconds
    
### How it was Completed

- To pass this task mass was changed to 0.5 from 0.4

```
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```

![scenario1](.img/scenario1.png)

## Scenario 2

### Task

- roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
- roll rate should less than 2.5 radian/sec for 0.75 seconds

### How it was Completed

- Changed kpBank from 5 to 14
- Changed kpYaw from 1 to 3
- Finally, changed kpPQR from (23, 23, 5) to (95, 95, 5)
- These changes allowed me to pass this scenario

```
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

![scenario2](.img/scenario2.png)

## Scenario 3

### Task

- X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
- Quad2 yaw should be within 0.1 of the target for at least 1 second

### How it was Completed

- Changed kpPosXY from 1 to 20
- Changed kpPosZ from 1 to 20
- Changed kpVelXY from 4 to 6.5
- Changed kpVelZ from 4 to 7.5

```
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
```

![scenario3](.img/scenario3.png)

## Scenario 4

### Task

- position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds
    
### How it was Completed

- 

```
PASS:
PASS:
PASS:
```

![scenario4](.img/scenario4.png)

## Scenario 5

### Task

- position error of the quad should be less than 0.25 meters for at least 3 seconds
    
### How it was Completed

- Keeping the same parameters from Scenario 3, but changing the kpPosXY from 20 to 35

```
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```

![scenario5](.img/scenario5.png)

# Implemented Controller Write Up

## Body Rate Control

- eh

```
float u_bar_p = kpPQR.x * (pqrCmd.x - pqr.x) * Ixx;
float u_bar_q = kpPQR.y * (pqrCmd.y - pqr.y) * Iyy;
float u_bar_r = kpPQR.z * (pqrCmd.z - pqr.z) * Izz;

momentCmd.x = u_bar_p;
momentCmd.y = u_bar_q;
momentCmd.z = u_bar_r;
```

## Roll Pitch Control

- eh

```
float b_x = R(0, 2);
float b_y = R(1, 2);
float R21 = R(1, 0);
float R11 = R(0, 0);
float R22 = R(1, 1);
float R12 = R(0, 1);
float R33 = R(2, 2);

float c = -collThrustCmd / mass;
float b_x_c = accelCmd.x / c;
float b_y_c = accelCmd.y / c;

float b_x_err = b_x_c - b_x;
float b_y_err = b_y_c - b_y;

float b_x_p_term = kpBank * b_x_err;
float b_y_p_term = kpBank * b_y_err;

float b_x_commanded_dot = b_x_p_term;
float b_y_commanded_dot = b_y_p_term;

float p_c = (R21 * b_x_commanded_dot - R11 * b_y_commanded_dot) / R33;
float q_c = (R22 * b_x_commanded_dot - R12 * b_y_commanded_dot) / R33;

pqrCmd.x = p_c;
pqrCmd.y = q_c;
pqrCmd.z = 0.0;
```

## Altitude Controller

- eh

```
float z_err = posZCmd - posZ;
float z_err_dot = velZCmd - velZ;

float b_z = R(2, 2);

float p_term = kpPosZ * z_err;
float d_term = kpVelZ * z_err_dot;

integratedAltitudeError += z_err * dt;

float u_1_bar = p_term + d_term + accelZCmd + integratedAltitudeError;
float c = (u_1_bar - CONST_GRAVITY)/b_z;

thrust = c * -mass;
```

## Lateral Position

- eh

```
float c = 1;
float x_err = posCmd.x - pos.x;
float x_err_dot = velCmd.x - vel.x;

float y_err = posCmd.y - pos.y;
float y_err_dot = velCmd.y - vel.y;

float p_term_x = kpPosXY * x_err;
float d_term_x = kpVelXY * x_err_dot;

float p_term_y = kpPosXY * y_err;
float d_term_y = kpVelXY * y_err_dot;

float x_dot_dot_command = p_term_x + d_term_x + accelCmd.x;
float y_dot_dot_command = p_term_y + d_term_y + accelCmd.y;

float b_x_c = x_dot_dot_command / c;
float b_y_c = y_dot_dot_command / c;

accelCmd.x = b_x_c;
accelCmd.y = b_y_c;
accelCmd.z = 0;
```

## Yaw Control

- eh

```
float p_err = yawCmd - yaw;
yawRateCmd = kpYaw * p_err;
```

## Motor Commands

- To get the motor commands is to find out how to calculate the front/rear left and right thrusters

```
float l = L / sqrt(2);

float A = collThrustCmd;
float B = momentCmd.x / l;
float C = momentCmd.y / l;
float D = -momentCmd.z / kappa;

float Thr1 = (A + B + C + D) / 4.0;
float Thr2 = (A - B + C - D) / 4.0;
float Thr3 = (A + B - C - D) / 4.0;
float Thr4 = (A - B - C + D) / 4.0;

cmd.desiredThrustsN[0] = CONSTRAIN(Thr1, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[1] = CONSTRAIN(Thr2, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[2] = CONSTRAIN(Thr3, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[3] = CONSTRAIN(Thr4, minMotorThrust, maxMotorThrust);
```
