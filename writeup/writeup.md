# Project: Building a Controller
## Write Up

## Scenario 1

### Task
    - Make the drone stay stable for at least .8 seconds
    
### How it was Completed
    - To pass this task mass was changed to 0.5 from 0.4
    - PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds

![scenario1](./writeup/scenario1.png)

## Scenario 2

### Task
    - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
    - roll rate should less than 2.5 radian/sec for 0.75 seconds

### How it was Completed
    - Changed kpBank from 5 to 14
    - Changed kpYaw from 1 to 3
    - Finally, changed kpPQR from (23, 23, 5) to (95, 95, 5)
    - These changes allowed me to pass this scenario
    - PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
    - PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

## Scenario 3

### Task
    - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
    - Quad2 yaw should be within 0.1 of the target for at least 1 second

### How it was Completed
    - Changed kpPosXY from 1 to 20
    - Changed kpPosZ from 1 to 20
    - Changed kpVelXY from 4 to 6.5
    - Changed kpVelZ from 4 to 7.5
    - PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
    - PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
    - PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds

## Scenario 4
### Task
    - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds
    
### How it was Completed
    - 

## Scenario 5

### Task
    - position error of the quad should be less than 0.25 meters for at least 3 seconds
    
### How it was Completed
    - Keeping the same parameters from Scenario 3, but changing the kpPosXY from 20 to 35
    - PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
