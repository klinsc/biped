Coordinate & Sign Convention

View reference: มองจากมุมมองของหุ่นไปด้านหน้า (Front view of the robot)

Axes: X หน้า, Y ซ้าย, Z ขึ้น (Front, Left, Up) //! Wait for confirmation

Positive rotation definitions:
- L_HIP_ROLL + : ขาเอียงออก (Leg rolls outward)
- L_HIP_PITCH + : ขากางไปข้างหน้า (Leg pitches forward)
- L_KNEE_PITCH + : ขางอไปข้างหน้า (Knee pitches forward)
- L_ANKLE_PITCH + : เท้าเตะไปข้างหน้า (Ankle pitches forward)
- L_ANKLE_ROLL + : เท้าเอียงทำให้ขาเอียงเข้าด้านใน (Ankle rolls inward)
- R_HIP_ROLL + : ขาเอียงออก (Leg rolls outward)
- R_HIP_PITCH + : ขากางไปข้างหน้า (Leg pitches forward)
- R_KNEE_PITCH + : ขางอไปข้างหน้า (Knee pitches forward)
- R_ANKLE_PITCH + : เท้าเตะไปข้างหน้า (Ankle pitches forward)
- R_ANKLE_ROLL + : เท้าเอียงทำให้ขาเอียงเข้าด้านใน (Ankle rolls inward)

Kinematic Tree (ASCII)
                TORSO
                  |
        ---------------------
        |                   |
     L_HIP_ROLL(8)       R_HIP_ROLL(9)
        |                   |
     L_HIP_PITCH(3)      R_HIP_PITCH(7)
        |                   |
     L_KNEE_PITCH(2)     R_KNEE_PITCH(6)
        |                   |
   L_ANKLE_PITCH(1)    R_ANKLE_PITCH(5)
        |                   |
   L_ANKLE_ROLL(0)     R_ANKLE_ROLL(4)
        |                   |
      L_FOOT             R_FOOT


Servo Map Table
| name          | channel | side | joint       | designation           |
| ------------- | ------: | ---- | ----------- | --------------------- |
| L_ANKLE_ROLL  |       0 | L    | ANKLE_ROLL  | Wait for confirmation |
| L_ANKLE_PITCH |       1 | L    | ANKLE_PITCH | Wait for confirmation |
| L_KNEE_PITCH  |       2 | L    | KNEE_PITCH  | Wait for confirmation |
| L_HIP_PITCH   |       3 | L    | HIP_PITCH   | Wait for confirmation |
| L_HIP_ROLL    |       8 | L    | HIP_ROLL    | Wait for confirmation |
| R_ANKLE_ROLL  |       4 | R    | ANKLE_ROLL  | Wait for confirmation |
| R_ANKLE_PITCH |       5 | R    | ANKLE_PITCH | Wait for confirmation |
| R_KNEE_PITCH  |       6 | R    | KNEE_PITCH  | Wait for confirmation |
| R_HIP_PITCH   |       7 | R    | HIP_PITCH   | Wait for confirmation |
| R_HIP_ROLL    |       9 | R    | HIP_ROLL    | Wait for confirmation |