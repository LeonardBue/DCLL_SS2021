# DCLL_SS2021
(Matlab) code distribution for the lecture Dynamics and Control of Legged Locomotion (Summer 2021)

This MATLAB code framework accompanies the lecture **Dynamics and Control for Legged Locomotion** held at the University of Stuttgart.

The code will be updated as we go along the semester. Feel free to look at everything, though not everything is guaranteed to work yet.

## Notes on the code
It includes several examples of different controllers, models, and contact solvers. You can plug together the following types:

| Controller          | Model                 | Solver                   |
|---------------------|-----------------------|--------------------------|
| Freeze              | 5LinkBiped\_floatBase | EventBased, TimeStepping |
| VMC\_inPlace        | 5LinkBiped\_floatBase | TimeStepping             |
| VMC\_forwardWalking | 5LinkBiped\_floatBase | TimeStepping             |
| HZD                 | 5LinkBiped\_floatBase | EventBased               |
| ZMP                 | 7LinkBiped\_floatBase | TimeStepping             |
| StaticLocomotion    | 7LinkBiped\_floatBase | TimeStepping             |
| Raibert             | SLIP                  | EventBased               |
| PassiveSLIP         | SLIP                  | EventBased               |
| NoControl           | SimplestWalkingModel  | EventBased               |
| NoControl           | All except SLIP       | EventBased, TimeStepping |
