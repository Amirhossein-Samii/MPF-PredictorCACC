# MPF-PredictorCACC

## Description

This repository contains the MATLAB code accompanying the paper
"Predictor-Feedback CACC for Vehicular Platoons with Actuation and Communication Delays Based on a Multiple-Predecessor-Following CTH Nominal Strategy"
([arXiv:2604.05667](https://arxiv.org/abs/2604.05667)).

The paper develops a predictor-feedback cooperative adaptive cruise control (CACC) design built on a multiple-predecessor-following (MPF) constant-time-headway (CTH) nominal delay-free law, for heterogeneous platoons with third-order linear vehicle dynamics, actuation delay, and V2V communication delay.

The script in this repository simulates the ten-vehicle platoon (a leader and nine followers) under the proposed design:

* Every vehicle has a third-order vehicle's dynamic and a common actuation delay of 0.7 s.
* Each follower receives its predecessors' data over V2V links with vehicle-dependent communication delays.
* Each follower uses a predictor to compensate the actuation delay.
* Followers 1 and 2 use all available predecessors; followers 3 to 9 use the three preceding vehicles.
* The leader performs a braking and acceleration maneuver, and the script plots the resulting spacing, speed, acceleration, and control input of every vehicle.

* The mathematical background and the documentation for the codes are briefly desribed in the file "codesdoc.pdf.".

## Requirements

* MATLAB R2022b or later (the script uses only base MATLAB functions, e.g. `expm`; no additional toolboxes are needed)

## Installation

1. Clone or download this repository:

```
   git clone https://github.com/<Amirhossein-Samii>/<MPF-PredictorCACC>.git
   ```

2. Open MATLAB and navigate to the project folder:

```
   cd /path/to/<MPF-PredictorCACC>
   ```

## Usage

1. Open MATLAB and make sure you are in the project folder.
2. Run the main script:

```
   MPF_PredictorCACC
   ```

3. Four figures are produced: spacing, speed, acceleration, and control input of all vehicles.

Simulation settings (horizon, step size, actuation delay) are at the top of the script. Each vehicle's communication delay, time constant, time headway, and controller gains (`a_i`, `b_i`, `c_i`) are defined in the "Defining Vehicles and control parameters" section. The leader's maneuver is defined in the "Leader's maneuver" section.

Note: the predictor integrals are evaluated numerically at every time step, so a full run (100 s of simulated time, step 0.01 s) is computationally heavy and may take a while.

### Files

* `MPF_PredictorCACC.m`: ten-vehicle platoon simulation with the MPF predictor-feedback CACC design

## Examples

Leader maneuver used in the script:

|Time (s)|Leader input|
|-|-|
|25 - 27.5|-4 m/s² (hard braking)|
|27.5 - 50|0|
|50 - 55|+2 m/s²|
|after 55|0|

## License

This project is licensed under the CC BY-NC-ND

## Contact

For questions or feedback, please contact <asamii@tuc.gr>.

## Acknowledgements

Funded by the European Union (ERC, C-NORA, 101088147). Views and opinions expressed are however those of the authors only and do not necessarily reflect those of the European Union or the European Research Council Executive Agency. Neither the European Union nor the granting authority can be held responsible for them.

## Cite this work

```
@misc{samii2026mpf,
  title         = {Predictor-Feedback CACC for Vehicular Platoons with Actuation and Communication Delays Based on a Multiple-Predecessor-Following CTH Nominal Strategy},
  author        = {Samii, Amirhossein and Angelopoulos, Dimitrios and Bekiaris-Liberis, Nikolaos},
  year          = {2026},
  eprint        = {2604.05667},
  archivePrefix = {arXiv},
  primaryClass  = {eess.SY},
  doi           = {10.48550/arXiv.2604.05667},
  url           = {https://arxiv.org/abs/2604.05667}
}
```

