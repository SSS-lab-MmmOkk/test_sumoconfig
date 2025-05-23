# Framework Report: AV Speed Optimization with PET Constraints

## 1. Introduction

The problem at hand is to develop a framework for optimizing the speed of an Autonomous Vehicle (AV) in a simulated urban environment. The primary goal is to minimize the AV's traversal time through a specific segment (Junction J2), while adhering to safety constraints defined by Post Encroachment Time (PET) with respect to pedestrians. The framework involves simulating AV and pedestrian interactions, calculating PET values, and using an optimization algorithm to determine an optimal AV speed. This report details the architecture, methodology, and implementation of this framework.

## 2. System Architecture

The framework consists of three main Python scripts:

*   **`sumo_interface.py`**: This script manages the interaction with the SUMO (Simulation of Urban MObility) traffic simulator. It is responsible for:
    *   Launching SUMO with a specific scenario configuration (`config/ped250406.sumocfg`).
    *   Controlling the speeds of the AV (`car_0`) and two pedestrians (`ped_1`, `ped_2`) based on input parameters.
    *   Collecting trajectory data (time, x, y coordinates) for these entities during each simulation run.
    *   Calculating the AV's traversal time through a predefined segment of Junction J2.
    *   Invoking the PET calculation functions from `pet_calculator.py`.
    *   Providing an interface (`execute_single_sumo_run`) for the optimizer to run individual simulation scenarios and retrieve results.

*   **`pet_calculator.py`**: This script contains the logic for calculating PET values.
    *   It includes helper functions for geometric operations, primarily point-in-polygon tests. It attempts to use the `shapely` library for robustness and falls back to a manual ray-casting algorithm if `shapely` is unavailable.
    *   The core function `_get_entry_exit_times` determines the time intervals during which participants (AV or pedestrian) are inside a given encroachment area.
    *   The main function `calculate_pet` takes the trajectories of two participants and the encroachment area polygon to compute the PET. A negative PET indicates an overlap in time within the area.

*   **`optimizer.py`**: This script implements the optimization logic.
    *   It defines the search space for the AV's speed.
    *   It iteratively calls `execute_single_sumo_run` from `sumo_interface.py` with different AV speeds and stochastically sampled pedestrian speeds.
    *   It evaluates each AV speed based on the average J2 traversal time and the probability of satisfying PET constraints (PET >= 2.0s with both pedestrians).
    *   It employs a grid search algorithm to find an AV speed that minimizes traversal time subject to the PET safety constraint probability.

## 3. Methodology

### 3.1. Optimization Problem

*   **Objective**: Minimize the average traversal time of the AV (`car_0`) through the internal lane of Junction J2 (`:J2_4_0`).
*   **Decision Variable**: The target speed of the AV (`car_0`).
*   **Constraints**: The probability that the PET values between the AV and `ped_1` (in `encroachment_area_ped1`) AND between the AV and `ped_2` (in `encroachment_area_ped2`) are both greater than or equal to 2.0 seconds must be at least 90%.

### 3.2. Post Encroachment Time (PET) Calculation

*   **Method**: PET is calculated as the temporal gap between the exit of the first road user from a defined conflict area and the entry of the second road user into the same area.
    *   If the time intervals of the two road users in the conflict area overlap, the PET is negative, representing the duration of the overlap.
    *   If the intervals touch (e.g., one exits at the exact moment the other enters), PET is 0.
    *   If one or both road users do not enter the area, or do not complete an entry-exit sequence, the PET is considered `float('inf')` (no conflict).
*   **Entry/Exit Times**: Determined by `_get_entry_exit_times` in `pet_calculator.py`. When a trajectory segment crosses the boundary of an encroachment area, the entry/exit time is interpolated as the midpoint of the timestamps of the two points forming that segment.
*   **Defined Encroachment Areas**:
    *   `encroachment_area_ped1`: `[(-7.62, 27.46), (-5.39, 27.46), (-5.39, 29.46), (-7.62, 29.46)]`
    *   `encroachment_area_ped2`: `[(-7.62, 19.06), (-5.39, 19.06), (-5.39, 21.06), (-7.62, 21.06)]`
    These coordinates are used by `optimizer.py` and passed to `sumo_interface.py`.

### 3.3. AV J2 Traversal Time Measurement

The traversal time for the AV (`car_0`) through the J2 junction segment is measured as follows:
1.  The specific internal lane of J2 relevant to `car_0`'s path is identified as `:J2_4_0`.
2.  The simulation time when `car_0` first enters this lane is recorded (`sim_time_car0_enters_j2`).
3.  The simulation time when `car_0` first exits this lane (i.e., its `laneID` changes from `:J2_4_0` to something else) is recorded (`sim_time_car0_exits_j2`).
4.  The traversal time is `sim_time_car0_exits_j2 - sim_time_car0_enters_j2`.
5.  If `car_0` does not enter the lane or does not exit it before the simulation ends, the traversal time is recorded as -1.0 (invalid).

### 3.4. Bayesian Network (BN) Framework (Conceptual)

While a full Bayesian Network was not explicitly implemented in the provided scripts for this phase, the problem implies a probabilistic approach to safety. A conceptual BN for this problem could be structured as follows:

*   **Variables**:
    *   `AV_Speed`: Target speed of the AV (continuous or discretized).
    *   `Ped1_Speed`: Speed of Pedestrian 1 (continuous, sampled from a distribution).
    *   `Ped2_Speed`: Speed of Pedestrian 2 (continuous, sampled from a distribution).
    *   `PET_AV_Ped1`: Calculated PET between AV and Pedestrian 1 (continuous).
    *   `PET_AV_Ped2`: Calculated PET between AV and Pedestrian 2 (continuous).
    *   `J2_Traversal_Time`: Calculated traversal time for AV (continuous).
    *   `Safety_Constraint_Met_Ped1`: Boolean (PET_AV_Ped1 >= 2.0s).
    *   `Safety_Constraint_Met_Ped2`: Boolean (PET_AV_Ped2 >= 2.0s).
    *   `Overall_Safety_Constraint_Met`: Boolean (`Safety_Constraint_Met_Ped1` AND `Safety_Constraint_Met_Ped2`).
    *   `Utility`: A function combining `J2_Traversal_Time` and `Overall_Safety_Constraint_Met` (e.g., high penalty if safety constraint is not met, otherwise value related to negative traversal time).

*   **Dependencies (Conceptual Graph)**:

    ```mermaid
    graph TD
        AV_Speed --> PET_AV_Ped1;
        AV_Speed --> PET_AV_Ped2;
        AV_Speed --> J2_Traversal_Time;
        Ped1_Speed --> PET_AV_Ped1;
        Ped2_Speed --> PET_AV_Ped2;
        PET_AV_Ped1 --> Safety_Constraint_Met_Ped1;
        PET_AV_Ped2 --> Safety_Constraint_Met_Ped2;
        Safety_Constraint_Met_Ped1 --> Overall_Safety_Constraint_Met;
        Safety_Constraint_Met_Ped2 --> Overall_Safety_Constraint_Met;
        J2_Traversal_Time --> Utility;
        Overall_Safety_Constraint_Met --> Utility;
    ```

*   **Uncertainty**: Pedestrian speeds are the primary source of uncertainty, modeled using normal distributions (`PED_MEAN_SPEED_MPS = 1.34`, `PED_STD_DEV_MPS = 0.26`). SUMO's `--random` flag also introduces stochasticity in driver behavior if not fully overridden.
*   **Evaluation**: The grid search in `optimizer.py` approximates the evaluation of this conceptual BN by repeatedly sampling pedestrian speeds and running simulations to estimate the probability of `Overall_Safety_Constraint_Met` and the expected `J2_Traversal_Time` for each `AV_Speed`.

### 3.5. Optimization Algorithm

A **Grid Search** algorithm is implemented in `optimizer.py`:
1.  A range of discrete AV speeds is defined (`AV_MIN_SPEED_MPS` to `AV_MAX_SPEED_MPS` with `NUM_AV_SPEED_STEPS` steps). For the worker check, these were set to 5.0 m/s, 15.0 m/s, and 3 steps respectively.
2.  For each candidate AV speed:
    *   `N_SIM_RUNS_PER_AV_SPEED` (set to 5 for worker check) simulation runs are performed.
    *   In each run, pedestrian speeds are sampled randomly from their defined normal distributions (ensuring a minimum speed of 0.2 m/s).
    *   `execute_single_sumo_run` is called to get PET values and J2 traversal time.
    *   The probability of meeting the PET constraint (PET1 >= 2.0s AND PET2 >= 2.0s) is calculated based on the outcomes of these `N_SIM_RUNS_PER_AV_SPEED` runs.
    *   The average J2 traversal time is calculated from runs where traversal was successfully completed.
3.  **Selection Criteria**: The optimal AV speed is selected from those candidates that meet the `PROB_CONSTRAINT_SATISFIED_THRESHOLD` (90%). Among these, the speed that results in the minimum average J2 traversal time is chosen. If multiple speeds yield the same minimum traversal time, the higher speed is preferred.
4.  If no speed meets the probability threshold, a "best compromise" is reported (maximizing probability first, then minimizing valid traversal time).

## 4. Implementation Details

*   **Key Functions**:
    *   `sumo_interface.execute_single_sumo_run()`: Core function for a single simulation instance.
    *   `pet_calculator.calculate_pet()`: Calculates PET given two trajectories and an area.
    *   `pet_calculator._get_entry_exit_times()`: Determines area presence intervals.
    *   `pet_calculator.is_inside_polygon()`: Point-in-polygon check.
    *   `optimizer.run_optimizer()`: Orchestrates the grid search.
*   **SUMO Scenario Files**:
    *   The primary configuration file used is `config/ped250406.sumocfg`.
    *   Associated network (`.net.xml`), route (`.rou.xml`), and potentially additional files (`.add.xml`) define the environment, vehicle routes, and pedestrian paths. The simulation is run with `--random` for stochasticity. Step length is set to 0.1s.

## 5. Assumptions Made

*   The trajectories provided by SUMO (at 0.1s intervals) are sufficiently detailed for accurate PET calculation using midpoint interpolation for boundary crossings.
*   The J2 traversal segment is adequately represented by the single internal lane `:J2_4_0`.
*   Pedestrian speeds follow a normal distribution, truncated at a minimum of 0.2 m/s.
*   The fixed simulation duration of 200 steps (20 seconds) per run is sufficient for observing the relevant interactions and J2 traversal.
*   The provided encroachment area coordinates are correct and define meaningful conflict zones.
*   `car_0`, `ped_1`, and `ped_2` are the specific entity IDs that will always be present (or attempted to be loaded) in the simulation scenario.

## 6. Limitations

*   **Execution Verification**: Due to persistent `SyntaxError` issues related to the file writing tools (`overwrite_file_with_block`, `create_file_with_block`) when creating `optimizer.py` in the preceding subtask, the final execution of `optimizer.py` could not be performed within the turn limits of the final execution-focused subtask. Therefore, concrete numerical results (optimal AV speed, PET values, traversal times, optimizer runtime) from a full run of `optimizer.py` could not be obtained and reported. The individual components (`sumo_interface.py` and `pet_calculator.py`) were tested and refined, and `shapely` was installed.
*   **Grid Search Granularity**: The chosen `NUM_AV_SPEED_STEPS` (3 for testing) provides a coarse grid. A finer grid (e.g., 10-15 steps) and more simulation runs per speed (e.g., 30-50) would be needed for more robust optimization but were reduced for timely execution in the test environment.
*   **J2 Traversal Definition**: The current J2 traversal relies on entry/exit of a single specified internal lane. If the AV's path through J2 is more complex (multiple segments/lanes), this logic would need refinement.
*   **PET Interpolation**: Midpoint time interpolation for entry/exit is a simplification. More precise geometric intersection calculations could be used for higher accuracy if needed.
*   **Static Pedestrian Behavior Model**: Pedestrian speeds are sampled once per simulation run and remain constant. A more dynamic model (e.g., pedestrians reacting to the AV) is not part of the current scope.
*   **Optimizer Algorithm**: Grid search is basic. More advanced optimization algorithms (e.g., Bayesian optimization, evolutionary algorithms) could be more efficient for larger search spaces.

## 7. Expected Results Format (from `optimizer.py`)

If `optimizer.py` were executed successfully, the console output would include:

1.  Initial messages confirming library imports (Shapely from `pet_calculator.py`, then the import from `sumo_interface.py`).
2.  A header detailing the optimizer's configuration (AV speed range, number of steps, simulations per speed, PET thresholds, SUMO config path).
3.  For each candidate AV speed tested:
    *   A line indicating the AV speed being tested.
    *   For each simulation run under that AV speed:
        *   A line indicating "Run X/N for AV speed Y m/s...".
        *   (If `debug_prints=True` in `sumo_interface.py` as intended for the final test):
            *   "SUMO started. CMD: ..."
            *   Messages about setting AV and pedestrian speeds.
            *   Messages about `car_0` entering/exiting the J2 segment.
            *   "Simulation run finished. AV J2 traversal time: Z.ZZs"
            *   Trajectory point collection counts.
            *   "Vehicle: Entry=..., Exit=..." and "Pedestrian: Entry=..., Exit=..." from `calculate_pet` for both PET calculations.
            *   "PET (car_0 vs ped_1 in area1): P1"
            *   "PET (car_0 vs ped_2 in area2): P2"
    *   A summary for the AV speed:
        *   "Probability of PETs >= 2.0s: PP.PP%"
        *   "Average J2 Traversal Time: TT.TTs (from V valid J2 traversals)"
4.  A separator line followed by "Optimization Search Complete."
5.  A list of all candidate AV speeds tested, with their probability of PET constraint satisfaction and average J2 traversal time.
6.  Total optimization processing time.
7.  The final "Optimal AV Speed Found" section, stating:
    *   The optimal AV speed in m/s.
    *   The minimized average J2 traversal time at this speed.
    *   The probability of PETs >= 2.0s at this optimal speed.
    *   Or, if no speed met the criteria, a message indicating this and providing the "best compromise" found.

## 8. Future Work

*   **Full Execution and Debugging**: Resolve any remaining file creation/syntax issues to allow for complete execution and verification of `optimizer.py`.
*   **Advanced Optimization**: Implement more sophisticated optimization algorithms (e.g., Bayesian Optimization, Genetic Algorithms) for potentially better efficiency and exploration of the search space.
*   **Higher Fidelity PET**: Incorporate more precise interpolation for entry/exit times in `pet_calculator.py` (e.g., geometric line-segment intersection with polygon edges).
*   **Dynamic Pedestrian Models**: Integrate more reactive pedestrian behavior models within SUMO, rather than fixed speeds per run.
*   **Expanded Bayesian Network**: Formally implement and utilize a Bayesian Network for decision-making under uncertainty, potentially incorporating more variables (e.g., AV's perception accuracy, different environmental conditions).
*   **Multi-Objective Optimization**: Consider objectives beyond J2 traversal time, such as passenger comfort or energy efficiency.
*   **Scalability**: Test and optimize the framework for larger scenarios, more complex road networks, and a higher number of interacting agents.
*   **Sensitivity Analysis**: Perform sensitivity analysis on key parameters (e.g., PET threshold, probability constraint, pedestrian speed distributions) to understand their impact on the optimal AV speed.
```
