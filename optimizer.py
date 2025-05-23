import numpy as np
import random
import time # For timing the optimization process

try:
    # Assuming sumo_interface.py is in the same directory or PYTHONPATH
    from sumo_interface import execute_single_sumo_run, ENCROACHMENT_AREA_PED1_DEFAULT, ENCROACHMENT_AREA_PED2_DEFAULT
    SUMO_INTERFACE_AVAILABLE = True
except ImportError as e:
    print(f"Error importing from sumo_interface.py: {e}. Optimizer cannot run.")
    SUMO_INTERFACE_AVAILABLE = False

# Configuration Constants
PED_MEAN_SPEED_MPS = 1.34  # Average pedestrian speed (m/s)
PED_STD_DEV_MPS = 0.26   # Standard deviation for pedestrian speed (m/s)
MIN_PED_SPEED_MPS = 0.2  # Minimum realistic pedestrian speed

# --- Parameters for Worker Check (Reduced) ---
AV_MIN_SPEED_MPS = 5.0    # Min AV speed to test (m/s)
AV_MAX_SPEED_MPS = 15.0   # Max AV speed to test (m/s)
NUM_AV_SPEED_STEPS = 3    # Number of AV speeds to evaluate in the grid
N_SIM_RUNS_PER_AV_SPEED = 5 # Number of simulation runs for each AV speed
# --- End Parameters for Worker Check ---

# Original values (for reference, commented out)
# AV_MIN_SPEED_MPS = 5.0
# AV_MAX_SPEED_MPS = 20.0
# NUM_AV_SPEED_STEPS = 10 
# N_SIM_RUNS_PER_AV_SPEED = 30 

PET_CONSTRAINT_THRESHOLD = 2.0  # seconds
PROB_CONSTRAINT_SATISFIED_THRESHOLD = 0.90 # Target probability (90%)

SUMO_CFG_PATH = "config/ped250406.sumocfg" # Path to SUMO configuration file

# Use imported default encroachment areas from sumo_interface.py
ENCROACHMENT_AREA_PED1 = ENCROACHMENT_AREA_PED1_DEFAULT
ENCROACHMENT_AREA_PED2 = ENCROACHMENT_AREA_PED2_DEFAULT


def run_optimizer():
    if not SUMO_INTERFACE_AVAILABLE:
        print("Optimizer cannot run because sumo_interface.py could not be imported.")
        return

    print("Starting PET Optimizer Grid Search...")
    print(f"AV Speed Range: {AV_MIN_SPEED_MPS:.1f} m/s to {AV_MAX_SPEED_MPS:.1f} m/s")
    print(f"Number of AV Speed Steps: {NUM_AV_SPEED_STEPS}")
    print(f"Simulations per AV Speed: {N_SIM_RUNS_PER_AV_SPEED}")
    print(f"PET Constraint Threshold: >= {PET_CONSTRAINT_THRESHOLD:.1f}s")
    print(f"Target Probability for Constraint Satisfaction: >= {PROB_CONSTRAINT_SATISFIED_THRESHOLD*100:.0f}%")
    print(f"SUMO Config: {SUMO_CFG_PATH}")
    print("-" * 60)

    candidate_av_speeds = np.linspace(AV_MIN_SPEED_MPS, AV_MAX_SPEED_MPS, NUM_AV_SPEED_STEPS)
    # List to store tuples of (av_speed, prob_constraint_satisfied, avg_traversal_time)
    optimization_summary = [] 

    total_sims_to_run = len(candidate_av_speeds) * N_SIM_RUNS_PER_AV_SPEED
    current_sim_run_number = 0
    optimization_start_time = time.time()

    for av_speed in candidate_av_speeds:
        print(f"\nTesting AV Speed: {av_speed:.2f} m/s")
        
        run_pet1s = []
        run_pet2s = []
        run_traversal_times = []
        successful_pet_constraint_runs = 0

        for i_run in range(N_SIM_RUNS_PER_AV_SPEED):
            current_sim_run_number += 1
            print(f"  Run {i_run + 1}/{N_SIM_RUNS_PER_AV_SPEED} for AV speed {av_speed:.2f} m/s... (Overall: {current_sim_run_number}/{total_sims_to_run})")

            # Sample pedestrian speeds for this run
            ped1_current_speed = max(MIN_PED_SPEED_MPS, random.normalvariate(PED_MEAN_SPEED_MPS, PED_STD_DEV_MPS))
            ped2_current_speed = max(MIN_PED_SPEED_MPS, random.normalvariate(PED_MEAN_SPEED_MPS, PED_STD_DEV_MPS))
            
            # Execute the SUMO simulation
            # debug_prints set to False for optimizer to reduce console output
            pet1, pet2, traversal_time, _, _, _ = execute_single_sumo_run(
                av_target_m_s=av_speed,
                ped1_target_m_s=ped1_current_speed,
                ped2_target_m_s=ped2_current_speed,
                sumo_cfg_path=SUMO_CFG_PATH,
                encroachment_area1_coords=ENCROACHMENT_AREA_PED1,
                encroachment_area2_coords=ENCROACHMENT_AREA_PED2,
                debug_prints=False 
            )

            run_pet1s.append(pet1)
            run_pet2s.append(pet2)
            if traversal_time >= 0: # Only consider valid traversal times (not -1.0)
                run_traversal_times.append(traversal_time)
            
            # Check if PET constraints are met for this specific run
            # float('inf') >= threshold is True, which is correct (no conflict means constraint met)
            if pet1 >= PET_CONSTRAINT_THRESHOLD and pet2 >= PET_CONSTRAINT_THRESHOLD:
                successful_pet_constraint_runs += 1
        
        # Calculate metrics for the current AV speed
        prob_constraint_satisfied = successful_pet_constraint_runs / N_SIM_RUNS_PER_AV_SPEED
        
        avg_traversal_time = -1.0 # Default if no valid J2 traversals occurred
        if run_traversal_times:
            avg_traversal_time = np.mean(run_traversal_times)
        
        optimization_summary.append((av_speed, prob_constraint_satisfied, avg_traversal_time))
        
        print(f"  Results for AV Speed {av_speed:.2f} m/s:")
        print(f"    Probability of PETs >= {PET_CONSTRAINT_THRESHOLD:.1f}s: {prob_constraint_satisfied:.2%}")
        print(f"    Average J2 Traversal Time: {avg_traversal_time:.2f}s (from {len(run_traversal_times)} valid J2 traversals)")

    print("-" * 60)
    print("\nOptimization Search Complete.")
    
    # Select Optimal Speed based on criteria
    best_av_speed = -1.0
    best_traversal_time = float('inf')
    best_prob_satisfaction = -1.0 # To track the probability at the chosen optimal speed

    eligible_speeds = [] # Speeds that meet the PET probability constraint

    for av_s, prob_cs, avg_tt in optimization_summary:
        print(f"  Candidate: AV Speed={av_s:.2f} m/s, Prob PET Constraint Met={prob_cs:.2%}, Avg J2 Traversal Time={avg_tt:.2f}s")
        if prob_cs >= PROB_CONSTRAINT_SATISFIED_THRESHOLD:
            eligible_speeds.append((av_s, avg_tt, prob_cs)) # Store prob_cs for final report

    if eligible_speeds:
        # Sort eligible speeds: primary key avg_tt (ascending), secondary key av_s (descending, if times are equal pick higher speed)
        eligible_speeds.sort(key=lambda x: (x[1] if x[1] != -1.0 else float('inf'), -x[0])) 
        
        best_av_speed = eligible_speeds[0][0]
        best_traversal_time = eligible_speeds[0][1]
        best_prob_satisfaction = eligible_speeds[0][2]

    optimization_duration_s = time.time() - optimization_start_time
    print(f"\nTotal optimization processing time: {optimization_duration_s:.2f} seconds.")

    if best_av_speed != -1.0 :
        print(f"\n--- Optimal AV Speed Found ---")
        print(f"Optimal AV Speed: {best_av_speed:.2f} m/s")
        print(f"Minimized Average J2 Traversal Time: {best_traversal_time:.2f}s")
        print(f"Probability of PETs >= {PET_CONSTRAINT_THRESHOLD:.1f}s at this speed: {best_prob_satisfaction:.2%}")
    else:
        print("\n--- No AV Speed Found ---")
        print(f"No AV speed tested met the PET constraint probability of {PROB_CONSTRAINT_SATISFIED_THRESHOLD*100:.0f}%.")
        if optimization_summary: # Find best compromise if no speed meets criteria
            # Maximize probability first, then minimize traversal time
            best_compromise = max(optimization_summary, key=lambda x: (x[1], -x[2] if x[2] != -1.0 else float('inf')))
            print(f"  Best compromise found: AV Speed={best_compromise[0]:.2f} m/s, "
                  f"Prob PET Constraint Met={best_compromise[1]:.2%}, "
                  f"Avg J2 Traversal Time={best_compromise[2]:.2f}s")

if __name__ == "__main__":
    if SUMO_INTERFACE_AVAILABLE:
        run_optimizer()
    else:
        print("Optimizer cannot start due to issues with sumo_interface.py import.")

```
