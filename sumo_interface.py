import os
import sys
import subprocess
import traci
import traci.constants as tc
import time
import random # For --random seed if needed, though SUMO handles it internally

# Try to import PET calculator functions
try:
    from pet_calculator import calculate_pet
    PET_CALCULATOR_AVAILABLE = True
    # print("Successfully imported from pet_calculator.py") # Silenced for optimizer runs
except ImportError as e:
    print(f"Error importing from pet_calculator.py: {e}. PET calculation will not be available.")
    PET_CALCULATOR_AVAILABLE = False
    def calculate_pet(*args, **kwargs): # Dummy function
        # print("calculate_pet is not available due to import error.") # Silenced
        return float('nan')

# Default Encroachment Area Definitions (can be overridden by optimizer.py)
ENCROACHMENT_AREA_PED1_DEFAULT = [(-7.62, 27.46), (-5.39, 27.46), (-5.39, 29.46), (-7.62, 29.46)]
ENCROACHMENT_AREA_PED2_DEFAULT = [(-7.62, 19.06), (-5.39, 19.06), (-5.39, 21.06), (-7.62, 21.06)]

# SUMO_HOME and sys.path setup
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    if os.path.isdir(tools) and tools not in sys.path:
        sys.path.append(tools)
else:
    try:
        import traci # Check if traci is globally available
    except ImportError:
        sys.exit("Please declare environment variable 'SUMO_HOME' or ensure 'traci' is in your PYTHONPATH.")

# J2 segment details
J2_INTERNAL_LANE_ID = ":J2_4_0" 

def _original_run_simulation():
    """
    Original simulation run logic, kept for reference or direct testing.
    Uses default encroachment areas.
    """
    car0_traj = []
    ped1_traj = []
    ped2_traj = []
    # Original sumo_cmd without --random, for consistent test runs if needed
    sumo_cmd_orig = ["sumo", "-c", "config/ped250406.sumocfg", "--step-length", "0.1", "--default.action-step-length", "0.1"]

    print(f"Starting ORIGINAL SUMO simulation with command: {' '.join(sumo_cmd_orig)}")
    try:
        traci.start(sumo_cmd_orig)
        print("Successfully connected to SUMO (Original Run).")
        num_simulation_steps = 100 
        for step_count in range(num_simulation_steps):
            current_sim_time = traci.simulation.getTime()
            traci.simulationStep()
            try:
                car_pos = traci.vehicle.getPosition("car_0")
                car0_traj.append({'time': current_sim_time, 'x': car_pos[0], 'y': car_pos[1], 'id': "car_0"})
            except traci.TraCIException: pass
            try:
                ped1_pos = traci.person.getPosition("ped_1")
                ped1_traj.append({'time': current_sim_time, 'x': ped1_pos[0], 'y': ped1_pos[1], 'id': "ped_1"})
            except traci.TraCIException: pass
            try:
                ped2_pos = traci.person.getPosition("ped_2")
                ped2_traj.append({'time': current_sim_time, 'x': ped2_pos[0], 'y': ped2_pos[1], 'id': "ped_2"})
            except traci.TraCIException: pass
        
        print(f"\nOriginal simulation finished after {num_simulation_steps} steps ({traci.simulation.getTime():.2f}s sim time).")
        print(f"Collected {len(car0_traj)} points for car_0.")
        print(f"Collected {len(ped1_traj)} points for ped_1.")
        print(f"Collected {len(ped2_traj)} points for ped_2.")

        if PET_CALCULATOR_AVAILABLE:
            print("\n--- PET Calculation Results (Original Run) ---")
            if car0_traj and ped1_traj:
                # calculate_pet now prints internal entry/exit times
                pet_car0_ped1 = calculate_pet(car0_traj, ped1_traj, ENCROACHMENT_AREA_PED1_DEFAULT)
                print(f"PET (car_0 vs ped_1 in area_ped1_default): {pet_car0_ped1:.2f} s")
            if car0_traj and ped2_traj:
                pet_car0_ped2 = calculate_pet(car0_traj, ped2_traj, ENCROACHMENT_AREA_PED2_DEFAULT)
                print(f"PET (car_0 vs ped_2 in area_ped2_default): {pet_car0_ped2:.2f} s")
    except Exception as e:
        print(f"Error in _original_run_simulation: {e}")
    finally:
        if traci.isLoaded() and traci.getConnection():
            traci.close()
            print("TraCI connection closed (Original Run).")

def execute_single_sumo_run(av_target_m_s, ped1_target_m_s, ped2_target_m_s, 
                            sumo_cfg_path, encroachment_area1_coords, 
                            encroachment_area2_coords, debug_prints=True):
    """
    Executes a single SUMO simulation run with parameterized speeds and collects data.
    """
    sumo_command = ["sumo", "-c", sumo_cfg_path, 
                    "--step-length", "0.1", 
                    "--default.action-step-length", "0.1", # Ensure actions are processed at each step
                    "--random"] # Enable random seed for variability

    car0_traj = []
    ped1_traj = []
    ped2_traj = []

    sim_time_car0_enters_j2 = -1.0
    sim_time_car0_exits_j2 = -1.0
    car0_on_j2_segment = False
    
    car0_speed_set_attempted = False # Flag to try setting speed only once after vehicle appears
    ped1_speed_set_attempted = False
    ped2_speed_set_attempted = False
    
    # Max number of steps to wait for entities to appear for speed setting
    MAX_ENTITY_WAIT_STEPS = 50 # 5 seconds

    try:
        traci.start(sumo_command)
        if debug_prints: print(f"SUMO started. CMD: {' '.join(sumo_command)}")

        num_simulation_steps = 200 # 20 seconds simulation time

        for step in range(num_simulation_steps):
            current_sim_time = traci.simulation.getTime()
            traci.simulationStep()

            # AV Speed Control for car_0
            if not car0_speed_set_attempted and step < MAX_ENTITY_WAIT_STEPS :
                try:
                    # Check if car_0 exists before trying to set its speed
                    if "car_0" in traci.vehicle.getIDList():
                        traci.vehicle.setSpeedMode("car_0", 0) # Disable all safety checks for precise speed
                        traci.vehicle.setSpeed("car_0", av_target_m_s)
                        car0_speed_set_attempted = True # Mark as attempted (successfully or not if it departs immediately)
                        if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: car_0 speed set to {av_target_m_s} m/s.")
                except traci.TraCIException as e_av:
                    if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: Minor TraCI error setting car_0 speed (may not exist yet): {e_av}")
            
            # Pedestrian Speed Control for ped_1
            if not ped1_speed_set_attempted and step < MAX_ENTITY_WAIT_STEPS:
                try:
                    if "ped_1" in traci.person.getIDList():
                        traci.person.setSpeed("ped_1", ped1_target_m_s)
                        ped1_speed_set_attempted = True
                        if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: ped_1 speed set to {ped1_target_m_s} m/s.")
                except traci.TraCIException as e_p1:
                    if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: Minor TraCI error setting ped_1 speed: {e_p1}")

            # Pedestrian Speed Control for ped_2
            if not ped2_speed_set_attempted and step < MAX_ENTITY_WAIT_STEPS:
                try:
                    if "ped_2" in traci.person.getIDList():
                        traci.person.setSpeed("ped_2", ped2_target_m_s)
                        ped2_speed_set_attempted = True
                        if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: ped_2 speed set to {ped2_target_m_s} m/s.")
                except traci.TraCIException as e_p2:
                    if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: Minor TraCI error setting ped_2 speed: {e_p2}")

            # Trajectory Collection and J2 Traversal for car_0
            try:
                if "car_0" in traci.vehicle.getIDList(): # Check if car_0 exists
                    car_pos = traci.vehicle.getPosition("car_0")
                    car0_traj.append({'time': current_sim_time, 'x': car_pos[0], 'y': car_pos[1], 'id': "car_0"})
                    
                    current_lane = traci.vehicle.getLaneID("car_0")
                    if current_lane == J2_INTERNAL_LANE_ID:
                        if not car0_on_j2_segment: 
                            sim_time_car0_enters_j2 = current_sim_time
                            car0_on_j2_segment = True
                            if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: car_0 entered J2 segment ({J2_INTERNAL_LANE_ID}).")
                    elif car0_on_j2_segment: # Was on J2, but now on a different lane (and exit time not yet recorded)
                        if sim_time_car0_exits_j2 == -1.0: # Record only the first exit
                            sim_time_car0_exits_j2 = current_sim_time 
                            if debug_prints: print(f"Step {step} @ {current_sim_time:.1f}s: car_0 exited J2 segment. End time: {sim_time_car0_exits_j2:.1f}s")
                            # We don't set car0_on_j2_segment = False here, to correctly handle if it re-enters J2_INTERNAL_LANE_ID
                            # The condition sim_time_car0_exits_j2 == -1.0 ensures we only capture the first exit time.
            except traci.TraCIException: pass 

            try:
                if "ped_1" in traci.person.getIDList():
                    ped1_pos = traci.person.getPosition("ped_1")
                    ped1_traj.append({'time': current_sim_time, 'x': ped1_pos[0], 'y': ped1_pos[1], 'id': "ped_1"})
            except traci.TraCIException: pass

            try:
                if "ped_2" in traci.person.getIDList():
                    ped2_pos = traci.person.getPosition("ped_2")
                    ped2_traj.append({'time': current_sim_time, 'x': ped2_pos[0], 'y': ped2_pos[1], 'id': "ped_2"})
            except traci.TraCIException: pass

        # Finalize J2 traversal time
        av_j2_traversal_time = -1.0 
        if sim_time_car0_enters_j2 != -1.0:
            if sim_time_car0_exits_j2 != -1.0:
                av_j2_traversal_time = sim_time_car0_exits_j2 - sim_time_car0_enters_j2
            elif car0_on_j2_segment: # Car ended simulation while still on J2 segment
                if debug_prints: print(f"Warning: car_0 was still on J2 segment ({J2_INTERNAL_LANE_ID}) at end of simulation. Traversal time incomplete.")
                # J2 traversal time remains -1.0
        elif debug_prints:
             print(f"Warning: car_0 never entered J2 segment ({J2_INTERNAL_LANE_ID}).")


        if debug_prints:
            print(f"\nSimulation run finished. AV J2 traversal time: {av_j2_traversal_time:.2f}s")
            print(f"Collected {len(car0_traj)} points for car_0.")
            print(f"Collected {len(ped1_traj)} points for ped_1.")
            print(f"Collected {len(ped2_traj)} points for ped_2.")

        # PET Calculation
        pet1, pet2 = float('inf'), float('inf') 
        if PET_CALCULATOR_AVAILABLE:
            if car0_traj and ped1_traj:
                pet1 = calculate_pet(car0_traj, ped1_traj, encroachment_area1_coords)
                if debug_prints: print(f"PET (car_0 vs ped_1 in area1): {pet1}")
            elif debug_prints: print("Skipping PET for car_0/ped_1: empty/missing trajectory.")
            
            if car0_traj and ped2_traj:
                pet2 = calculate_pet(car0_traj, ped2_traj, encroachment_area2_coords)
                if debug_prints: print(f"PET (car_0 vs ped_2 in area2): {pet2}")
            elif debug_prints: print("Skipping PET for car_0/ped_2: empty/missing trajectory.")
        elif debug_prints: print("PET calculator not available for this run.")

        return pet1, pet2, av_j2_traversal_time, car0_traj, ped1_traj, ped2_traj

    except traci.exceptions.TraCIException as e: 
        if debug_prints: print(f"A TraCI error occurred during simulation: {e}")
        return float('inf'), float('inf'), -1.0, car0_traj, ped1_traj, ped2_traj # Return collected data
    except FileNotFoundError as e:
        print(f"Error starting SUMO (FileNotFound): {e}. Check SUMO path and config.")
        # This is a fatal error for this run, can't return trajectory data.
        return float('inf'), float('inf'), -1.0, [], [], [] 
    except Exception as e:
        if debug_prints: print(f"An unexpected error in execute_single_sumo_run: {e}")
        return float('inf'), float('inf'), -1.0, car0_traj, ped1_traj, ped2_traj # Return collected data
    finally:
        if traci.isLoaded() and traci.getConnection():
            traci.close()
            # if debug_prints: print("TraCI connection closed.")

# if __name__ == "__main__":
#    # Example call to the new function (for testing purposes)
#    # print("Executing a test run of execute_single_sumo_run...")
#    # pet1, pet2, traversal_time, _, _, _ = execute_single_sumo_run(
#    #     av_target_m_s=10.0, 
#    #     ped1_target_m_s=1.0, 
#    #     ped2_target_m_s=1.2,
#    #     sumo_cfg_path="config/ped250406.sumocfg",
#    #     encroachment_area1_coords=ENCROACHMENT_AREA_PED1_DEFAULT,
#    #     encroachment_area2_coords=ENCROACHMENT_AREA_PED2_DEFAULT,
#    #     debug_prints=True
#    # )
#    # print(f"\nTest Run Results: PET1={pet1:.2f}, PET2={pet2:.2f}, J2 Traversal={traversal_time:.2f}s")
#    
#    # _original_run_simulation() # Call the old main if needed for specific testing
```
