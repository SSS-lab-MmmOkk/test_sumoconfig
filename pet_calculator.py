import math

try:
    from shapely.geometry import Point, Polygon
    SHAPELY_AVAILABLE = True
    # print("Shapely library found. Using it for polygon operations.") # Keep console clean for test output
except ImportError:
    SHAPELY_AVAILABLE = False
    print("Warning: Shapely library not found. Using manual point-in-polygon algorithm.")

# 1. Helper Geometric Functions
def distance_between_points(p1, p2):
    """Calculates the Euclidean distance between two points (x, y)."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def is_inside_polygon(point, polygon_vertices):
    """
    Checks if a point (x, y) is inside a polygon defined by its vertices.
    Uses Shapely if available, otherwise a manual ray casting algorithm.
    Point: tuple (x, y)
    Polygon_vertices: list of tuples [(x1, y1), (x2, y2), ...]
    """
    if not polygon_vertices: 
        return False
    if SHAPELY_AVAILABLE:
        if len(polygon_vertices) < 3: 
            return False 
        try:
            poly = Polygon(polygon_vertices)
            if not poly.is_valid:
                poly = poly.buffer(0)
                if not poly.is_valid: 
                    # print(f"Warning: Shapely polygon {polygon_vertices} is invalid and could not be fixed.")
                    return False 
            p = Point(point)
            return poly.contains(p)
        except Exception as e: 
            # print(f"Shapely error with polygon {polygon_vertices} and point {point}: {e}")
            return False 
    else:
        # Manual Ray Casting Algorithm
        if len(polygon_vertices) < 3:
            return False 
        x, y = point[0], point[1]
        n = len(polygon_vertices)
        inside = False
        
        p1x, p1y = polygon_vertices[0]
        for i in range(n + 1):
            p2x, p2y = polygon_vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters: 
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

def _get_entry_exit_times(trajectory, area_polygon):
    entry_time = None
    exit_time = None
    currently_inside = False

    if not trajectory or not area_polygon or len(area_polygon) < 3:
        return None, None

    def get_coords_time(point_data, index_for_error_msg=""):
        if not isinstance(point_data, dict): 
            raise TypeError(f"Trajectory point {index_for_error_msg} not a dict: {point_data}")
        if 'x' not in point_data or 'y' not in point_data or 'time' not in point_data:
            raise ValueError(f"Trajectory point {index_for_error_msg} missing 'x', 'y', or 'time'. Got: {point_data}")
        return (point_data['x'], point_data['y']), point_data['time']

    try:
        p_start_coords, t_start = get_coords_time(trajectory[0], "0 (start)")
        if is_inside_polygon(p_start_coords, area_polygon):
            entry_time = t_start
            currently_inside = True
    except (ValueError, TypeError) as e:
        # print(f"Error processing first trajectory point: {e}") 
        return None, None 
    
    for i in range(1, len(trajectory)):
        try:
            _, t_prev = get_coords_time(trajectory[i-1], str(i-1))
            p_curr_coords, t_curr = get_coords_time(trajectory[i], str(i))
        except (ValueError, TypeError) as e:
            # print(f"Error processing trajectory points: {e}") 
            return None, None 

        is_p_curr_inside = is_inside_polygon(p_curr_coords, area_polygon)

        if not currently_inside and is_p_curr_inside: 
            if entry_time is None: 
                 entry_time = (t_prev + t_curr) / 2.0
            currently_inside = True
        elif currently_inside and not is_p_curr_inside: 
            exit_time = (t_prev + t_curr) / 2.0
            currently_inside = False
            break 

    if currently_inside and entry_time is not None and exit_time is None:
        try:
            _, exit_time_val = get_coords_time(trajectory[-1], "last (for exit)")
            exit_time = exit_time_val
        except (ValueError, TypeError) as e:
            # print(f"Error processing last trajectory point for exit time: {e}") 
            return entry_time, None 
        
    if len(trajectory) == 1 and entry_time is not None: 
        exit_time = entry_time 

    if exit_time is not None and entry_time is None: 
        try:
            p_first_coords, t_first = get_coords_time(trajectory[0], "0 (implied entry)")
            if is_inside_polygon(p_first_coords, area_polygon):
                 entry_time = t_first
            else: 
                return None, None
        except (ValueError, TypeError) as e:
            # print(f"Error processing first point for implied entry: {e}") 
            return None, None

    if entry_time is not None and exit_time is not None and exit_time < entry_time:
        # print(f"Warning: Invalid time sequence. Exit time {exit_time} < Entry time {entry_time}.") 
        return None, None 

    return entry_time, exit_time


def calculate_pet(vehicle_trajectory, pedestrian_trajectory, encroachment_area_polygon):
    for traj_list in [vehicle_trajectory, pedestrian_trajectory]:
        for point in traj_list:
            point.setdefault('speed', 0) 
            point.setdefault('id', 'dummy')

    T_veh_entry, T_veh_exit = _get_entry_exit_times(vehicle_trajectory, encroachment_area_polygon)
    T_ped_entry, T_ped_exit = _get_entry_exit_times(pedestrian_trajectory, encroachment_area_polygon)

    # print(f"Vehicle: Entry={T_veh_entry}, Exit={T_veh_exit}") 
    # print(f"Pedestrian: Entry={T_ped_entry}, Exit={T_ped_exit}")

    if T_veh_entry is None or T_veh_exit is None or \
       T_ped_entry is None or T_ped_exit is None:
        return float('inf') 

    latest_entry = max(T_veh_entry, T_ped_entry)
    earliest_exit = min(T_veh_exit, T_ped_exit)

    if latest_entry < earliest_exit: 
        return latest_entry - earliest_exit 
    else:
        if T_veh_exit <= T_ped_entry:
            pet = T_ped_entry - T_veh_exit
        elif T_ped_exit <= T_veh_entry:
            pet = T_veh_entry - T_ped_exit
        else: 
             pet = 0.0 
    return pet

if __name__ == '__main__':
    encroachment_area = [(0, 0), (10, 0), (10, 10), (0, 10)]

    print("--- Test Case 1: Clear Separation (Vehicle first) ---")
    vehicle_traj1 = [
        {'time': 0, 'x': -5, 'y': 5}, {'time': 1, 'x': 5, 'y': 5}, {'time': 2, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj1 = [
        {'time': 3, 'x': 5, 'y': -5}, {'time': 4, 'x': 5, 'y': 5}, {'time': 5, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0.5, 1.5), Ped(3.5, 4.5) -> PET = 3.5 - 1.5 = 2.0
    pet1 = calculate_pet(vehicle_traj1, pedestrian_traj1, encroachment_area)
    print(f"Calculated PET 1: {pet1} (Expected: 2.0)")

    print("\n--- Test Case 2: Clear Separation (Pedestrian first) ---")
    # Expected: Ped(0.5, 1.5), Veh(3.5, 4.5) -> PET = 3.5 - 1.5 = 2.0
    pet2 = calculate_pet(pedestrian_traj1, vehicle_traj1, encroachment_area)
    print(f"Calculated PET 2: {pet2} (Expected: 2.0)")


    print("\n--- Test Case 3: Collision/Overlap ---")
    vehicle_traj3 = [
        {'time': 0, 'x': -5, 'y': 5}, {'time': 1, 'x': 5, 'y': 5}, {'time': 2, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj3 = [
        {'time': 0.5, 'x': 5, 'y': -5}, {'time': 1.5, 'x': 5, 'y': 5}, {'time': 2.5, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0.5, 1.5), Ped(1.0, 2.0) -> Overlap: latest_entry=1.0, earliest_exit=1.5. PET = 1.0 - 1.5 = -0.5
    pet3 = calculate_pet(vehicle_traj3, pedestrian_traj3, encroachment_area)
    print(f"Calculated PET 3: {pet3} (Expected: -0.5)")

    print("\n--- Test Case 4: Vehicle does not enter ---")
    vehicle_traj4 = [
        {'time': 0, 'x': -5, 'y': 20}, {'time': 1, 'x': 5, 'y': 20}, {'time': 2, 'x': 15, 'y': 20}
    ] 
    pedestrian_traj4 = pedestrian_traj1 
    # Expected: Veh(None,None), Ped(3.5,4.5) -> PET = inf
    pet4 = calculate_pet(vehicle_traj4, pedestrian_traj4, encroachment_area)
    print(f"Calculated PET 4: {pet4} (Expected: inf)")

    print("\n--- Test Case 5: Pedestrian does not enter ---")
    vehicle_traj5 = vehicle_traj1 
    pedestrian_traj5 = vehicle_traj4 
    # Expected: Veh(0.5,1.5), Ped(None,None) -> PET = inf
    pet5 = calculate_pet(vehicle_traj5, pedestrian_traj5, encroachment_area)
    print(f"Calculated PET 5: {pet5} (Expected: inf)")

    print("\n--- Test Case 6: Vehicle starts inside, exits; Pedestrian enters after vehicle exits ---")
    vehicle_traj6 = [
        {'time': 0, 'x': 5, 'y': 5}, {'time': 1, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj6 = [
        {'time': 1, 'x': 5, 'y': -5}, {'time': 2, 'x': 5, 'y': 5}, {'time': 3, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0, 0.5), Ped(1.5, 2.5) -> PET = 1.5 - 0.5 = 1.0
    pet6 = calculate_pet(vehicle_traj6, pedestrian_traj6, encroachment_area)
    print(f"Calculated PET 6: {pet6} (Expected: 1.0)")

    print("\n--- Test Case 7: Vehicle enters, ends inside; Pedestrian enters and exits ---")
    vehicle_traj7 = [
        {'time': 0, 'x': -5, 'y': 5}, {'time': 1, 'x': 5, 'y': 5}
    ] 
    pedestrian_traj7 = [
        {'time': 0, 'x': 5, 'y': -5}, {'time': 1, 'x': 5, 'y': 5}, {'time': 2, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0.5, 1), Ped(0.5, 1.5) -> Overlap: latest_entry=0.5, earliest_exit=1.0. PET = 0.5 - 1.0 = -0.5
    pet7 = calculate_pet(vehicle_traj7, pedestrian_traj7, encroachment_area)
    print(f"Calculated PET 7: {pet7} (Expected: -0.5)")

    print("\n--- Test Case 8: Both start inside, vehicle exits first ---")
    vehicle_traj8 = [
        {'time': 0, 'x': 1, 'y': 1}, {'time': 1, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj8 = [
        {'time': 0, 'x': 2, 'y': 2}, {'time': 2, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0, 0.5), Ped(0, 1.0) -> Overlap: latest_entry=0, earliest_exit=0.5. PET = 0 - 0.5 = -0.5
    pet8 = calculate_pet(vehicle_traj8, pedestrian_traj8, encroachment_area)
    print(f"Calculated PET 8: {pet8} (Expected: -0.5)")
    
    print("\n--- Test Case 9: Vehicle passes, Pedestrian starts inside and exits after vehicle ---")
    vehicle_traj9 = [
        {'time': 0, 'x': -5, 'y': 5}, {'time': 1, 'x': 5, 'y': 5}, {'time': 2, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj9 = [
        {'time': 0, 'x': 1, 'y': 1}, {'time': 2.5, 'x': 2, 'y': 2}, {'time': 3.5, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0.5, 1.5), Ped(0, 3.0) -> Overlap: latest_entry=0.5, earliest_exit=1.5. PET = 0.5 - 1.5 = -1.0
    pet9 = calculate_pet(vehicle_traj9, pedestrian_traj9, encroachment_area)
    print(f"Calculated PET 9: {pet9} (Expected: -1.0)")

    print("\n--- Test Case 10: Pedestrian passes, Vehicle starts inside and exits after pedestrian ---")
    # Expected: Ped(0.5, 1.5), Veh(0, 3.0) -> Overlap: latest_entry=0.5, earliest_exit=1.5. PET = 0.5 - 1.5 = -1.0
    pet10 = calculate_pet(pedestrian_traj9, vehicle_traj9, encroachment_area)
    print(f"Calculated PET 10: {pet10} (Expected: -1.0)")

    print("\n--- Test Case 11: Vehicle enters and exits, Pedestrian enters and exits, completely separate, Ped later ---")
    vehicle_traj11 = [
        {'time': 1, 'x': -5, 'y': 5}, {'time': 2, 'x': 5, 'y': 5}, {'time': 3, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj11 = [
        {'time': 4, 'x': 5, 'y': -5}, {'time': 5, 'x': 5, 'y': 5}, {'time': 6, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(1.5, 2.5), Ped(4.5, 5.5) -> PET = 4.5 - 2.5 = 2.0
    pet11 = calculate_pet(vehicle_traj11, pedestrian_traj11, encroachment_area)
    print(f"Calculated PET 11: {pet11} (Expected: 2.0)")

    print("\n--- Test Case 12: Vehicle starts inside, exits. Pedestrian starts inside, exits later. ---")
    vehicle_traj12 = [
        {'time': 0, 'x': 1, 'y': 1}, {'time': 1, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj12 = [
        {'time': 0, 'x': 2, 'y': 2}, {'time': 1.5, 'x': 3, 'y': 3}, {'time': 2.5, 'x': 5, 'y': 15}
    ] 
    # Expected: Veh(0, 0.5), Ped(0, 2.0) -> Overlap: latest_entry=0, earliest_exit=0.5. PET = 0 - 0.5 = -0.5
    pet12 = calculate_pet(vehicle_traj12, pedestrian_traj12, encroachment_area)
    print(f"Calculated PET 12: {pet12} (Expected: -0.5)")

    print("\n--- Test Case 13: One point trajectory - outside ---")
    vehicle_traj13 = [{'time': 0, 'x': 100, 'y': 100}] 
    pedestrian_traj13 = pedestrian_traj1 
    # Expected: Veh(None,None), Ped(3.5,4.5) -> PET = inf
    pet13 = calculate_pet(vehicle_traj13, pedestrian_traj13, encroachment_area)
    print(f"Calculated PET 13: {pet13} (Expected: inf)")

    print("\n--- Test Case 14: One point trajectory - inside ---")
    vehicle_traj14 = [{'time': 0, 'x': 5, 'y': 5}] 
    pedestrian_traj14 = pedestrian_traj1 
    # Expected: Veh(0,0), Ped(3.5,4.5) -> PET = 3.5 - 0 = 3.5
    pet14 = calculate_pet(vehicle_traj14, pedestrian_traj14, encroachment_area)
    print(f"Calculated PET 14: {pet14} (Expected: 3.5)")

    print("\n--- Test Case 15: Empty trajectory ---")
    vehicle_traj15 = [] 
    pedestrian_traj15 = pedestrian_traj1 
    # Expected: Veh(None,None), Ped(3.5,4.5) -> PET = inf
    pet15 = calculate_pet(vehicle_traj15, pedestrian_traj15, encroachment_area)
    print(f"Calculated PET 15: {pet15} (Expected: inf)")
    
    print("\n--- Test Case 16: Vehicle touches edge (boundary point) ---")
    vehicle_traj16 = [
        {'time': 0, 'x': -1, 'y': 0}, {'time': 1, 'x': 0, 'y': 0}, {'time': 2, 'x': 1, 'y': -1}
    ] 
    # Expected: Veh(None,None) because (0,0) is on boundary, not strictly inside by Shapely's .contains()
    # Ped(3.5,4.5) -> PET = inf
    pet16 = calculate_pet(vehicle_traj16, pedestrian_traj1, encroachment_area)
    print(f"Calculated PET 16: {pet16} (Expected: inf)")

    print("\n--- Test Case 17: Vehicle just crosses a corner ---")
    vehicle_traj17 = [
        {'time': 0, 'x': -1, 'y': -1}, {'time': 1, 'x': 0.1, 'y': 0.1}, {'time': 2, 'x': 1, 'y': -1}
    ] 
    pedestrian_traj17 = pedestrian_traj1 
    # Expected: Veh(0.5, 1.5), Ped(3.5, 4.5) -> PET = 3.5 - 1.5 = 2.0
    pet17 = calculate_pet(vehicle_traj17, pedestrian_traj17, encroachment_area)
    print(f"Calculated PET 17: {pet17} (Expected: 2.0)")
    
    print("\n--- Test Case 18: Exact same interval ---")
    vehicle_traj18 = [
        {'time': 0, 'x': -5, 'y': 5}, {'time': 1, 'x': 5, 'y': 5}, {'time': 2, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj18_mod = [
        {'time': 0, 'x': -5, 'y': 5}, 
        {'time': 1, 'x': 5, 'y': 5}, 
        {'time': 2, 'x': 15, 'y': 5}
    ]
    # Expected: Veh(0.5,1.5), Ped(0.5,1.5) -> Overlap: latest_entry=0.5, earliest_exit=1.5. PET = 0.5-1.5 = -1.0
    pet18 = calculate_pet(vehicle_traj18, pedestrian_traj18_mod, encroachment_area)
    print(f"Calculated PET 18: {pet18} (Expected: -1.0)")

    print("\n--- Test Case 19: Zero duration PET (touching intervals) ---")
    vehicle_traj19 = [ 
        {'time': 0, 'x': -5, 'y': 5}, {'time': 1, 'x': 5, 'y': 5}, {'time': 2, 'x': 15, 'y': 5}
    ] 
    pedestrian_traj19_mod = [ 
        {'time': 1.0, 'x': 5, 'y': -5}, 
        {'time': 2.0, 'x': 5, 'y': 5}, 
        {'time': 3.0, 'x': 5, 'y': 15}  
    ] 
    # Expected: Veh(0.5,1.5), Ped(1.5,2.5) -> No Overlap (latest_entry=1.5, earliest_exit=1.5 is not <). PET = 1.5 - 1.5 = 0.0
    pet19 = calculate_pet(vehicle_traj19, pedestrian_traj19_mod, encroachment_area)
    print(f"Calculated PET 19: {pet19} (Expected: 0.0)")

    if not SHAPELY_AVAILABLE: 
        print("\n--- Test Case (Manual PnP): Non-Rectangular Polygon ---")
        non_rect_area = [(0,0), (10,5), (5,10), (0,5)] 
        vehicle_traj_manual = [
            {'time': 0, 'x': -2, 'y': 2}, {'time': 1, 'x': 2, 'y': 4}, {'time': 2, 'x': 12, 'y': 6}
        ] 
        pedestrian_traj_manual = [
            {'time': 3, 'x': 2, 'y': 12}, {'time': 4, 'x': 2, 'y': 6}, {'time': 5, 'x': 2, 'y': -2}
        ] 
        pet_manual = calculate_pet(vehicle_traj_manual, pedestrian_traj_manual, non_rect_area)
        print(f"Calculated PET (manual, non-rect): {pet_manual} (Expected: 2.0)")
```
