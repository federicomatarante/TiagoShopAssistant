import numpy as np
import heapq
import yaml
import os
import sys
from PIL import Image
from scipy.interpolate import CubicSpline
import math
import argparse
# It's good practice to try-except matplotlib import if it's optional
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Matplotlib not found. Path visualization will be skipped.")

class AStarPathPlanner:
    def __init__(self, map_file):
        """
        Inizializza il path planner A*
        
        :param map_file: Percorso del file YAML con la mappa
        """
        self.map_file = os.path.abspath(map_file)
        with open(map_file, 'r') as f:
            self.map_data = yaml.safe_load(f)
        
        self.grid = self._load_occupancy_grid(self.map_data['image'])
        self.resolution = self.map_data.get('resolution', 0.05)
        self.height, self.width = self.grid.shape
        self.movements = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        self.distance_grid = self._compute_distance_grid()
    
    def _compute_distance_grid(self):
        from scipy.ndimage import distance_transform_edt
        free_space = (self.grid == 0).astype(np.float32)
        return distance_transform_edt(free_space)
    
    def _load_occupancy_grid(self, image_path):
        full_image_path = os.path.join(os.path.dirname(self.map_file), image_path)
        img = Image.open(full_image_path).convert('L')
        grid = np.array(img)
        
        occupied_thresh = self.map_data.get('occupied_thresh', 0.65)
        # free_thresh = self.map_data.get('free_thresh', 0.196) # Not used in this logic
        negate = self.map_data.get('negate', 0)
        
        if negate:
            grid = 255 - grid
        
        # In your PGM, 0 is occupied, 255 is free.
        # The map server interprets values: 0 for free, 100 for occupied.
        # For A*, we typically use 0 for free, 1 for occupied.
        # Your current logic: (grid > (occupied_thresh * 255))
        # If occupied_thresh is 0.65, this means pixels > 165.75 are obstacles (1).
        # This seems inverted if your PGM has 0=occupied, 255=free.
        # Standard map_server interpretation of PGM (negate=0):
        #   - pixels < free_thresh*255 are free (0)
        #   - pixels > occupied_thresh*255 are occupied (100 for map_server, 1 for A*)
        #   - pixels in between are unknown
        # Let's assume the PGM stores 0 for occupied, 255 for free.
        # We want grid cells to be 1 if occupied, 0 if free.
        # So, if a PGM pixel value is low (e.g., < 0.1*255 = 25.5), it's an obstacle.
        # If it's high (e.g., > 0.9*255 = 242), it's free.
        # Map server uses free_thresh and occupied_thresh.
        # A common way for PGM where 0=black/occupied, 255=white/free is:
        # occupied = (grid < (free_thresh * 255)) # pixels darker than free_thresh are occupied
        # This also depends on the 'negate' flag.
        # Given your current setup: `return (grid > (occupied_thresh * 255)).astype(np.uint8)`
        # If PGM 0=occupied, 255=free, and negate=0:
        # This means if pixel_value > 0.65*255 (=165.75), it's an obstacle (1). This is correct if lighter colors are obstacles.
        # If PGM 0=occupied means black=obstacle, then this is inverted.
        # Let's assume the map_generator output PGM (0=occupied, 255=free)
        # And the map.yaml has negate: 0.
        # Then map_server considers values close to 0 as occupied.
        # For A* (1 = occupied, 0 = free):
        # We need to identify pixels that are considered occupied.
        # Thresholds are usually between 0.0 and 1.0 for map_server.
        # occupied_thresh = 0.65  (values above this fraction of 255 are occupied)
        # free_thresh = 0.196 (values below this fraction of 255 are free)
        # If negate is 0:
        #   grid value < free_thresh * 255 means free.
        #   grid value > occupied_thresh * 255 means occupied.
        # So for our internal grid where 1 is occupied:
        return (grid < ( (1.0 - occupied_thresh) * 255) ).astype(np.uint8) # Assuming 0 is black/occupied
                                                                      # This will make dark areas 1.
                                                                      # If your map PGM is 0=occupied, 255=free, then
                                                                      # occupied pixels are those with low values.
                                                                      # Example: if pixel value is 10, 10 < (1-0.65)*255 = 0.35*255 = 89.25 -> True (1, occupied)
                                                                      # if pixel value is 250, 250 < 89.25 -> False (0, free)
                                                                      # This seems more standard for PGM where 0=occupied.

    def _heuristic(self, a, b, w):
        distance_to_goal = np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
        # Ensure a[1] (y) and a[0] (x) are within bounds of distance_grid
        y_idx, x_idx = int(round(a[1])), int(round(a[0]))
        if not (0 <= y_idx < self.distance_grid.shape[0] and 0 <= x_idx < self.distance_grid.shape[1]):
            # If outside, penalize heavily or handle as error, for now, high cost
            return distance_to_goal + w * 1e6 
        d_min = self.distance_grid[y_idx, x_idx]
        return distance_to_goal + w * (1 / (d_min + 1e-6))
        
    def _is_valid_cell(self, x, y):
        # Ensure x and y are integers for grid indexing
        x_int, y_int = int(round(x)), int(round(y))
        return (0 <= x_int < self.width and 
                0 <= y_int < self.height and 
                self.grid[y_int, x_int] == 0) # 0 means free
    
    def find_path(self, start, goal, safety_margin=1, w=100):
        start_grid = self._world_to_grid(start)
        goal_grid = self._world_to_grid(goal)

        if not self._is_valid_cell(start_grid[0], start_grid[1]):
            print(f"Start point {start} (grid: {start_grid}) is not valid (occupied or out of bounds).")
            return None
        if not self._is_valid_cell(goal_grid[0], goal_grid[1]):
            print(f"Goal point {goal} (grid: {goal_grid}) is not valid (occupied or out of bounds).")
            return None

        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {tuple(start_grid): 0}
        # Use tuple for f_score keys as well for consistency
        f_score = {tuple(start_grid): self._heuristic(start_grid, goal_grid, w)}
        
        while open_set:
            _, current = heapq.heappop(open_set) # current_f not needed if not used
            current_tuple = tuple(current)

            if current_tuple == tuple(goal_grid):
                return self._reconstruct_path(came_from, current)
            
            for dx, dy in self.movements:
                neighbor = (current[0] + dx, current[1] + dy)
                neighbor_tuple = tuple(neighbor)
                
                if not self._is_valid_cell(neighbor[0], neighbor[1]):
                    continue
                
                # Cost for straight move is 1, diagonal can be sqrt(2) or 1 for simplicity
                # If using 1 for all, ensure heuristic is consistent (admissible)
                move_cost = np.sqrt(dx**2 + dy**2) # More accurate cost
                tentative_g_score = g_score[current_tuple] + move_cost
                
                if tentative_g_score < g_score.get(neighbor_tuple, float('inf')):
                    came_from[neighbor_tuple] = current
                    g_score[neighbor_tuple] = tentative_g_score
                    f_score[neighbor_tuple] = tentative_g_score + self._heuristic(neighbor, goal_grid, w)
                    heapq.heappush(open_set, (f_score[neighbor_tuple], list(neighbor)))
        
        return None
    
    def _reconstruct_path(self, came_from, current):
        path = [list(current)] # Store as list
        current_tuple = tuple(current)
        while current_tuple in came_from:
            current = came_from[current_tuple]
            path.append(list(current))
            current_tuple = tuple(current)
        return list(reversed([self._grid_to_world(p) for p in path]))
    
    def _world_to_grid(self, world_coords):
        origin = self.map_data.get('origin', [0, 0, 0])
        x = int(round((world_coords[0] - origin[0]) / self.resolution)) # Use round for robustness
        y = int(round((world_coords[1] - origin[1]) / self.resolution)) # Use round
        return [x, y]
    
    def _grid_to_world(self, grid_coords):
        origin = self.map_data.get('origin', [0, 0, 0])
        # Add 0.5*resolution to get cell center, helps avoid being exactly on edge
        x = grid_coords[0] * self.resolution + origin[0] + (self.resolution / 2.0)
        y = grid_coords[1] * self.resolution + origin[1] + (self.resolution / 2.0)
        return [x, y]
    
    def visualize_path(self, path):
        if not MATPLOTLIB_AVAILABLE:
            print("Skipping visualization because matplotlib is not available.")
            return
        if not path:
            print("No path to visualize.")
            return

        plt.figure(figsize=(10, 10))
        # self.grid: 0 is free, 1 is occupied. imshow needs 0 for black, 1 for white (or vice-versa with cmap)
        # If 0 is free, (1-self.grid) will make free cells 1 (white) and occupied cells 0 (black)
        plt.imshow(1 - self.grid, cmap='gray', origin='lower') # origin='lower' to match map server usually
        
        if path: # Ensure path is not None
            path_array = np.array([self._world_to_grid(p) for p in path])
            plt.plot(path_array[:, 0], path_array[:, 1], 'r-', linewidth=2)
            plt.plot(path_array[0, 0], path_array[0, 1], 'go') # Start point
            plt.plot(path_array[-1, 0], path_array[-1, 1], 'bo') # Goal point
        
        plt.title('Percorso A*')
        plt.xlabel('Grid X')
        plt.ylabel('Grid Y')
        plt.show()

    def smooth_path(self, path, num_points=50, collision_threshold=0.1): # Increased default num_points
        if not path or len(path) < 3: # Check if path is None or too short
            return path
        
        path_array = np.array(path)
        x = path_array[:, 0]
        y = path_array[:, 1]
        
        dist = np.cumsum(np.sqrt(np.ediff1d(x, to_begin=0)**2 + np.ediff1d(y, to_begin=0)**2))
        if dist[-1] == 0: # Path has no length (e.g., start and goal are same)
            return path
        dist /= dist[-1]
        
        cs_x = CubicSpline(dist, x)
        cs_y = CubicSpline(dist, y)
        
        new_dist = np.linspace(0, 1, num_points)
        smoothed_x = cs_x(new_dist)
        smoothed_y = cs_y(new_dist)
        
        optimized_path = []
        for px, py in zip(smoothed_x, smoothed_y):
            grid_x, grid_y = self._world_to_grid([px, py])
            
            # Ensure grid_x, grid_y are within bounds for distance_grid
            if not (0 <= grid_y < self.distance_grid.shape[0] and 0 <= grid_x < self.distance_grid.shape[1]):
                print(f"Warning: Smoothed point ({px},{py}) -> grid ({grid_x},{grid_y}) is out of distance_grid bounds. Skipping adjustment.")
                optimized_path.append([px, py])
                continue

            current_distance_pixels = self.distance_grid[grid_y, grid_x]
            current_distance_meters = current_distance_pixels * self.resolution
            
            if current_distance_meters < collision_threshold:
                gradient_x, gradient_y = self._compute_gradient(grid_x, grid_y)
                # Push away by the difference needed
                push_factor = (collision_threshold - current_distance_meters)
                px += gradient_x * push_factor 
                py += gradient_y * push_factor
            
            optimized_path.append([px, py])
        
        return optimized_path
    
    def generate_circular_motion(self, center, radius=1, num_points=10, start_angle=0, direction='ccw', arc_degrees=90):
        # output_file argument removed as .nav is not needed here
        delta_theta_rad = math.radians(arc_degrees) / (num_points -1 if num_points > 1 else 1)
        angles = []
        for i in range(num_points):
            if direction == 'cw':
                angles.append(start_angle - i * delta_theta_rad)
            else:
                angles.append(start_angle + i * delta_theta_rad)

        waypoints = []
        for theta in angles:
            x = center[0] + radius * math.cos(theta)
            y = center[1] + radius * math.sin(theta)
            waypoints.append([x, y])
        return waypoints
    
    def _compute_gradient(self, x, y):
        # Ensure x, y are within bounds for gradient calculation
        x_int, y_int = int(round(x)), int(round(y))

        # Check bounds to prevent accessing outside the distance_grid
        if not (0 < y_int < self.distance_grid.shape[0]-1 and \
                0 < x_int < self.distance_grid.shape[1]-1) :
            # At the border, can't compute centered gradient, return no change
            return 0.0, 0.0

        # Central difference for gradient
        dx = self.distance_grid[y_int, x_int+1] - self.distance_grid[y_int, x_int-1]
        dy = self.distance_grid[y_int+1, x_int] - self.distance_grid[y_int-1, x_int]
        norm = np.sqrt(dx**2 + dy**2) + 1e-6
        return dx/norm, dy/norm

# generate_nav_file function can be kept for other uses if needed, but not called from main
def generate_nav_file(path, output_file):
    if not path: return
    with open(output_file, 'w') as f:
        for wp in path[1:]:
            f.write(f"moveTo({wp[0]}, {wp[1]})\n")

def main():
    parser = argparse.ArgumentParser(description='Path Planning with A* or Circular Motion')
    parser.add_argument('map_file', help='Path to map YAML file')
    parser.add_argument('--motion_type', choices=['full', 'circular'], required=True)
    
    parser.add_argument('--start_coords', nargs=2, type=float, metavar=('X', 'Y'), help="Start coordinates (X Y) for 'full' motion")
    parser.add_argument('--goal_coords', nargs=2, type=float, metavar=('X', 'Y'), help="Goal coordinates (X Y) for 'full' motion")
    parser.add_argument('--center', nargs=2, type=float, metavar=('CX', 'CY'), help="Center coordinates (CX CY) for 'circular' motion")
    parser.add_argument('--radius', type=float, help="Radius for 'circular' motion")
    
    # Parameters for smoothing / circular motion points
    parser.add_argument('--num_points', type=int, default=15, help="Number of points for smoothed path or circular motion. Try values like 50, 75, 100.")
    parser.add_argument('--collision_threshold', type=float, default=0.15, help="Collision threshold for path smoothing (meters). Try values like 0.05, 0.1, 0.15.")
    parser.add_argument('--no_smooth', action='store_true', help="Disable path smoothing for 'full' motion, show raw A* path.")
    parser.add_argument('--w_heuristic', type=float, default=100.0, help="Weight 'w' for the obstacle avoidance term in the A* heuristic.")


    args = parser.parse_args()

    try:
        print(f"Loading map: {args.map_file}...")
        planner = AStarPathPlanner(args.map_file)
        print("Map loaded successfully.")
    except FileNotFoundError as e:
        print(f"Error: Map file not found. {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading map or initializing planner: {e}")
        # print full traceback for debugging
        import traceback
        traceback.print_exc()
        sys.exit(1)


    if args.motion_type == 'full':
        if not args.start_coords or not args.goal_coords:
            parser.error("--start_coords and --goal_coords are required for 'full' motion type.")
        
        print(f"Finding path from {args.start_coords} to {args.goal_coords} with heuristic weight w={args.w_heuristic}...")
        raw_path = planner.find_path(args.start_coords, args.goal_coords, w=args.w_heuristic)
        
        final_path = None
        path_type_message = ""

        if raw_path:
            print(f"Raw A* path found with {len(raw_path)} waypoints.")
            if args.no_smooth:
                print("Smoothing disabled. Using raw A* path.")
                final_path = raw_path
                path_type_message = "Raw A* Path"
            else:
                print(f"Smoothing path with num_points={args.num_points}, collision_threshold={args.collision_threshold}m...")
                smoothed_path = planner.smooth_path(raw_path, num_points=args.num_points, collision_threshold=args.collision_threshold)
                if smoothed_path and len(smoothed_path) > 0 :
                    final_path = smoothed_path
                    path_type_message = "Smoothed Path"
                    print(f"Smoothed path generated with {len(final_path)} waypoints.")
                else:
                    print("Path smoothing failed or resulted in an empty path. Using raw A* path instead.")
                    final_path = raw_path
                    path_type_message = "Raw A* Path (smoothing failed)"
            
            if final_path:
                print(f"\nFinal Path ({path_type_message}) Coordinates:")
                for i, p in enumerate(final_path):
                    print(f"  {i}: ({p[0]:.3f}, {p[1]:.3f})") # More precision for coordinates
                
                if MATPLOTLIB_AVAILABLE:
                    print(f"\nVisualizing final path ({path_type_message})... Close plot window to continue.")
                    planner.visualize_path(final_path)
                else:
                    print("\nMatplotlib not available. Skipping visualization.")
            # Removed .nav file generation
        else:
            print("No path found.")
    
    elif args.motion_type == 'circular':
        if not args.center or args.radius is None:
            parser.error("--center and --radius are required for 'circular' motion type.")
        
        print(f"Generating circular motion: center={args.center}, radius={args.radius}, num_points={args.num_points}")
        waypoints = planner.generate_circular_motion(
            center=args.center,
            radius=args.radius,
            num_points=args.num_points
        )
        if waypoints:
            print(f"Generated circular motion with {len(waypoints)} waypoints:")
            for i, p in enumerate(waypoints): print(f"  {i}: ({p[0]:.3f}, {p[1]:.3f})")
            if MATPLOTLIB_AVAILABLE:
                print("\nVisualizing circular motion path... Close plot window to continue.")
                planner.visualize_path(waypoints)
            # Removed .nav file generation
        else:
            print("Failed to generate circular motion waypoints.")

if __name__ == '__main__':
    main()