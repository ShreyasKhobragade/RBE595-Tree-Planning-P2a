import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Environment3D:
    def __init__(self):
        self.boundary = []
        self.blocks = []
        
        # # Map4 Start and End Points
        self.start_point = [7.954360487979886, 6.822833826909669, 1.058209137433761]
        self.goal_point = [44.304797815557095, 29.328280798754054, 4.454834705539382]
        
        # # TEST Map2 Start and End Points
        # self.start_point = [0,20,2]
        # self.goal_point = [10, 20, 3]

        # TEST Map3 Start and End Points
        # self.start_point = [0,3,2]
        # self.goal_point = [20, 2, 4]

        # Map1 Start and End Points
        # self.start_point = [0.5, 1.0, 0.5]
        # self.goal_point  = [0.5, 5.0, 0.5]
        self.safety_margin = 0.5  # Safety margin around obstacles
        # self.safety_margin = 0.1  # Safety margin around obstacles



    ###############################################
    ##### TODO - Implement map file parsing ####### 
    ###############################################    
    def parse_map_file(self, filename):
        """
        Parse the map file and extract boundary and blocks
        coords = [xmin, ymin, zmin, xmax, ymax, zmax]
        colors = [r, g, b] each in [0, 1] (make sure color values are in range 0-1)
        self.blocks.append((coords, colors))
        self.boundary = [xmin, ymin, zmin, xmax, ymax, zmax]
        return True if successful, False otherwise (True if file was parsed successfully, without any error.)
        """
        try:
            with open(filename, "r") as f:
                for line in f:
                    # Remove leading/trailing whitespace
                    line = line.strip()
                    # Skip empty lines and comments
                    if not line or line.startswith("#"):
                        continue

                    tokens = line.split()

                    # Boundary line
                    if tokens[0].lower() == "boundary":
                        if len(tokens) != 7:
                            print(f"Invalid boundary line: {line}")
                            return False
                        self.boundary = [float(x) for x in tokens[1:7]]
                        print(f"Boundary parsed: {self.boundary}")

                    # Block line
                    elif tokens[0].lower() == "block":
                        if len(tokens) != 10:
                            print(f"Invalid block line: {line}")
                            return False
                        coords = [float(x) for x in tokens[1:7]]
                        colors = [float(c) / 255.0 for c in tokens[7:10]]
                        self.blocks.append((coords, colors))
                        print(f"Block parsed: coords={coords}, colors={colors}")

                    else:
                        print(f"Unknown line type: {line}")
                        return False
            print(f"\nTotal blocks parsed: {len(self.blocks)}")
            return True
        except Exception as e:
            print(f"Error parsing map file {filename}: {e}")
            return False


    ##############################################
    #### TODO - Implement collision checking #####
    ##############################################
    def is_point_in_free_space(self, point):
        """
        Check if a point is in free space (not inside any obstacle)
        Complete implementation with collision checking
        return True if free, False if in collision
        """
        x, y, z = point

        # 1. Check if inside boundary
        xmin, ymin, zmin, xmax, ymax, zmax = self.boundary
        if not (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax):
            print(f"Point {point} is outside boundary!")
            return False

        # 2. Check if inside any block (collision)
        for coords, _ in self.blocks:
            bxmin, bymin, bzmin, bxmax, bymax, bzmax = coords
            if (bxmin <= x <= bxmax and 
                bymin <= y <= bymax and 
                bzmin <= z <= bzmax):
                # print(f"Point {point} collides with block {coords}")
                return False

        # Free space
        # print(f"Point {point} is in free space")
        return True
    


    ##############################################
    #### TODO - Implement line - collision checking #####
    ##############################################
    def is_line_collision_free(self, p1, p2, num_checks=20):
        """
        Check if a line segment between two points is collision-free
        Used for RRT* edge validation
        return True if free, False if in collision
        """
        # Linear interpolation between p1 and p2
        for i in range(num_checks + 1):
            alpha = i / num_checks
            x = (1 - alpha) * p1[0] + alpha * p2[0]
            y = (1 - alpha) * p1[1] + alpha * p2[1]
            z = (1 - alpha) * p1[2] + alpha * p2[2]
            point = (x, y, z)

            # Use point collision checker
            if not self.is_point_in_free_space(point):
                # print(f"Collision detected at {point} between {p1} and {p2}")
                return False

        # If no collision detected
        # print(f"Line from {p1} to {p2} is collision-free")
        return True
    

    
    def generate_random_free_point(self):
        """
        Generate a random point in free space
        Used for RRT* sampling
        """
        if not self.boundary:
            return None
        
        xmin, ymin, zmin, xmax, ymax, zmax = self.boundary
        
        max_attempts = 1000
        for _ in range(max_attempts):
            x = np.random.uniform(xmin + self.safety_margin, xmax - self.safety_margin)
            y = np.random.uniform(ymin + self.safety_margin, ymax - self.safety_margin)
            z = np.random.uniform(zmin + self.safety_margin, zmax - self.safety_margin)
            
            point = [x, y, z]
            if self.is_point_in_free_space(point):
                return point
        
        print("Warning: Could not generate random free point after", max_attempts, "attempts")
        return None

    def get_environment_info(self):
        """Get information about the environment layout"""
        if not self.boundary:
            return "No boundary defined"
        
        xmin, ymin, zmin, xmax, ymax, zmax = self.boundary
        
        info = f"""
        Environment Information:
        Boundary: [{xmin}, {ymin}, {zmin}] to [{xmax}, {ymax}, {zmax}]
        Size: {xmax-xmin:.1f} x {ymax-ymin:.1f} x {zmax-zmin:.1f} meters
        Volume: {(xmax-xmin)*(ymax-ymin)*(zmax-zmin):.1f} cubic meters
        Obstacles: {len(self.blocks)} blocks
        Safety margin: {self.safety_margin} meters
        """
        
        if self.start_point and self.goal_point:
            distance = np.linalg.norm(np.array(self.goal_point) - np.array(self.start_point))
            info += f"  Start-Goal distance: {distance:.2f} meters\n"
        
        return info
