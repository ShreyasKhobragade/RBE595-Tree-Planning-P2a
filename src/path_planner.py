import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.random.seed(42)

class RRTNode:
    """Node for RRT* tree"""
    def __init__(self, position, parent=None):
        self.position = np.array(position, dtype=float)
        self.parent = parent
        self.cost = 0.0
        self.children = []

class PathPlanner:
    """
    Robust RRT* implementation for 3D path planning
    """
    
    def __init__(self, environment):
        self.env = environment
        self.waypoints = []
        self.tree_nodes = []
        
        # RRT* parameters
        self.max_iterations = 3000
        self.step_size = 1.0
        self.goal_radius = 1.0
        self.search_radius = 2.5
        self.goal_bias = 0.15  # 15% bias towards goal
        
    
    ############################################################################################################
    #### TODO - Implement RRT* path planning algorithm in 3D (use the provided environment class) ##############
    #### TODO - Store the final path in self.waypoints as a list of 3D points ##################################
    #### TODO - Add member functions as needed #################################################################
    ############################################################################################################
    def plan(self, start=None, goal=None, max_iterations=None, verbose=False):
        """Run RRT* and populate self.waypoints with a collision-free 3D path."""
        self.waypoints = []
        self.tree_nodes = []

        start = np.array(start if start is not None else self.env.start_point, dtype=float)
        goal  = np.array(goal  if goal  is not None else self.env.goal_point,  dtype=float)

        if max_iterations is None:
            max_iterations = self.max_iterations

        # Validate start and goal
        if not self.env.is_point_in_free_space(start):
            if verbose: print("[RRT*] Start is invalid.")
            return False
        if not self.env.is_point_in_free_space(goal):
            if verbose: print("[RRT*] Goal is invalid.")
            return False

        # Root node
        root = RRTNode(start, parent=None)
        root.cost = 0.0
        self.tree_nodes.append(root)

        goal_node = None

        for it in range(max_iterations):
            # 1) Sample
            rnd = self.sample_free(goal)

            # 2) Nearest
            nearest = self.tree_nodes[self.nearest_index(rnd)]

            # 3) Steer
            new_pos = self.steer(nearest.position, rnd, self.step_size)

            if not self.env.is_point_in_free_space(new_pos): continue
            if not self.env.is_line_collision_free(nearest.position, new_pos): continue

            # 4) Create new node
            new_node = RRTNode(new_pos, parent=nearest)
            new_node.cost = nearest.cost + self.distance(nearest.position, new_pos)

            # 5) Find neighbors and choose best parent
            neighbors = self.find_near_nodes(self.tree_nodes, new_node.position, self.neighbor_radius(len(self.tree_nodes)))

            if neighbors:
                best_parent, best_cost = self.choose_parent(neighbors, new_node.position)
                if best_parent is not None:
                    new_node.parent = best_parent
                    new_node.cost   = best_cost
                    best_parent.children.append(new_node)
                else:
                    nearest.children.append(new_node)
            else:
                nearest.children.append(new_node)

            # 6) Add to tree
            self.tree_nodes.append(new_node)

            # 7) Rewire neighbors
            self.rewire_tree(self.tree_nodes, new_node, neighbors)

            # 8) Check goal connection
            if self.distance(new_node.position, goal) <= self.goal_radius:
                if self.env.is_line_collision_free(new_node.position, goal):
                    goal_node = RRTNode(goal, parent=new_node)
                    goal_node.cost = new_node.cost + self.distance(new_node.position, goal)
                    new_node.children.append(goal_node)
                    self.tree_nodes.append(goal_node)

                    self.waypoints = self.extract_path(goal_node)
                    if verbose:
                        print(f"[RRT*] Path found at iteration {it+1}, cost={goal_node.cost:.3f}")
                    return True

        if verbose:
            print("[RRT*] No path found within iteration budget.")
        return False

    # ========================= Helpers =========================
    def distance(self, a, b):
        return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))

    def sample_free(self, goal):
        if np.random.rand() < self.goal_bias:
            return goal.copy()
        pt = self.env.generate_random_free_point()
        return np.array(pt if pt is not None else goal, float)

    def nearest_index(self, point):
        point = np.asarray(point)
        d2_best, idx_best = float("inf"), 0
        for i, node in enumerate(self.tree_nodes):
            d2 = np.sum((node.position - point)**2)
            if d2 < d2_best:
                d2_best, idx_best = d2, i
        return idx_best

    def steer(self, from_pt, to_pt, step):
        vec = np.asarray(to_pt, float) - np.asarray(from_pt, float)
        d = np.linalg.norm(vec)
        if d < 1e-12: return np.asarray(from_pt, float).copy()
        if d <= step: return np.asarray(to_pt, float)
        return np.asarray(from_pt, float) + (vec / d) * step

    def neighbor_radius(self, n_nodes):
        if n_nodes <= 1:
            return max(1.5 * self.step_size, self.search_radius)
        d = 3.0
        gamma = max(self.search_radius, 2.0 * self.step_size)
        r = gamma * (np.log(n_nodes) / n_nodes) ** (1.0 / d)
        return float(np.clip(r, 1.5 * self.step_size, self.search_radius))

    def find_near_nodes(self, tree, position, radius):
        r2 = float(radius) ** 2
        return [n for n in tree if np.sum((n.position - np.asarray(position)) ** 2) <= r2]

    def choose_parent(self, near_nodes, new_position):
        best_parent, best_cost = None, float("inf")
        for nb in near_nodes:
            if not self.env.is_line_collision_free(nb.position, new_position):
                continue
            cand = nb.cost + self.distance(nb.position, new_position)
            if cand < best_cost:
                best_parent, best_cost = nb, cand
        return best_parent, best_cost

    def rewire_tree(self, tree, new_node, near_nodes):
        for nb in near_nodes:
            if nb is new_node or nb is new_node.parent: continue
            cand_cost = new_node.cost + self.distance(new_node.position, nb.position)
            if cand_cost + 1e-9 < nb.cost:
                if self.env.is_line_collision_free(new_node.position, nb.position):
                    if nb.parent is not None:
                        try: nb.parent.children.remove(nb)
                        except ValueError: pass
                    nb.parent = new_node
                    new_node.children.append(nb)
                    delta = cand_cost - nb.cost
                    nb.cost = cand_cost
                    self.propagate_costs(nb, delta)

    def propagate_costs(self, node, delta):
        for ch in node.children:
            ch.cost += delta
            self.propagate_costs(ch, delta)

    def extract_path(self, end_node):
        path, cur = [], end_node
        while cur is not None:
            path.append(cur.position.tolist())
            cur = cur.parent
        return list(reversed(path))

    def simplify_path(self, path, max_iters=200):
        if self.env is None or len(path) < 3: return path
        pts = [np.asarray(p, float) for p in path]
        it = 0
        while it < max_iters and len(pts) > 2:
            i = np.random.randint(0, len(pts)-2)
            j = np.random.randint(i+2, len(pts))
            if self.env.is_line_collision_free(pts[i], pts[j]):
                del pts[i+1:j]
            it += 1
        return [p.tolist() for p in pts]
    
    def find_nearest_node(self, tree, point):
        """Return the nearest node in the given tree to the point."""
        point = np.asarray(point, float)
        best, best_d2 = None, float("inf")
        for n in tree:
            d2 = np.sum((n.position - point) ** 2)
            if d2 < best_d2:
                best, best_d2 = n, d2
        return best

    def is_path_valid(self, a, b):
        """Check if the straight-line path between a and b is valid."""
        return self.env.is_point_in_free_space(b) and self.env.is_line_collision_free(a, b)


    
    
    ############################################################################################################

    def visualize_tree(self, ax=None):
        """Visualize the RRT* tree"""
        if ax is None:
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            standalone = True
        else:
            standalone = False
        
        # Draw tree edges
        for node in self.tree_nodes:
            if node.parent is not None:
                ax.plot([node.parent.position[0], node.position[0]],
                       [node.parent.position[1], node.position[1]],
                       [node.parent.position[2], node.position[2]],
                       'b-', alpha=0.3, linewidth=0.5)
        
        # Draw tree nodes
        if self.tree_nodes:
            positions = np.array([node.position for node in self.tree_nodes])
            ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                      c='blue', s=10, alpha=0.6)
        
        # Draw final path
        if len(self.waypoints) > 0:
            waypoints = np.array(self.waypoints)
            ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 
                   'ro-', markersize=8, linewidth=3, label='RRT* Path')
        
        if standalone:
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('RRT* Tree and Path')
            ax.legend()
            plt.tight_layout()
            plt.show()
        
        return ax