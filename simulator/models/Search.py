from queue import PriorityQueue
from models.Drone import *
from models.Utils import coords_distance
import itertools

class Node:
    def __init__(self, position, parent, depth):
        self.position = position
        self.parent = parent
        self.depth = depth
        self.distance_to_goal = 0

    def apply_heuristics(self, closest):
        self.distance_to_goal = closest
        self.heuristic = self.depth + self.distance_to_goal

    def __eq__(self, other):
        return (self.position, self.parent, self.depth) == (other.position, other.parent, other.depth)

    def __lt__(self, other):
        return self.heuristic < other.heuristic
    
    def __hash__(self):
        return hash((self.position, self.parent, self.depth))
    
    def trail(self):
        path = set()
        current_node = self
        while current_node.parent is not None:
            path.add(current_node.position)
            current_node = current_node.parent
        return path

def astar_get_path(start, goals, poi):
    # start is the drone itself
    # goals are the ground station drones
    # poi are the points of interess
    closed = set()
    open_nodes = PriorityQueue()
    list_goal_coords = [goal.coords for goal in goals]
    goals_coords = set(list_goal_coords)
    start_node = Node(start.coords, None, 0)
    open_nodes.put((0, start_node))
    closed.add(start_node)

    while not open_nodes.empty():
        current_node = open_nodes.get()[1]
        # found solution
        if current_node.position in goals_coords:
            path = []
            while current_node.parent is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path

        # searching neighbor drones
        neighbor_drones = [drone for drone in poi+list_goal_coords if coords_distance(current_node.position, drone) < Drone.MAX_RADIUS_CONNECTION]
        closest = min([coords_distance(current_node.position, goal) for goal in goals_coords])
        for neighbor_drone in neighbor_drones:
            neighbor = Node(neighbor_drone, current_node, current_node.depth + 1)
            if neighbor in closed: continue
            if neighbor_drone in current_node.trail(): continue
            neighbor.apply_heuristics(closest)
            if neighbor not in closed: open_nodes.put((neighbor.heuristic, neighbor))
        closed.add(current_node)
    return None



def drone_directions(relay_drones, relays_needed):
    all_combinations = [list(zip(each_permutation, relays_needed)) for each_permutation in itertools.permutations(relay_drones, len(relays_needed))]
    if len(all_combinations) == 0: return None
    combination = all_combinations[0]
    shortest = sum([coords_distance(distribution[0].coords, distribution[1]) for distribution in all_combinations[0]])
    for c in all_combinations:
        distance = sum([coords_distance(distribution[0].coords, distribution[1]) for distribution in c])
        if distance < shortest:
            shortest = distance
            combination = c
    
    # comply with drone directions interface
    drones_movement = dict()
    for c in combination: drones_movement[c[0]] = c[1]    
    return drones_movement