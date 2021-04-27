from queue import PriorityQueue
from models.Drone import *
from models.Utils import coords_distance
import itertools

class Node:
    def __init__(self, state, parent, depth):
        self.state = state
        self.parent = parent
        self.depth = depth

    def apply_heuristics(self):
        self.heuristic = self.state.heuristic
    
    def in_parent(self, state):
        if self.state == state:
            return True
        if self.parent is None:
            return False
        return self.parent.in_parent(state)

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.heuristic < other.heuristic
    
    def __hash__(self):
        return hash(self.state)
    
    def __str__(self):
        return str(self.state)

class DronesState:
    def __init__(self, drones_connection, connected_coords, drone_to_coords):
        self._drones_connection = drones_connection # dict of mission_drone and list of coords
        self._connected_coords = connected_coords # set of connected coords
        self._drone_to_coords = drone_to_coords
        self._mission_drones_coords = frozenset(coord for coord in list(self._drone_to_coords.values()))
        self._previous_distance = 0

    @property
    def drones_connection(self):
        return self._drones_connection
    
    @property
    def connected_coords(self):
        return self._connected_coords

    @property
    def drones_relay(self):
        drones = set()
        for relay_drones in self._drones_connection.values():
            for drone in relay_drones:
                if drone not in self._mission_drones_coords: drones.add(drone)
        return drones
    
    @property
    def mission_drones(self):
        return list(self._drones_connection.keys())
    
    @property
    def mission_drone_coords(self):
        return list(self._mission_drones_coords)

    @property
    def connected_drones(self):
        return [drone_coord for drone_coord in self._mission_drones_coords if drone_coord in self._connected_coords]

    @property
    def heuristic(self):
        distance = 0
        for drone in self.mission_drones:
            last_poi = self.last_poi(drone)
            
            min_distance = coords_distance(last_poi, next(iter(self._connected_coords)))
            for connected_coord in self._connected_coords:
                distance_aux = coords_distance(last_poi, connected_coord)
                if min_distance > distance_aux: min_distance = distance_aux
            distance += min_distance
        return distance + self._previous_distance + len(self.drones_relay)*1.5

    def update_state(self, drones_changed, drone_to_coords):

        # do this for performance increase

        for drone in self._drones_connection:
            pass

        
    def closest_connected_coord(self, drone):
        connected_coord = next(iter(self._connected_coords))
        drone_coords = self._drone_to_coords[drone]
        distance_connected_coord = coords_distance(drone_coords, connected_coord)
        
        for cc in self._connected_coords:
            distance = coords_distance(drone_coords, cc)
            if distance < distance_connected_coord:
                connected_coord = cc
                distance_connected_coord = distance
        return (connected_coord, distance_connected_coord)
    
    def last_poi(self, drone):
        drone_relays = self.get_relays(drone)
        return self.get_drone_coords(drone) if drone_relays == [] else drone_relays[-1]

    def get_relays(self, mission_drone):
        return self._drones_connection[mission_drone]
    
    def get_drone_coords(self, drone):
        return self._drone_to_coords[drone]

    def completed(self):
        #for drone in self.mission_drones:
        #    coords = self.get_drone_coords(drone)
        for coords in self._mission_drones_coords:
            if coords not in self._connected_coords: return False
        return True        
    
    def connect_to(self, mission_drone, coord_to_connect):
        # add new relay drone
        self._previous_distance += coords_distance(self.last_poi(mission_drone), coord_to_connect)
        self._drones_connection[mission_drone].append(coord_to_connect)        
        if coord_to_connect in self._connected_coords:
            # connect both the drone and the relay drones to the network
            self._connected_coords.add(self._drone_to_coords[mission_drone])
            for coord in self.get_relays(mission_drone): self._connected_coords.add(coord)
    
    def connect_drone(self, drone_to_connect, path_taken):
        drone_coords = self._drone_to_coords[drone_to_connect]
        self._connected_coords.add(drone_coords)
        for coord in path_taken:
            self._previous_distance += coords_distance(drone_coords, coord)
            self._connected_coords.add(coord)
        self._drones_connection[drone_to_connect].extend(path_taken)
    
    def get_path(self, not_connected_drone, closest_coords):
        closed_nodes = set()
        open_nodes = PriorityQueue()
        start_node = Node(self._drone_to_coords[not_connected_drone], None, 0)
        open_nodes.put((0, start_node))
        closed_nodes.add(start_node)

        while not open_nodes.empty():
            current_node = open_nodes.get()[1]
            # found solution
            if current_node.state in self._connected_coords:
                path = []
                while current_node != start_node:
                    path.append(current_node.state)
                    current_node = current_node.parent
                return path

            # expand new nodes
            for coord in closest_coords[current_node.state]:
                if current_node.in_parent(coord): continue
                new_node = Node(coord, current_node, current_node.depth + 1)
                new_node.heuristic = min([coords_distance(coord, cc) for cc in self._connected_coords])
                if new_node not in closed_nodes:
                    open_nodes.put((new_node.heuristic, new_node))
            closed_nodes.add(current_node)
        return None

    def __hash__(self):
        return hash(frozenset(self._drones_connection))*hash(frozenset(self._connected_coords))
    
    def __str__(self):
        return str(self._drones_connection)
    
    def __eq__(self, other):
        return self._drones_connection == other._drones_connection and self._connected_coords == other._connected_coords
    
    def deepcopy(self):
        return DronesState({key: value[:] for key, value in self._drones_connection.items()}, {value for value in self._connected_coords}, self._drone_to_coords)

def astar_drone_relay_paths(drones_state: DronesState,  poi):
    # get closest drones for performance reasons
    poi_extended = poi+[coord for coord in drones_state.mission_drone_coords]
    closest_poi = dict()
    for p1 in poi_extended:
        aux_list = []
        for p2 in poi_extended:
            distance = coords_distance(p1, p2)
            if p1 != p2 and distance <= Drone.RADIUS_CONNECTION_THRESHOLD and distance >= Drone.MINIMAL_DISTANCE: aux_list.append(p2)
        closest_poi[p1] = aux_list
    
    # prepare for start astar search
    closed_nodes = set()
    open_nodes = PriorityQueue()
    start_node = Node(drones_state, None, 0)
    open_nodes.put((0, start_node))
    closed_nodes.add(start_node)

    while not open_nodes.empty():
        current_node = open_nodes.get()[1]

        # expand new nodes
        not_connected_drones = [drone for drone in current_node.state.mission_drones if current_node.state.get_drone_coords(drone) not in current_node.state.connected_coords]

        # found solution
        if current_node.state.completed() or not_connected_drones == []: return current_node.state

        #print(not_connected_drones)
        for not_connected_drone in not_connected_drones:
            new_node = Node(current_node.state.deepcopy(), current_node, current_node.depth + 1)
            path = current_node.state.get_path(not_connected_drone, closest_poi)
            if path is not None:
                new_node.state.connect_drone(not_connected_drone, path)
                new_node.apply_heuristics()
                if new_node not in closed_nodes:
                    open_nodes.put((new_node.heuristic, new_node))
        closed_nodes.add(current_node)
    return None

def drone_directions(relay_drones, relays_needed):

    

    '''
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
    '''
    r = dict()
    for i in range(len(relays_needed)): r[relay_drones[i]] = relays_needed[i]
    return r