from time import sleep
from models.Utils import *
from models.Search import astar_get_path, drone_directions
from models.Drone import Drone

NUM_DRONES = 4
NUM_RELAY_DRONES = 6

def load_mission(drones):
    # drone 0 is the ground station
    drones[0].is_access_point = True
    drones[1].is_access_point = True

    drones[1].addTask("moveTo", (250, 300, 10))
    drones[1].connectedTo(drones[0])

    drones[2].addTask("moveTo", (280, 350, 10))
    drones[2].connectedTo(drones[0])

    drones[3].addTask("moveTo", (410, 320, 10))
    drones[3].connectedTo(drones[0])
    
    sleep(4)
    drones[3].addTask("moveTo", (300, 350, 10))
    drones[2].addTask("moveTo", (400, 290, 10))

    sleep(4)
    drones[2].addTask("moveTo", (300, 350, 10))
    drones[3].addTask("moveTo", (400, 290, 10))

    sleep(4)
    drones[1].addTask("moveTo", (400, 390, 10))
    drones[3].addTask("moveTo", (290, 400, 10))

def relay(mission_drones, relay_drones):
    gs = [drone for drone in mission_drones if drone.is_access_point]
    
    while(True):
        # get points of interess
        poi = get_poi([mission_drone for mission_drone in mission_drones if mission_drone.state == "ARMED" or mission_drone.is_access_point], gs, 2)
        paths = dict()
        for mission_drone in mission_drones:
            path = astar_get_path(mission_drone, gs, poi+[mission_drone.coords for mission_drone in mission_drones])
            if path != None: paths[mission_drone.coords] = path
        
        # get worst possible relays needed
        relays_needed = []
        for key, value in paths.items():
            if len(value)>1: relays_needed.extend(value[1:])
        relays_needed = list(set(relays_needed))
        
        # optimize relays needed
        relays_needed = optimize_relays(relays_needed, [mission_drone.coords for mission_drone in mission_drones])

        # change drone relay positions
        drones_movement = drone_directions(relay_drones, relays_needed)
        if drones_movement != None:
            used_drones = set()
            for drone, coord in drones_movement.items():
                drone.addTask("moveTo", coord)
                used_drones.add(drone)
            for drone in relay_drones:
                if drone not in drones_movement:
                    drone.addTask("returnBase")
        #sleep(0.1)

def optimize_relays(relays_needed, mission_drones):
    for relay1 in relays_needed:
        for relay2 in relays_needed:
            if relay1 != relay2:
                r1_connections = [drone for drone in mission_drones + relays_needed if coords_distance(relay1, drone) < Drone.MAX_RADIUS_CONNECTION]
                r2_connections = [drone for drone in mission_drones + relays_needed if coords_distance(relay2, drone) < Drone.MAX_RADIUS_CONNECTION]
                if set(r1_connections) == set(r2_connections):
                    relays_needed.remove(relay2)
                    return optimize_relays(relays_needed, mission_drones)
    return relays_needed