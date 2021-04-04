from time import sleep
from models.Utils import *
from models.Search import astar_get_path, drone_directions
from models.Drone import Drone

NUM_DRONES = 10
NUM_RELAY_DRONES = 30

def load_mission(drones):
    # drone 0 is the ground station
    drones[0].is_access_point = True
    drones[1].is_access_point = True

    drones[1].addTask("moveTo", (250, 300, 10))
    drones[1].connectedTo(drones[0])

    drones[2].addTask("moveTo", (280, 350, 10))
    drones[2].connectedTo(drones[0])

    #

    drones[4].addTask("moveTo", (270, 330, 10))
    drones[4].connectedTo(drones[0])
    
    drones[5].addTask("moveTo", (210, 490, 10))
    drones[5].connectedTo(drones[0])
    
    drones[6].addTask("moveTo", (250, 270, 10))
    drones[6].connectedTo(drones[0])

    drones[7].addTask("moveTo", (170, 450, 10))
    drones[7].connectedTo(drones[0])

    drones[8].addTask("moveTo", (350, 400, 10))
    drones[8].connectedTo(drones[0])

    drones[9].addTask("moveTo", (220, 100, 10))
    drones[9].connectedTo(drones[0])
    
    #
    
    sleep(10)
    drones[3].addTask("moveTo", (410, 320, 10))
    drones[3].connectedTo(drones[0])
    
    sleep(3)
    drones[3].addTask("moveTo", (300, 350, 10))
    drones[2].addTask("moveTo", (400, 290, 10))

    sleep(4)
    drones[2].addTask("moveTo", (300, 350, 10))
    drones[3].addTask("moveTo", (400, 290, 10))

    sleep(4)
    drones[1].addTask("moveTo", (400, 390, 10))
    drones[3].addTask("moveTo", (200, 400, 10))
    
    

def relay(mission_drones, relay_drones):
    gs = [drone for drone in mission_drones if drone.is_access_point]
    mission_drones_history = None
    update_counter = 0
    while(True):
        if set([drone.coords for drone in mission_drones]) == mission_drones_history:
            sleep(0.1)
            continue
        mission_drones_history = set([drone.coords for drone in mission_drones])
        # get points of interess
        poi = get_poi([mission_drone for mission_drone in mission_drones if mission_drone.state == "ARMED" and not mission_drone.is_access_point], gs, 2)
        #print(poi)
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
            for drone, coord in drones_movement.items(): drone.addTask("moveTo", coord)
            for drone in relay_drones:
                if drone not in drones_movement and drone.state == "ARMED": drone.addTask("returnBase")
        

        print("update counter: " + str(update_counter))
        update_counter += 1
        
        #sleep(0.1)

def get_poi(drones, ground_stations, depth):
    # points leading to closest ground station or blob of drones
    poi = set()
    for drone in drones:
        gs = ground_stations[0]
        closest_gs = drone.distance(gs)

        # get closest ground station
        for gs_aux in ground_stations:
            distance = drone.distance(gs_aux)
            if distance < closest_gs:
                gs = gs_aux
                closest_gs = distance

        # get points to closest ground station
        if closest_gs > Drone.RADIUS_CONNECTION_THRESHOLD:
            vector = (drone.coords[0]-gs.coords[0], drone.coords[1]-gs.coords[1], drone.coords[2]-gs.coords[2])
            fragmentation = closest_gs/(Drone.RADIUS_CONNECTION_THRESHOLD)
            for i in range(1, int(fragmentation)+1):
                poi.add((drone.coords[0]-vector[0]*i/fragmentation, drone.coords[1]-vector[1]*i/fragmentation, drone.coords[2]-vector[2]*i/fragmentation))
        
        # get points to closest mission drone
        for drone2 in drones:
            if drone != drone2:
                distance = drone.distance(drone2)
                if distance < closest_gs and distance > Drone.RADIUS_CONNECTION_THRESHOLD:
                    vector = (drone.coords[0]-drone2.coords[0], drone.coords[1]-drone2.coords[1], drone.coords[2]-drone2.coords[2])
                    fragmentation = distance/(Drone.RADIUS_CONNECTION_THRESHOLD)
                    for i in range(1, int(fragmentation)+1):
                        poi.add((drone.coords[0]-vector[0]*i/fragmentation, drone.coords[1]-vector[1]*i/fragmentation, drone.coords[2]-vector[2]*i/fragmentation))
    return [(round(p[0],2), round(p[1], 2), round(p[2], 2)) for p in poi]

def optimize_relays(relays_needed, mission_drones):
    mission_drones_set = set(mission_drones)
    for relay1 in relays_needed:
        if relay1 in mission_drones_set: continue
        for relay2 in relays_needed:
            if relay1 != relay2:
                r1_connections = [drone for drone in mission_drones + relays_needed if coords_distance(relay1, drone) < Drone.MAX_RADIUS_CONNECTION]
                r2_connections = [drone for drone in mission_drones + relays_needed if coords_distance(relay2, drone) < Drone.MAX_RADIUS_CONNECTION]
                if set(r1_connections) == set(r2_connections):
                    relays_needed.remove(relay2)
                    return optimize_relays(relays_needed, mission_drones)
    return relays_needed