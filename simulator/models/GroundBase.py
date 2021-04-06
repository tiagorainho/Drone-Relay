from time import sleep, time
from models.Utils import *
from models.Search import drone_relay_paths, drone_directions, DronesState
from models.Drone import Drone

NUM_DRONES = 10
NUM_RELAY_DRONES = 20

def initialize_mission(drones):
    # drone 0 is the ground station
    drones[0].is_access_point = True
    #drones[1].is_access_point = True

def load_mission(drones):
    

    #drones[1].is_access_point = True
    drones[1].addTask("moveTo", (250, 300, 10))
    drones[1].connectedTo(drones[0])
    
    sleep(3)
    drones[1].addTask("moveTo", (400, 370, 10))
    drones[1].addTask("moveTo", (450, 400, 10))
    
    drones[2].addTask("moveTo", (280, 350, 10))
    drones[2].connectedTo(drones[0])
    
    sleep(5)
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

    # extended testing
    
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


def relay(mission_drones, relay_drones):
    # get initial drone state
    gs = [drone for drone in mission_drones if drone.is_access_point]
    #drones_connection = {drone: [] for drone in mission_drones if drone.state != "OFFLINE" or drone.is_access_point}

    mission_drones_history = None
    update_counter = 0

    while(True):

        if mission_drones_history == set([drone.coords for drone in mission_drones]):
            sleep(0.05)
            continue
        
        print("...")
        start_time = time()

        mission_drones_history = set([drone.coords for drone in mission_drones])
        
        # get current coords
        drone_to_coords = {drone: drone.coords for drone in mission_drones}

        # get points of interess
        poi = get_poi([mission_drone for mission_drone in mission_drones if mission_drone.state != "OFFLINE" or mission_drone.is_access_point], gs, drone_to_coords)

        #print(poi)

        # update drones relay position
        drones_connection = {drone: [] for drone in mission_drones if drone.state != "OFFLINE" or drone.is_access_point}
        connected_coords = {coords for drone, coords in drone_to_coords.items() if drone.is_access_point}
        start_state = DronesState(drones_connection, connected_coords, drone_to_coords)
        drones_state = drone_relay_paths(start_state, poi)
        
        #print(drones_state)

        if drones_state is not None:
            relays_needed = list(drones_state.drones_relay)
            
            # optimize relays needed
            #relays_needed = optimize_relays(relays_needed, [mission_drone.coords for mission_drone in mission_drones])

            # change drone relay positions
            drones_movement = drone_directions(relay_drones, relays_needed)
            #print(drones_movement)
            if drones_movement is not None:
                for drone, coord in drones_movement.items(): drone.addTask("moveTo", coord)
                for drone in relay_drones:
                    if drone not in drones_movement and drone.state == "ARMED":
                        drone.addTask("returnBase")

        print("update counter: " + str(update_counter) + "; time: " + str(time()-start_time) + " seconds")
        update_counter += 1

        sleep(0.05)

def get_poi(drones, ground_stations, drone_to_coords):
    # points leading to closest ground station or blob of drones

    # FALTA COMPOR -> GUARDAR OS PONTOS ANTES DA FUNCAO SENAO ELE TA A MEXER AO MSM TEMPO
    poi = set()
    for drone in drones:
        gs = ground_stations[0]
        drone_coords = drone_to_coords[drone]
        closest_gs = coords_distance(drone_coords, drone_to_coords[gs])

        # get closest ground station
        for gs_aux in ground_stations:
            distance = coords_distance(drone_coords, drone_to_coords[gs_aux])
            if distance < closest_gs:
                gs = gs_aux
                closest_gs = distance

        # get points to closest ground station
        if closest_gs > Drone.RADIUS_CONNECTION_THRESHOLD:
            gs_coords = drone_to_coords[gs]
            vector = (gs_coords[0]-drone_coords[0], gs_coords[1]-drone_coords[1], gs_coords[2]-drone_coords[2])
            fragmentation = closest_gs/(Drone.OPTIMAL_DISTANCE_CONNECTION)
            for i in range(1, int(fragmentation)):
                poi.add((gs_coords[0]-vector[0]*i/fragmentation, gs_coords[1]-vector[1]*i/fragmentation, gs_coords[2]-vector[2]*i/fragmentation))
        
        # get points to closest mission drone
        for drone2 in drones:
            if drone != drone2:
                drone2_coords = drone_to_coords[drone2]
                distance = coords_distance(drone_coords, drone2_coords)
                if distance < closest_gs and distance > Drone.RADIUS_CONNECTION_THRESHOLD:
                    vector = (drone_coords[0]-drone2_coords[0], drone_coords[1]-drone2_coords[1], drone_coords[2]-drone2_coords[2])
                    fragmentation = distance/(Drone.RADIUS_CONNECTION_THRESHOLD)
                    for i in range(1, int(fragmentation)+1):
                        poi.add((drone_coords[0]-vector[0]*i/fragmentation, drone_coords[1]-vector[1]*i/fragmentation, drone_coords[2]-vector[2]*i/fragmentation))
    return [coord for coord in poi]

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