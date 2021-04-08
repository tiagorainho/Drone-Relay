from time import sleep, time
from models.Utils import *
from models.Search import astar_drone_relay_paths, drone_directions, DronesState
from models.Drone import Drone

NUM_DRONES = 15
NUM_RELAY_DRONES = 60

def load_mission(drones):
    # drone 0 is the ground station
    #drones[0].state == "ARMED"
    drones[0].is_access_point = True
    drones[2].is_access_point = True
    drones[6].is_access_point = True
    drones[12].is_access_point = True
    
    #drones[0].addTask("moveTo", (270, 310, 10))

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

    sleep(1)
    drones[9].addTask("moveTo", (220, 100, 10))
    drones[9].connectedTo(drones[0])
    
    sleep(1)
    drones[10].addTask("moveTo", (200, 50, 10))

    drones[11].addTask("moveTo", (120, 220, 10))

    sleep(1)
    drones[12].addTask("moveTo", (240, 30, 10))

    sleep(1)
    drones[13].addTask("moveTo", (320, 550, 10))

    drones[14].addTask("moveTo", (390, 440, 10))
    
    #


def relay(mission_drones, relay_drones):

    drone_to_coords = dict()
    update_counter = 0
    refresh_state = True

    while(True):
        # update current drone to coords registry
        changed_drones = set()
        for drone in mission_drones:
            if drone not in drone_to_coords or drone.coords != drone_to_coords[drone]:
                changed_drones.add(drone)
                drone_to_coords[drone] = drone.coords

        if len(changed_drones) == 0:
            sleep(0.1)
            continue

        print("...")
        start_time = time()

        # get current access points
        gs = [drone for drone in mission_drones if drone.is_access_point]

        # get points of interess
        #poi = get_poi([mission_drone for mission_drone in mission_drones if mission_drone.state != "OFFLINE" or mission_drone.is_access_point], gs, drone_to_coords)
        
        '''
        if refresh_state:
            # update drones relay position
            drones_connection = {drone: [] for drone in mission_drones if drone.state != "OFFLINE" or drone.is_access_point}
            connected_coords = {coords for drone, coords in drone_to_coords.items() if drone.is_access_point}
            start_state = DronesState(drones_connection, connected_coords, drone_to_coords)
            refresh_state = False
        else:
            start_state = start_state.update_state(changed_drones, drone_to_coords)
        '''
        drones_connection = {drone: [] for drone in mission_drones if drone.state != "OFFLINE" or drone.is_access_point}
        connected_coords = {coords for drone, coords in drone_to_coords.items() if drone.is_access_point}
        start_state = DronesState(drones_connection, connected_coords, drone_to_coords)

        # get points of interess
        poi = get_poi([drone for drone in drones_connection.keys() if drone.state != "OFFLINE" or drone.is_access_point], gs, drone_to_coords)
        

        drones_state = astar_drone_relay_paths(start_state, poi)        

        if drones_state is not None:
            relays_needed = list(drones_state.drones_relay)
            
            # optimize relays needed
            relays_needed = optimize_relays(relays_needed, [coord for coord in drone_to_coords.values()])

            # change drone relay positions
            drones_movement = drone_directions(relay_drones, relays_needed)
            if drones_movement is not None:
                for drone, coord in drones_movement.items(): drone.addTask("moveTo", coord)
                for drone in relay_drones:
                    if drone not in drones_movement and drone.state == "ARMED":
                        drone.addTask("returnBase")

            print("update counter: " + str(update_counter) + "; time: " + str(time()-start_time) + " seconds")
        else:
            print("falhou counter: " + str(update_counter) + "; time: " + str(time()-start_time) + " seconds")
        update_counter += 1

def get_poi(drones, ground_stations, drone_to_coords):
    # points leading to closest ground station or blob of drones

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
        if closest_gs > Drone.OPTIMAL_DISTANCE_CONNECTION:
            gs_coords = drone_to_coords[gs]
            vector = (gs_coords[0]-drone_coords[0], gs_coords[1]-drone_coords[1], gs_coords[2]-drone_coords[2])
            fragmentation = closest_gs/Drone.OPTIMAL_DISTANCE_CONNECTION
            for i in range(1, int(fragmentation)+1):
                poi.add((gs_coords[0]-vector[0]*i/fragmentation, gs_coords[1]-vector[1]*i/fragmentation, gs_coords[2]-vector[2]*i/fragmentation))
        
        # get points to closest mission drone
        for drone2 in drones:
            if drone != drone2:
                drone2_coords = drone_to_coords[drone2]
                distance = coords_distance(drone_coords, drone2_coords)
                if distance < closest_gs and distance > Drone.OPTIMAL_DISTANCE_CONNECTION:
                    vector = (drone_coords[0]-drone2_coords[0], drone_coords[1]-drone2_coords[1], drone_coords[2]-drone2_coords[2])
                    fragmentation = distance/(Drone.OPTIMAL_DISTANCE_CONNECTION)
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