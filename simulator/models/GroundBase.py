from time import sleep, time
from models.Utils import *
from models.Search import astar_drone_relay_paths, drone_directions, DronesState
from models.Drone import Drone
import random

NUM_DRONES = 15
NUM_RELAY_DRONES = 40

def load_mission(drones, ground_stations):
    # add ground_station
    ground_stations.append((400, 305, 10))

    '''
    drones[0].is_access_point = True
    
    drones[1].is_access_point = True
    drones[2].is_access_point = True
    drones[6].is_access_point = True
    drones[12].is_access_point = True
    drones[9].is_access_point = True
    '''
    '''
    # automatic testing
    while True:
        for drone in drones:
            drone.addTask("moveTo", (random.randrange(10, 790), random.randrange(10, 590), random.randrange(5, 20)))
        sleep(2000)
    '''
    drones[0].addTask("moveTo", (400, 305, 10))
    
    #drones[12].addTask("moveTo", (500, 300, 10))

    drones[1].addTask("moveTo", (250, 300, 10))
    drones[1].connectedTo(drones[0])
    
    sleep(3)
    drones[1].addTask("moveTo", (400, 370, 10))
    drones[1].addTask("moveTo", (450, 400, 10))
    
    drones[2].addTask("moveTo", (280, 350, 10))
    drones[2].connectedTo(drones[0])
    
    sleep(3)
    drones[3].addTask("moveTo", (410, 320, 10))
    drones[3].connectedTo(drones[0])
    
    sleep(3)
    drones[3].addTask("moveTo", (300, 350, 10))
    drones[2].addTask("moveTo", (400, 290, 10))

    sleep(3)
    drones[2].addTask("moveTo", (300, 350, 10))
    drones[3].addTask("moveTo", (400, 290, 10))

    sleep(3)
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

    #drones[12].state == "ARMED"
    drones[12].addTask("moveTo", (240, 30, 10))

    sleep(1)
    drones[13].addTask("moveTo", (320, 550, 10))

    drones[14].addTask("moveTo", (390, 440, 10))
    
    #


def relay(mission_drones, relay_drones, ground_stations, shared_coords):
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
        connected_coords = [drone for drone in mission_drones if drone.is_access_point] # gs


        drones_connection = {drone: [] for drone in mission_drones if drone.state != "OFFLINE" or drone.is_access_point}

        #
        connected_coords = {drone_to_coords[drone] for drone in connected_coords}
        for gs in ground_stations: connected_coords.add(gs)

        #connected_coords = {coords for drone, coords in drone_to_coords.items() if drone.is_access_point}
        start_state = DronesState(drones_connection, connected_coords, drone_to_coords)

        # get points of interess
        poi = get_poi([drone.coords for drone in drones_connection.keys() if drone.state != "OFFLINE" and not drone.is_access_point], list(connected_coords), drone_to_coords)
        

        # eliminate points too close // confirm if really needed  // NOT good performant
        #print(len(poi))
        #poi = average_close_points(poi)
        #print(len(poi))

        for _ in range(len(shared_coords)):
            shared_coords.pop()

        for coord in poi:
            shared_coords.add(coord)

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

def get_poi(not_connected_coords, connected_coords, drone_to_coords):
    # points leading to closest ground station or blob of drones
    poi = set()
    for drone_coords in not_connected_coords:

        # get closest connected coord
        closest_coord = connected_coords[0]
        closest_distance = coords_distance(drone_coords, closest_coord)
        for cc_aux in connected_coords:
            distance = coords_distance(drone_coords, cc_aux)
            if distance < closest_distance:
                closest_coord = cc_aux
                closest_distance = distance

        # get points to closest ground station
        if closest_distance > Drone.OPTIMAL_DISTANCE_CONNECTION:
            gs_coords = closest_coord
            vector = (gs_coords[0]-drone_coords[0], gs_coords[1]-drone_coords[1], gs_coords[2]-drone_coords[2])
            fragmentation = closest_distance/Drone.OPTIMAL_DISTANCE_CONNECTION
            for i in range(1, int(fragmentation)+1):
                poi.add((gs_coords[0]-vector[0]*i/fragmentation, gs_coords[1]-vector[1]*i/fragmentation, gs_coords[2]-vector[2]*i/fragmentation))
        
        # get points to closest mission drone
        for drone2_coords in not_connected_coords:
            if drone_coords != drone2_coords:
                distance = coords_distance(drone_coords, drone2_coords)
                if distance < closest_distance and distance > Drone.OPTIMAL_DISTANCE_CONNECTION:
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

def average_close_points(poi):
    for p1 in poi:
        for p2 in poi:
            if p1 == p2: continue
            if coords_distance(p1, p2) <= 5:
                poi.remove(p1)
                poi.remove(p2)
                coord = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2)
                poi.append(coord)
                return average_close_points(poi)
    return poi