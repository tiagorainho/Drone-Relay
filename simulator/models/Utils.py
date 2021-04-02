import math

def get_poi(drones, ground_stations, depth):
    poi = []
    points = [(drone.x, drone.y, drone.z) for drone in drones]
    for _ in range(depth):
        aux = []
        points_passed = set()
        for p1 in points:
            p1_min_distance_to_gs = min([coords_distance(p1, gs.coords) for gs in ground_stations])
            for p2 in points:
                if p1 != p2:
                    if (p1, p2) in points_passed:
                        break
                    else: points_passed.add((p1, p2))
                    if p1_min_distance_to_gs < coords_distance(p1, p2): continue
                    middle_point = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2, (p1[2] + p2[2])/2)
                    aux.append(middle_point)
                    poi.append(middle_point)
        points.extend(aux)
    return [(round(p[0],2), round(p[1], 2), round(p[2], 2)) for p in set(poi)]

def coords_distance(coords1, coords2):
    return math.sqrt((coords1[0]-coords2[0])**2 + (coords1[1]-coords2[1])**2 + (coords1[2]-coords2[2])**2)

def signal(number):
    return 1 if number >= 0 else -1