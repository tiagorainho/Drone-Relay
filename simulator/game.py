import pygame
from models.Drone import Drone
from models.GroundBase import load_mission, relay, NUM_DRONES, NUM_RELAY_DRONES
import threading

pygame.font.init()
STAT_FONT = pygame.font.SysFont("comicsans", 25)

WIDTH = 800
HEIGHT = 600
FPS = 30


shared_coords = set()

def draw_window(pygame, screen, drone_list):
    screen.fill((0,0,0))
    for drone in drone_list:
        drone.draw(pygame, screen)
    
    # show mouse coordinates
    mouse_coords = pygame.mouse.get_pos()
    if mouse_coords:
        text = STAT_FONT.render(str(mouse_coords), 1, (255,255,255))
        screen.blit(text, (WIDTH-80, 10))

        active_relays = [drone for drone in drone_list if drone.type == "relay" and drone.state != "OFFLINE" and drone.coords != drone.initial_coords]
        text = STAT_FONT.render('nº drones relay: ' + str(len(active_relays)), 1, (255,255,255))
        screen.blit(text, (10, 10))

        # show points of interess
        for coord in shared_coords:
            pygame.draw.circle(screen, (255,0,0), (coord[0], coord[1]), 1)

    pygame.display.update()

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    simulation_running = True

    # create drones
    drone_list = [Drone((WIDTH/2, HEIGHT/2, 0)) for _ in range(NUM_DRONES)]
    relay_list = [Drone((WIDTH/2, HEIGHT/2, 0), "relay") for _ in range(NUM_RELAY_DRONES)]

    # add missions
    missions = threading.Thread(target=load_mission, args=(drone_list,), daemon=True)
    missions.start()

    # activate relay
    relay_algorithm = threading.Thread(target=relay, args=(drone_list, relay_list, shared_coords), daemon=True)
    relay_algorithm.start()

    # run simulation
    while simulation_running:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                simulation_running = False
                pygame.quit()
        draw_window(pygame, screen, drone_list+relay_list)

if __name__ == "__main__":
    main()