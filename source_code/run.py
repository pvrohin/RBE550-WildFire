#Adapted slightly from https://github.com/hlfshell

from obstacle import burning, not_burning, Obstacle
from planners import AStarPlanner, PRMAStarPlanner
from prm import PRM
from state_tracking import stateTracker
from vehicle_tracking import vehicleTracker

import pygame
import argparse
import math
from time import sleep, time_ns
from random import choice, randint, random
from secrets import randbits
from math import floor, pi, sqrt

parser = argparse.ArgumentParser()
parser.add_argument("--algorithm", type=int, help="Enter 1 for A*, 2 for PRM, leave arguments empty to run both")
args = parser.parse_args()

#Some constant parameters for building simulation
proximity_to_goal = 10.0
proximity_to_ignite = 30.0

time_spread = 20.0
on_fire_seconds = 60.0

max_simulation_time = 3600

class Run():
    def __init__(self, win_size, pixels_per_meter, per_second_time = 1.0):

        self.win_size = win_size
        self.display_surface = pygame.display.set_mode(self.win_size)
        pygame.display.set_caption("Wildfire")

        self.pixels_per_meter = pixels_per_meter

        self.frames_per_sec = pygame.time.Clock()
        self.required_fps = 60

        self.obstacles = []
        self.not_reached_obstacles = []

        self.goal = None

        self.time_per_frame = per_second_time/self.required_fps

        self.current_time = 0.0

        self.time_last_set_fire = 0.0

        self.prm_path = None

        self.vehicle = None

        self.map = None

        # Data for plotting later
        self.time_steps = []
        self.fire_percentage = []
        self.time_to_path = []
        self.distance_to_goal = []
        self.intact = []

        self.render()

    def current_obstacle_coverage(self):
        pixels_occupied = 0

        width, height = self.display_surface.get_size()

        total_pixels_count = width * height

        for obstacle in self.obstacles:
            pixels_occupied = pixels_occupied + obstacle.no_of_pixels

        obstacle_percentage = pixels_occupied / total_pixels_count

        return obstacle_percentage
    
    def obstacle_fill(self, percentage):
        while True:
            if self.current_obstacle_coverage() >= percentage:
                self.obstacles.sort(key = lambda o: o.xy_coordinate[1])
                return
            
            x = randint(0, 250)
            y = randint(0, 250)
            self.obstacles.append(Obstacle((x, y), self.pixels_per_meter))

    def create_vehicle(self):
        while self.vehicle is None:
            x = randint(0, 250)
            y = randint(0, 250)
            theta = random() * pi
            if randbits(1):
                theta = -theta

            vehicle = vehicleTracker(stateTracker(
                (x, y),
                theta,
                0.0
            ), self.pixels_per_meter)

            if not self.collision_detection(vehicle):
                self.vehicle = vehicle

    def generate_obstacles(self, fill_percentage):
        iterations = 0
        max_iterations = 1_000
        while self.obstacle_fill() < fill_percentage:
            iterations += 1
            if iterations > max_iterations:
                raise Exception("Can not fill map to sufficient percentage")
            try:
                self.add_random_obstacle()
            except:
                continue

    def generate_map(self):
        self.map = PRM(
            (250, 250),
            10_000,
            self.collision_detection,
            self.pixels_per_meter
        )
        print("Creating Probablistic Road Map")
        self.map.generate()
        print("Map created")

    def render(self):
        # Draw background
        pygame.Surface.fill(self.display_surface, (0, 255, 0))

        for obstacle in self.obstacles:
            obstacle.render()
            self.display_surface.blit(obstacle.operating_surface, obstacle.rect)
        
        if self.map is not None:
            self.map.render(self.display_surface)

            if self.prm_path is not None:
                self.draw_prm_path()

        if self.vehicle is not None:
            self.vehicle.draw_path(self.display_surface)
            self.vehicle.render()
            self.vehicle.blit(self.display_surface)

        pygame.display.update()

    def collision_detection(self, vehicle):
        for obstacle in self.obstacles:
            if vehicle.check_collision(obstacle):
                return True
        return False

    def tick(self):
        if self.goal is None and self.vehicle.path is None:
            burning_obstacles = self.obstacles_by_state(burning)
            burning_obstacles = [
                    o for o \
                        in burning_obstacles \
                        if o.xy_coordinate not in self.not_reached_obstacles
                ]
            if len(burning_obstacles) > 0:
                burning_obstacles.sort(
                    key=lambda o: self.vehicle.current_state.calc_obstacle_distance(o)
                )
                chosen_obstacle = burning_obstacles[0]
                self.goal = chosen_obstacle.xy_coordinate

                self.plan()
        
        for obstacle in self.obstacles:
            obstacle.tick(self.time_per_frame)
        
        for obstacle in self.obstacles_by_state(burning):
            if obstacle.burning_time - obstacle.prev_node_burning_time >= time_spread:
                obstacle.prev_node_burning_time = obstacle.burning_time
                to_ignite = self.obstacles_within_range(obstacle, proximity_to_ignite)
                for ignite in to_ignite:
                    if ignite != obstacle:
                        ignite.burn_tetromino()
        
        self.vehicle.tick(self.time_per_frame)

        if self.current_time > self.time_last_set_fire + on_fire_seconds or self.current_time == 0.0:
            self.time_last_set_fire = self.current_time
            extinguished_obstacles = self.obstacles_by_state(not_burning)
            if len(extinguished_obstacles) > 0:
                obstacle = choice(extinguished_obstacles)
                obstacle.burn_tetromino()
        
        if self.vehicle.present_at_goal():
            goal_obstacle = [o for o in self.obstacles if o.xy_coordinate == self.goal][0]
            goal_obstacle.extinguish_tetromino()
            self.goal = None
            self.vehicle.reset_path()
            self.prm_path = None

        self.current_time += self.time_per_frame

        self.time_steps.append(self.current_time)
        self.fire_percentage.append(
            len(self.obstacles_by_state(burning)) / len(self.obstacles)
        )
        self.intact.append(
            len(self.obstacles_intact()) / len(self.obstacles)
        )

    def loop(self):
        self.time_last_set_fire = 0.0

        while True:
            self.render()
            self.tick()
            pygame.event.get() # To prevent feezing, get input events
            self.frames_per_sec.tick(self.required_fps)
            print(self.current_time)
            print(max_simulation_time)
            if self.current_time >= max_simulation_time:
                self.end()
                return

    def plan(self):
        start_time = time_ns()
        if self.map is not None:
            self.prm_plan()
        else:
            self.astar_plan()
        end_time = time_ns()    
        if self.vehicle.path is not None:
            self.time_to_path.append(end_time - start_time)
            distance = sqrt(
                (self.vehicle.current_state.x - self.goal[0])**2 +
                (self.vehicle.current_state.y - self.goal[1])**2
            )
            self.distance_to_goal.append(distance)
    
    def prm_plan(self):
        # Get the closest node to the vehicle
        start = self.map.get_nodes_nearest(
            (self.vehicle.current_state.x, self.vehicle.current_state.y),
            1
        )[0]
        goal = self.map.get_nodes_nearest(
            self.goal,
            1
        )[0]

        try:
            self.planner = PRMAStarPlanner(
                start,
                goal,
                self.map,
                self.display_surface,
                self.pixels_per_meter
            )
            self.prm_path = self.planner.search()
        except Exception as e:
            print("Could not solve path - prm")
            print(e)
            self.not_reached_obstacles.append(self.goal)
            self.goal = None
            return

        self.draw_prm_path()

        prm_path = self.prm_path.copy()
        current_node = prm_path.pop(0)
        current_vehicle_state = self.vehicle.current_state
        planner_time_delta = 0.25
        self.vehicle.path = []
        self.vehicle.path_time_delta = planner_time_delta
        while len(prm_path) > 0:
            self.planner = AStarPlanner(
                current_vehicle_state,
                current_node,
                2.0,
                planner_time_delta,
                self.collision_detection,
                self.display_surface,
                self.pixels_per_meter
            )

            try:
                path = self.planner.search()
                # self.unreachable_obstacles = []
                if len(path) <= 1:
                    current_node = prm_path.pop(0)
                    continue

                self.vehicle.path += path[1:]
                current_vehicle_state = path[-1]

                current_node = prm_path.pop(0)
            except Exception as e:
                print("Could not solve path - astar")
                print(e)
                self.not_reached_obstacles.append(self.goal)
                self.goal = None
                self.vehicle.path = None
                return

    def astar_plan(self):
        planner_time_delta = 0.25
        self.planner = AStarPlanner(
            self.vehicle.current_state,
            self.goal,
            proximity_to_goal,
            planner_time_delta,
            self.collision_detection,
            self.display_surface,
            self.pixels_per_meter
        )

        try:
            path = self.planner.search()
            # self.unreachable_obstacles = []
            self.vehicle.path = path
            self.vehicle.path_time_delta = planner_time_delta
        except Exception as e:
            print("Could not solve path")
            print(e)
            self.not_reached_obstacles.append(self.goal)
            self.goal = None

    def draw_prm_path(self):
        if self.prm_path is None:
            return
        if len(self.prm_path) <= 1:
            return

        color = (0, 0,255, 128)
        drawn = self.prm_path.copy()
        first = drawn.pop(0)
        second  = drawn.pop(0)
        while True:
            firstxy = (first[0]*self.pixels_per_meter, first[1]*self.pixels_per_meter)
            secondxy = (second[0]*self.pixels_per_meter, second[1]*self.pixels_per_meter)
            pygame.draw.line(self.display_surface, color, firstxy, secondxy, width=3)
            first = second
            if len(drawn) == 0:
                break
            second = drawn.pop(0)
        pygame.display.update()

    def obstacles_by_state(self, state):
        by_state_obstacles = []
        for obstacle in self.obstacles:
            if obstacle.current_state == state:
                by_state_obstacles.append(obstacle)

        return by_state_obstacles
    
    def obstacles_within_range(self, obstacle, range):
        within_range_obstacles = []

        for other in self.obstacles:
            if obstacle.calc_distance(other) <= range:
                within_range_obstacles.append(other)

        return within_range_obstacles
    
    def obstacles_intact(self):
        intact_obstacles_list = []

        for obstacle in self.obstacles:
            if not obstacle.already_burnt:
                intact_obstacles_list.append(obstacle)

        return intact_obstacles_list

    def reset(self):
        self.time = 0.0
        for obstacle in self.obstacles:
            obstacle.reset()

        self.vehicle.path = None
        self.prm_path = None
        self.vehicle.path_time_tracker = 0.0

        self.time_steps = []
        self.fire_percentage = []
        self.time_to_path = []
        self.distance_to_goal = []
        self.intact = []

    def end(self):
        print("Finished!")

def astar():
    run = Run((1250, 1250),5,per_second_time = 5.0)

    run.obstacle_fill(0.2)
    run.create_vehicle()
    run.render()

    run.loop()

def prm():
    run = Run((1250, 1250),5,per_second_time = 5.0)

    run.obstacle_fill(0.2)
    run.create_vehicle()
    run.render()
    run.generate_map()
    run.loop()

def both():
    run = Run((1250, 1250),5,per_second_time = 1.0)

    run.obstacle_fill(0.2)
    run.create_vehicle()
    run.render()
    run.loop()
    run.reset()
    run.generate_map()
    run.loop()

def main():

    if args.algorithm == 1:
        astar()
    elif args.algorithm == 2:
        prm()
    else:
        both()

if __name__=="__main__":
    main()
