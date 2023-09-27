import pygame
from math import floor, degrees, pi

from obstacle import Obstacle
from state_tracking import stateTracker

#Load the vehicle image to display in simulation
vehicle_image = "./images/firetruck.png"

#A class to track where the vehicle is in the map
class vehicleTracker(pygame.sprite.Sprite):

    def __init__(self, current_state, pixels_to_meter):
        self.current_state = current_state
        self.pixels_to_meter = pixels_to_meter

        self.current_surface = None

        self.path = None

        self.path_time_tracker = 0.0
        self.path_time_delta = 0.0
        self.target_state = None

        #self.image = pygame.image.load(vehicle_image).convert_alpha()
        #image_size = (2.3*pixels_to_meter, 6.1*pixels_to_meter)
        self.image = pygame.transform.scale(pygame.image.load(vehicle_image).convert_alpha(), (2.3*pixels_to_meter, 6.1*pixels_to_meter))
        
        self.render()

    #Function to blit/ move the vehicle across the current state where the current position of the truck is recorded as self.rect
    def blit(self, surface):
        surface.blit(self.current_surface, self.rect)

    #Render the screen according to the current position of the vehicle which is stored in self.rect
    def render(self):

        rotated_angle = self.current_state.theta + (pi/2)

        angle_to_rotate = -degrees(rotated_angle)

        self.current_surface = pygame.transform.rotate(self.image, angle_to_rotate)

        self.rect = self.current_surface.get_rect(center = (self.current_state.x*self.pixels_to_meter, self.current_state.y*self.pixels_to_meter))

        self.mask = pygame.mask.from_surface(pygame.transform.rotate(self.image, angle_to_rotate))

    #Draw the path traced by the vehicle
    def draw_path(self, surface):
        #If no path exists, return empty value from this function
        if self.path is None or len(self.path) < 2:
              return
 
        to_draw = self.path.copy()
        first = to_draw.pop(0)
        second  = to_draw.pop(0)
        while True:
            pygame.draw.line(surface, (255, 0, 0, 128), (first.x*self.pixels_to_meter, first.y*self.pixels_to_meter), (second.x*self.pixels_to_meter, second.y*self.pixels_to_meter), width=3)
            first = second
            if len(to_draw) == 0:
                break
            second = to_draw.pop(0)
        pygame.display.update()
     
    #Check collision of vehicle with current obstacle distribution in configuration space    
    def check_collision(self, obstacle):
        #Obstacle collision checking
        if obstacle.rect.colliderect(self.rect):
            return True
        return False
    
    #Check if vehicle is present at current goal location
    def present_at_goal(self):
        
        if self.path is None:
            return False
        
        if len(self.path) == 0:
            return True
        
        if self.current_state == self.path[-1]:
            return True
        
        else:
            return False
        
    #Reset path to plan new path next
    def reset_path(self):
        self.path_time_delta = 0.0
        self.path = None
        self.path_time_tracker = 0.0

    #Tick function to track current state
    def tick(self, time_delta):

        if self.path is None or len(self.path) == 0:
            return

        self.path_time_tracker += time_delta

        index_td = floor(self.path_time_tracker / self.path_time_delta)

        index_ptd = floor(self.path_time_tracker / time_delta)

        if index_td > index_ptd:
            index = index_td
        else:
            index = index_ptd

        #index = index_td if index_td > index_ptd else index_ptd

        goal_indication = self.path_time_delta * len(self.path)

        # If we've reached goal state, return
        if self.path_time_tracker >= goal_indication:
            self.current_state = self.path[-1]
            return

        if index >= len(self.path):
            self.current_state = self.path[-1]
            return

        # This is the current state now, whose params we'll take
        current_state = self.path[index]

        xdelta = current_state.xdot * time_delta
        ydelta = current_state.ydot * time_delta
        thetadelta = current_state.thetadot * time_delta

        if thetadelta > pi:
            thetadelta = -1*((2*pi) - thetadelta)
        elif thetadelta < -pi:
            thetadelta = (2*pi) + thetadelta

        x = current_state.x + xdelta
        y = current_state.y + ydelta
        theta = current_state.theta + thetadelta

        self.current_state = stateTracker((x, y), theta, self.current_state.psi, exact = True, x_dot = current_state.xdot, y_dot = current_state.ydot, theta_dot = current_state.thetadot)

        if self.path[-1].distance_between_neighbours(self.current_state) < 0.5:
            self.current_state = self.path[-1]
    
