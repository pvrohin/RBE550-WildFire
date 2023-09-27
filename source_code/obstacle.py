import pygame
from generate_tetrominoes import grab_random
from secrets import randbits
from math import sqrt

#The two possible states of a tetromino, burning or not burning
not_burning = 0
burning = 1

#Declare a class to track the states of all tetromino obstacles, 
class Obstacle():
    def __init__(self,xy_coordinate,pixels_per_meter):
        #Get the xy coordinates, pixels per meter and get a random pair of tetrominos, and set the necessary initial variables

        self.xy_coordinate = xy_coordinate
        self.pixels_per_meter = pixels_per_meter

        image, burning_image = grab_random(pixels_per_meter)

        self.image = image
        self.burning_image = burning_image

        if bool(randbits(1)):
            self.image = pygame.transform.flip(self.image, True, False)
            self.burning_image = pygame.transform.flip(self.burning_image, True, False)

        self.current_state = not_burning
        self.burning_time = 0.0
        self.already_burnt = False

        self.prev_node_burning_time = 0.0

        self.no_of_pixels = self.get_pixel_coverage()

        self.operating_surface = self.image

        x_xy_coordinate_pixel = xy_coordinate[0]*self.pixels_per_meter

        y_xy_coordinate_pixel = xy_coordinate[1]*self.pixels_per_meter

        self.xy_coordinate_pixel = (x_xy_coordinate_pixel,y_xy_coordinate_pixel)

        self.rect = self.operating_surface.get_rect(center=self.xy_coordinate_pixel)

        self.render()

    #Get the current pixel coverage at that instant of time
    def get_pixel_coverage(self):
        #Initialize the total number of pixels to zero, and calculate the total number of pixels that the current image is occupying
        total_no_of_pixels = 0
        width, height = self.image.get_size()

        for w in range(width):
            for h in range(height):

                condition = self.image.get_at((w,h))

                if condition.a == 255:
                    total_no_of_pixels = total_no_of_pixels + 1
        
        return total_no_of_pixels
    
    #Standard render function used to set the current state to burning or not burning depending on its current state
    def render(self):

        if self.current_state == not_burning:
            self.operating_surface = self.image
        
        else:
            self.operating_surface = self.burning_image

    def blit(self, surface):
        surface.blit(self.operating_surface, self.rect)

    def burn_tetromino(self):
        if self.current_state == not_burning:
            self.current_state = burning
            self.burning_time = 0.0
            self.prev_node_burning_time = 0.0
            self.already_burnt = True
        else:
            return
    
    def extinguish_tetromino(self):
        self.current_state = not_burning
        self.burning_time = 0.0
        self.prev_node_burning_time = 0.0

    def reset(self):
        self.extinguish_tetromino()
        self.already_burnt = False

    def tick(self, delta_t):
        if self.current_state == burning:
            self.burning_time  = self.burning_time + delta_t
        else:
            return
    
    def calc_distance(self, node):
        distance = sqrt((self.xy_coordinate[0] - node.xy_coordinate[0])**2 + (self.xy_coordinate[1] - node.xy_coordinate[1])**2 )

        return distance




    
        