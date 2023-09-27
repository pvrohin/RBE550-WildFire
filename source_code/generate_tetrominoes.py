import pygame
import random

#Initially put all the possible tetrominos shape type and size into a list to pick from at random during generation
tetrominoes_list = []

#Various Tetromino shapes
tetronminoe_L = ("L", (30, 45)) 
tetronminoe_T = ("T", (45, 30))
tetronminoe_Z = ("Z", (45, 30))
tetronminoe_I = ("I", (15, 60))

#Append to list
tetrominoes_list.append(tetronminoe_L)
tetrominoes_list.append(tetronminoe_T)
tetrominoes_list.append(tetronminoe_Z)
tetrominoes_list.append(tetronminoe_I)

#Function to load the tetromino and its burning pair and scale it 
def load_tetrominoe(image, image_burning, size, pixels_per_meter):

    scaling = 15*pixels_per_meter

    image = pygame.transform.scale(pygame.image.load(image).convert_alpha(), (scaling, scaling))

    burning_image = pygame.transform.scale(pygame.image.load(image_burning).convert_alpha(),(scaling, scaling))

    return image, burning_image

#Function to grab a random tetromino and load it using the function defined above
def grab_random(pixels_per_meter):

    tetrominoe_location = "./images/"

    shape, size = random.choice(tetrominoes_list)

    image_location = f"{tetrominoe_location}/{shape}.png"

    burning_image_location = f"{tetrominoe_location}/{shape}_burning.png"

    image, burning_image = load_tetrominoe(image_location, burning_image_location, size, pixels_per_meter)

    return image, burning_image
