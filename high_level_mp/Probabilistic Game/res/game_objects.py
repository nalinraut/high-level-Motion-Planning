import pygame
from pygame import Rect
import random

tiled_map = None
walls = None
entities = None
ladders = None

# "complex" game logic
def interact_button(name):
    if random.uniform(0, 1) < 0.8:
        other_button_name = interact_button_helper(name)
        interact_button_helper(other_button_name)

def interact_button_helper(name):
    global entities
    name_parts = name.split("_")
    rock_name = "rock_" + name_parts[1]

    if name_parts[2] == "pressed":
        name_parts[2] = "unpressed"
        rock_visibility = 1
    else:
        name_parts[2] = "pressed"
        rock_visibility = 0

    compliment_name = '_'.join(name_parts)

    # hide button
    layer = tiled_map.layernames["entities"]
    for i in range(len(layer)):
        if layer[i].name == name:               # change button to invisible
            layer[i].visible = 0
        if layer[i].name == rock_name:          # change rock visibility
            layer[i].visible = rock_visibility
        if layer[i].name == compliment_name:    # change compliment to visible
            layer[i].visible = 1

    regenerate_entities()                       # update secondary references

    # return 2nd button name
    if name_parts[1] == "1":
        name_parts[1] = "2"
    elif name_parts == "2":
        name_parts[1] = "1"
    return '_'.join(name_parts)

def interact_rock(name):
    print "you can't interact with rocks."

def interact_collectible(name):
    layer = tiled_map.layernames["entities"]
    for i in range(len(layer)):
        if layer[i].name == name:               # change collectible to invisible
            layer[i].visible = 0
    regenerate_entities()                       # update secondary references

def interact_grave(name):
    shovel_found = reduce((lambda found, entity: found and "shovel" not in entity.name), entities, True)
    if not shovel_found:
        return

    layer = tiled_map.layernames["entities"]
    for i in range(len(layer)):
        if layer[i].name == name:               # change grave to invisible
            layer[i].visible = 0
        if layer[i].name == "rock_3":           # change rock 3 to invisible
            layer[i].visible = 0
    regenerate_entities()                       # update secondary references

interact_instructions = {
    'button_1_unpressed': interact_button,
    'button_2_unpressed': interact_button,
    'button_1_pressed': interact_button,
    'button_2_pressed': interact_button,
    'rock_1': interact_rock,
    'rock_2': interact_rock,
    'rock_3': interact_rock,
    'shovel': interact_collectible,
    'grave': interact_grave,
    'treasure': interact_collectible
}


def regenerate_entities():
    global tiled_map, entities
    entities = [Entity(entity) for entity in tiled_map.layernames["entities"] if entity.visible == 1]


class Hero(pygame.sprite.Sprite):
    def __init__(self, x, y):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load("./hero.png")

        self.rect = Rect(x - 10, y - 16, 20, 32)        # x and y are supposed to be center coordinates
        self.x_vel = 0  # pixels/40ms
        self.y_vel = 0  # pixels/40ms

        self.on_ground = False
        self.on_ladder = True
        self.jumping = False

    def draw(self, surface):
        surface.blit(self.image, (self.rect.x, self.rect.y))

    def do_physics(self):
        # gravity logic
        if not self.on_ground and not self.on_ladder:   # increase velocity if in air
            self.y_vel += 1
        elif self.y_vel == 0 and not self.on_ladder:    # otherwise fall into the ground
            self.y_vel = 0.5
            self.on_ground = False

        # ladder logic
        self.update_ladder_collision()
        if self.on_ladder and self.jumping:
            self.y_vel = 0
            self.jumping = False

        # velocity logic
        self.move(self.x_vel, 0)
        self.move(0, self.y_vel)

        # win logic
        if self.check_win_condition():
            assert False

    def check_win_condition(self):
        have_treasure = False
        in_win_location = self.rect.y < 16

        layer = tiled_map.layernames["entities"]
        for i in range(len(layer)):
            if layer[i].name == "treasure":  # change collectible to invisible
                have_treasure = not layer[i].visible


        return have_treasure and in_win_location

    def update_ladder_collision(self):
        self.on_ladder = False
        for ladder in ladders:
            if pygame.sprite.collide_rect(self, ladder): # check for ladder logic
                self.on_ladder = True

    def move(self, px, py):
        global entities, walls, ladders

        self.rect.x += px
        self.rect.y += py

        rocks = filter(lambda entity: "rock" in entity.name, entities)
        for collidable in walls + rocks:
            if pygame.sprite.collide_rect(self, collidable):
                if px > 0:
                    self.rect.right = collidable.rect.left
                if px < 0:
                    self.rect.left = collidable.rect.right
                if py > 0:
                    self.rect.bottom = collidable.rect.top
                    self.on_ground = True
                    self.jumping = False
                    self.y_vel = 0
                elif py < 0:
                    self.rect.top = collidable.rect.bottom
                    self.y_vel = 0

    def interact(self):
        colliding_entities = pygame.sprite.spritecollide(self, entities, False)
        if len(colliding_entities) > 0:
            entity = colliding_entities[0]
            interact_instructions[entity.name](entity.name)


class Wall(pygame.sprite.Sprite):
    def __init__(self, w):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.Surface((w.width, w.height))
        self.rect = Rect(w.x, w.y, w.width, w.height)

class Entity(pygame.sprite.Sprite):
    def __init__(self, e):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.Surface((e.width, e.height))
        self.rect = Rect(e.x, e.y, e.width, e.height)
        self.name = e.name