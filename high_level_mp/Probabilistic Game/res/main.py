from pytmx.util_pygame import load_pygame
import pygame
from pytmx import TiledObjectGroup
import game_objects as go


def draw_all_layers(tiled_map, screen):
    def normal():
        for x, y, image in layer.tiles():
            screen.blit(image, [x*tiled_map.tilewidth, y*tiled_map.tileheight])
    def entity():
        for tile in layer:
            if tile.visible == 1:
                screen.blit(tile.image, [tile.x, tile.y])
    def other():
        pass

    for layer in tiled_map.layers:
        if not isinstance(layer, TiledObjectGroup):
            normal()
        elif layer.name == "entities":
            entity()
        else:
            other()
    pygame.display.update()


def handle_keys(hero, event):
    AD_SPEED = 3      # pixels/40ms
    WS_SPEED = 3      # pixels/40ms
    JUMP_SPEED = 10   # pixels/40ms

    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_SPACE and not hero.on_ladder: # jump
            if not hero.jumping:        # can only jump once
                hero.on_ground = False  # stop falling into ground
                hero.jumping = True     # watch for stop on ladder impact
                hero.y_vel = -JUMP_SPEED
        if event.key == pygame.K_w and hero.on_ladder:  # move up
            hero.y_vel = -WS_SPEED
        if event.key == pygame.K_a:     # move left
            hero.x_vel = hero.x_vel - AD_SPEED
        if event.key == pygame.K_s and hero.on_ladder:  # move down
            hero.y_vel = WS_SPEED
        if event.key == pygame.K_d:     # move right
            hero.x_vel = hero.x_vel + AD_SPEED
        if event.key == pygame.K_f:     # interact
            hero.interact()
    elif event.type == pygame.KEYUP:
        if event.key == pygame.K_w and hero.on_ladder:  # stop move up
            hero.y_vel = 0
        if event.key == pygame.K_a:     # stop move left
            hero.x_vel = hero.x_vel + AD_SPEED
        if event.key == pygame.K_s and hero.on_ladder:  # stop move down
            hero.y_vel = 0
        if event.key == pygame.K_d:     # stop move right
            hero.x_vel = hero.x_vel - AD_SPEED

def main():
    # global tiled_map, walls, entities
    pygame.display.init()
    screen = pygame.display.set_mode((448, 416))    # 14 x 13 tiles @ 32x32 pixels each

    go.tiled_map = load_pygame('map.tmx')
    go.walls = [go.Wall(wall) for wall in go.tiled_map.layernames["wall_collisions"]]
    go.ladders = [go.Wall(wall) for wall in go.tiled_map.layernames["climbing_zones"]]

    go.regenerate_entities()

    hero = go.Hero(128 + 16, 0 + 16)
    clock = pygame.time.Clock()

    running = True
    while running:
        # handle every event since the last frame.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()                   # quit the screen
                running = False

            handle_keys(hero, event)            # deal with keyboard input

        hero.do_physics()                       # handle velocity-based movement and collisions
        draw_all_layers(go.tiled_map, screen)   # refresh background
        hero.draw(screen)                       # draw the hero to the screen
        pygame.display.update()                 # update the screen
        clock.tick(40)                          # 40 ms clock

main()
