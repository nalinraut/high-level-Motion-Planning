import random
import sys
import manual
from random import randint
import numpy as np


# Parameters
SPEED = 50
LENGTH = 1000
PENALTY = -1
REWARD = 100000


def m_dist(d1, d2):
    """Calculates manhattan distance between two items"""
    return abs(d1.x - d2.x) + abs(d1.y - d2.y)

class Item:
    """storage object for related state variables"""
    def __init__(self, res_path):
        self.x = random.randint(0, LENGTH + 1)
        self.y = random.randint(0, LENGTH + 1)
        self.res_path = res_path
        self.img = None

class State:
    """
    maintains a full state representation of the world at any given time.  Per the continuous playground problem
    definition, it contains:
        Effectors:   [crosshair, eye, hand]
        Objects:     [green_button, red_button, light_switch, bell, ball]
        Monkey:      [monkey]
        Environment: [music_playing, light_on]
    """
    def __init__(self):
        # instantiate playroom environment booleans
        self.music_playing = bool(random.getrandbits(1))
        self.light_on = bool(random.getrandbits(1))
        self.monkey_cried = False

        # instantiate playroom effectors
        self.crosshair = Item('./res/crosshair.png')
        self.eye = Item('./res/eye.png')
        self.hand = Item('./res/hand.png')

        # instantiate playroom items
        self.green_button = Item('./res/green_button.png')
        self.red_button = Item('./res/red_button.png')
        self.light_switch = Item('./res/light_switch.png')
        self.bell = Item('./res/bell.png')
        self.ball = Item('./res/ball.png')

        # instantiate playroom monkey (kind-of irrelevant)
        self.monkey = Item('./res/monkey.png')

        self.items = [self.green_button, self.red_button, self.light_switch, self.bell, self.ball,
                      self.crosshair, self.eye, self.hand, self.monkey]

        if self.check_overlap():
            self = State()

    def check_overlap(self):
        """determines if any objects have been randomly spawned too closely to one another"""
        for i in range(len(self.items)):
            for j in range(i):
                if m_dist(self.items[j], self.items[i]) < 2*SPEED:
                    return True
        return False

    def list_states(self):
        s1 = self.green_button.x - self.hand.x
        s2 = self.green_button.y - self.hand.y
        s3 = self.red_button.x - self.hand.x
        s4 = self.red_button.y - self.hand.y
        s5 = self.bell.x - self.hand.x
        s6 = self.bell.y - self.hand.y
        s7 = self.ball.x - self.hand.x
        s8 = self.ball.y - self.hand.y
        s9 = self.light_switch.x - self.hand.x
        s10 = self.light_switch.y - self.hand.y

        s11 = self.green_button.x - self.crosshair.x
        s12 = self.green_button.y - self.crosshair.y
        s13 = self.red_button.x - self.crosshair.x
        s14 = self.red_button.y - self.crosshair.y
        s15 = self.bell.x - self.crosshair.x
        s16 = self.bell.y - self.crosshair.y
        s17 = self.ball.x - self.crosshair.x
        s18 = self.ball.y - self.crosshair.y
        s19 = self.light_switch.x - self.crosshair.x
        s20 = self.light_switch.y - self.crosshair.y

        s21 = self.green_button.x - self.eye.x
        s22 = self.green_button.y - self.eye.y
        s23 = self.red_button.x - self.eye.x
        s24 = self.red_button.y - self.eye.y
        s25 = self.bell.x - self.eye.x
        s26 = self.bell.y - self.eye.y
        s27 = self.ball.x - self.eye.x
        s28 = self.ball.y - self.eye.y
        s29 = self.light_switch.x - self.eye.x
        s30 = self.light_switch.y - self.eye.y

        s31 = self.music_playing*1
        s32 = self.light_on*1
        s33 = self.monkey_cried*1

        return [s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18, s19, s20, s21, s22, s23, s24, s25, s26, s27, s28, s29, s30, s31, s32, s33]
        
class Agent:
    """Each method in this class represents of of 13 possible agent actions. (NOT THE SAME AS AGENT OPTIONS)."""
    def __init__(self, state):
        self.state = state
        self.agent_options = { 'hand_to_green_button':0, 'hand_to_red_button':1, 'hand_to_bell':2, 'hand_to_ball':3, 'hand_to_light':4, 
                               'marker_to_green_button':5, 'marker_to_red_button':6, 'marker_to_bell':7, 'marker_to_ball':8, 'marker_to_light':9,
                               'eye_to_green_button':10, 'eye_to_red_button':11, 'eye_to_bell':12, 'eye_to_ball':13, 'eye_to_light':14,
                               'interact_green_button':15, 'interact_red_button':16, 'interact_bell':17, 'interact_ball':18, 'interact_light_on':19, 'interact_light_off':20 }
        self.state_chg = False
        self.noStates = 33
        
    def execute(self,option):
        self.state_chg = False
        mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        if option == 0 :
            self.state.hand.x = self.state.green_button.x + np.random.randint(-50,50)
            self.state.hand.y = self.state.green_button.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 1:
            self.state.hand.x = self.state.red_button.x + np.random.randint(-50,50)
            self.state.hand.y = self.state.red_button.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        elif option == 2:
            self.state.hand.x = self.state.bell.x + np.random.randint(-50,50)
            self.state.hand.y = self.state.bell.y + np.random.randint(-50,50)
            self.state_chg = True
            mask =  [1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 3:
            self.state.hand.x = self.state.ball.x + np.random.randint(-50,50)
            self.state.hand.y = self.state.ball.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 4:
            self.state.hand.x = self.state.light_switch.x + np.random.randint(-50,50)
            self.state.hand.y = self.state.light_switch.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 5:
            self.state.crosshair.x = self.state.green_button.x + np.random.randint(-50,50)
            self.state.crosshair.y = self.state.green_button.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0]
           
        elif option == 6:
            self.state.crosshair.x = self.state.red_button.x + np.random.randint(-50,50)
            self.state.crosshair.y = self.state.red_button.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 7:
            self.state.crosshair.x = self.state.bell.x + np.random.randint(-50,50)
            self.state.crosshair.y = self.state.bell.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 8:
            self.state.crosshair.x = self.state.ball.x + np.random.randint(-50,50)
            self.state.crosshair.y = self.state.ball.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 9:
            self.state.crosshair.x = self.state.light_switch.x + np.random.randint(-50,50)
            self.state.crosshair.y = self.state.light_switch.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0]
            
        elif option == 10:
            self.state.eye.x = self.state.green_button.x + np.random.randint(-50,50)
            self.state.eye.y = self.state.green_button.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0]
          
        elif option == 11:
            self.state.eye.x = self.state.red_button.x + np.random.randint(-50,50)
            self.state.eye.y = self.state.red_button.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0]
         
        elif option == 12:
            self.state.eye.x = self.state.bell.x + np.random.randint(-50,50)
            self.state.eye.y = self.state.bell.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0]
           
        elif option == 13:
            self.state.eye.x = self.state.ball.x + np.random.randint(-50,50)
            self.state.eye.y = self.state.ball.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0]
           
        elif option == 14:
            self.state.eye.x = self.state.light_switch.x + np.random.randint(-50,50)
            self.state.eye.y = self.state.light_switch.y + np.random.randint(-50,50)
            self.state_chg = True
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0]
          
        elif option == 15:
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0]
            if self.state.light_on == True and m_dist(self.state.hand, self.state.green_button) <= SPEED and m_dist(self.state.eye, self.state.green_button) <= SPEED:
                self.state.music_playing = True
                self.state_chg = True
                
        elif option == 16:
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0]
            if self.state.light_on == True and m_dist(self.state.hand, self.state.red_button) <= SPEED and m_dist(self.state.eye, self.state.red_button) <= SPEED:
                self.state.music_playing = False
                self.state_chg = True
                  
        elif option == 17:
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            s = self.state
            bell = m_dist(s.hand, s.bell) <= SPEED and m_dist(s.eye, s.bell) <= SPEED
            if s.light_on and bell:
                self.state_chg = False
                
        elif option == 18:
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
            s = self.state # for shorter lines
            ball = m_dist(s.hand, s.ball) <= SPEED and m_dist(s.eye, s.ball) <= SPEED
            bell = m_dist(s.crosshair, s.bell) <= SPEED

            if (s.light_on == False) and ball and bell and s.music_playing:
                self.state.monkey_cried = True
                self.state_chg = True
               
        elif option == 19:
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0]
            s = self.state
            light_dist = m_dist(s.hand, s.light_switch) <= SPEED and m_dist(s.eye, s.light_switch) <=SPEED

            if s.light_on == False and light_dist:
                self.light_on = True
                self.state_chg = True
               
        elif option == 20:
            mask = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0]
            s = self.state
            light_dist = m_dist(s.hand, s.light_switch) <= SPEED and m_dist(s.eye, s.light_switch) <=SPEED

            if s.light_on == True and light_dist:
                self.light_on = False
                self.state_chg = True
            
        return mask

    def interact(self):
        s = self.state # for shorter lines
        switch = m_dist(s.hand, s.light_switch) <= SPEED and m_dist(s.eye, s.light_switch) <= SPEED
        g_button = m_dist(s.hand, s.green_button) <= SPEED and m_dist(s.eye, s.green_button) <= SPEED
        r_button = m_dist(s.hand, s.red_button) <= SPEED and m_dist(s.eye, s.red_button) <= SPEED
        ball = m_dist(s.hand, s.ball) <= SPEED and m_dist(s.eye, s.ball) <= SPEED
        bell = m_dist(s.crosshair, s.bell) <= SPEED

        if switch:
            s.light_on = not s.light_on

        if g_button and s.light_on:
            s.music_playing = True

        if r_button and s.light_on:
            s.music_playing = False

        if ball and bell and s.light_on and s.music_playing:
            self.state.monkey_cried = True
            return REWARD     # the monkey is pissed

        return PENALTY        # the monkey is sleeping


    def move_hand_up(self):
        if self.state.hand.y - SPEED > 0:
            self.state.hand.y = self.state.hand.y - SPEED
        return PENALTY
    def move_hand_down(self):
        if self.state.hand.y + SPEED < LENGTH:
            self.state.hand.y = self.state.hand.y + SPEED
        return PENALTY
    def move_hand_left(self):
        if self.state.hand.x - SPEED > 0:
            self.state.hand.x = self.state.hand.x - SPEED
        return PENALTY
    def move_hand_right(self):
        if self.state.hand.x + SPEED < LENGTH:
            self.state.hand.x = self.state.hand.x + SPEED
        return PENALTY
    def move_crosshair_up(self):
        if self.state.crosshair.y - SPEED > 0:
            self.state.crosshair.y = self.state.crosshair.y - SPEED
        return PENALTY
    def move_crosshair_down(self):
        if self.state.crosshair.y + SPEED < LENGTH:
            self.state.crosshair.y = self.state.crosshair.y + SPEED
        return PENALTY
    def move_crosshair_left(self):
        if self.state.crosshair.x - SPEED > 0:
            self.state.crosshair.x = self.state.crosshair.x - SPEED
        return PENALTY
    def move_crosshair_right(self):
        if self.state.crosshair.x + SPEED < LENGTH:
            self.state.crosshair.x = self.state.crosshair.x + SPEED
        return PENALTY
    def move_eye_up(self):
        if self.state.eye.y - SPEED > 0:
            self.state.eye.y = self.state.eye.y - SPEED
        return PENALTY
    def move_eye_down(self):
        if self.state.eye.y + SPEED < LENGTH:
            self.state.eye.y = self.state.eye.y + SPEED
        return PENALTY
    def move_eye_left(self):
        if self.state.eye.x - SPEED > 0:
            self.state.eye.x = self.state.eye.x - SPEED
        return PENALTY
    def move_eye_right(self):
        if self.state.eye.x + SPEED < LENGTH:
            self.state.eye.x = self.state.eye.x + SPEED
        return PENALTY


if __name__ == "__main__":
    '''
    Eventually, this entrypoint will support 3 modes:
        manual:     human-operable play environment to
        skilltree:  skill-tree like learning of the play environment (probably based on manual data scraping)
        highlevel:  Konidaris-esk high-level learning of the environment
    '''
    assert len(sys.argv) == 2, "no mode specified.  (manual, UNSUPPORTED, UNSUPPORTED)"
    mode = str(sys.argv[1])

    if mode == "manual":
        manual.run()
    else:
        print "Unsupported Mode"
