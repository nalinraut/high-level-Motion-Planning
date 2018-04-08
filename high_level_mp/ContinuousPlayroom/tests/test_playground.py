from playground import *

'''
    This file includes unit tests for Gunnar's emulation of the ContinuousPlayroom
    You could can run these tests by typing `nosetests -v test_playground.py` in the terminal.
'''

# -------------------------------------------------------------------------
def overlap_effectors(state, to_overlap):
    """overlaps hand and eye with specified object"""
    state.hand.x = to_overlap.x
    state.hand.y = to_overlap.y
    state.eye.x = to_overlap.x
    state.eye.y = to_overlap.y


def move_to(dest, to_move):
    """moves object to provided (x,y) dest"""
    to_move.x = dest[0]
    to_move.y = dest[1]


# -------------------------------------------------------------------------
def test_python_version():
    """python version should be 2.x"""
    assert sys.version_info[0]==2 # require python 2 (instead of python 3)


def test_red_button():
    """red button should stop music, but only if the light is on, and the hand and eye are over it"""
    state = State()
    agent = Agent(state)

    state.light_on = False
    state.music_playing = True

    assert state.music_playing
    score1 = agent.interact()        # should do nothing
    assert state.music_playing
    overlap_effectors(state, state.red_button)
    score2 = agent.interact()        # should do nothing
    assert state.music_playing
    state.light_on = True
    score3 = agent.interact()        # should turn off music
    assert not state.music_playing

    assert score1 == PENALTY and score2 == PENALTY and score3 == PENALTY    # should receive PENALTY rewards


def test_green_button():
    """green button should start music, but only if the light is on, and the hand and eye are over it"""
    state = State()
    agent = Agent(state)

    state.light_on = False
    state.music_playing = False

    assert not state.music_playing
    score1 = agent.interact()        # should do nothing
    assert not state.music_playing
    overlap_effectors(state, state.green_button)
    score2 = agent.interact()        # should do nothing
    assert not state.music_playing
    state.light_on = True
    score3 = agent.interact()        # should turn on music
    assert state.music_playing

    assert score1 == PENALTY and score2 == PENALTY and score3 == PENALTY    # should receive PENALTY rewards


def test_light_switch():
    """light switch should toggle light, but only if the hand and eye are over it"""
    # test turning light ON
    state = State()
    agent = Agent(state)
    state.light_on = False

    assert not state.light_on
    score1 = agent.interact()       # should do nothing
    assert not state.light_on
    overlap_effectors(state, state.light_switch)
    score2 = agent.interact()       # should turn on light
    assert state.light_on

    assert score1 == PENALTY and score2 == PENALTY    # should receive PENALTY rewards

    # turning light OFF
    state = State()
    agent = Agent(state)
    state.light_on = True

    assert state.light_on
    score1 = agent.interact()       # should do nothing
    assert state.light_on
    overlap_effectors(state, state.light_switch)
    score2 = agent.interact()       # should turn off light
    assert not state.light_on

    assert score1 == PENALTY and score2 == PENALTY    # should receive PENALTY rewards



def test_reward_condition():
    """
    ensure reward is granted on interact when...
        1) music is on
        2) light is on
        3) hand and eye are over ball
        4) target is over bell
    """

    state = State()
    agent = Agent(state)
    state.light_on = False
    state.music_playing = False

    score1 = agent.interact()       # should be PENALTY
    overlap_effectors(state, state.ball)
    score2 = agent.interact()       # should be PENALTY
    state.crosshair.x = state.bell.x
    state.crosshair.y = state.bell.y
    score3 = agent.interact()       # should be PENALTY
    state.light_on = True
    score4 = agent.interact()       # should be PENALTY
    state.music_playing = True
    score5 = agent.interact()       # should be REWARD
    assert state.monkey_cried

    assert score1 == PENALTY and score2 == PENALTY and score3 == PENALTY and score4 == PENALTY
    assert score5 == REWARD


def test_border_jumping():
    """Ensure that movements don't take effectors out of bounds"""
    state = State()
    agent = Agent(state)

    effctrs = [state.crosshair, state.eye, state.hand]
    corners = [(1, 1), (LENGTH-1, 1), (1, LENGTH-1), (LENGTH-1, LENGTH-1)]
    actions = [agent.move_crosshair_up, agent.move_crosshair_right, agent.move_crosshair_left, agent.move_crosshair_down,
               agent.move_eye_up, agent.move_eye_right, agent.move_eye_left, agent.move_eye_down,
               agent.move_hand_up, agent.move_hand_right, agent.move_hand_left, agent.move_hand_down]

    for corner in corners:                  # for every corner,
        for effector in effctrs:            # for every effector,
            for action in actions:          # perform each movement option, and ensure the effector stays in bounds
                move_to(corner, effector)
                action()
                assert (0 < effector.x < LENGTH) and (0 < effector.y < LENGTH)
