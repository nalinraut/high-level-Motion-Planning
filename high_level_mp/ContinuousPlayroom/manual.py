from playground import *
from Tkinter import *
import pyaudio
import wave


class Visualizer:
    """
    A Tkinter-based class for manual play through of the continuous playroom environment.

    Keyboard Controls:
        Arrow keys: Move active effector
        M:          Toggles between active effector
        Space:      Attempts an "interact" action

    Environment Indicators:
        Background Color: White/Gray --> Light On/Off
        TopRight Speaker: On/Off     --> Sound On/Off
    """
    def __init__(self, state, agent):
        self.state = state
        self.agent = agent

        self.canvas = Canvas(width=LENGTH, height=LENGTH, bg='white')
        self.canvas.pack(expand=YES, fill=BOTH)
        self.canvas.focus_set()

        self.effector_mode = "eye" # hand, crosshair

        self.canvas.bind('<Left>', self.go_left)
        self.canvas.bind('<Right>', self.go_right)
        self.canvas.bind('<Up>', self.go_up)
        self.canvas.bind('<Down>', self.go_down)
        self.canvas.bind('<m>', self.toggle_mode)
        self.canvas.bind('<space>', self.interact)

    def render(self):
        # clear objects
        self.canvas.delete("all")

        # handle light
        if self.state.light_on:
            self.canvas.configure(background='white')
        else:
            self.canvas.configure(background='gray')

        # handle sound
        if self.state.music_playing:
            self.tmp = PhotoImage(file='./res/speaker_on.png')
        else:
            self.tmp = PhotoImage(file='./res/speaker_off.png')
        self.canvas.create_image(LENGTH, 0, image=self.tmp, anchor=NE)

        # draw objects
        for item in self.state.items:
            item.img = PhotoImage(file=item.res_path)
            self.canvas.create_image(item.x, item.y, image=item.img, anchor=CENTER)

        # show changes
        self.canvas.update()
        self.canvas.update_idletasks()

    def go_left(self, event):
        to_run = {
            "eye": self.agent.move_eye_left,
            "hand": self.agent.move_hand_left,
            "crosshair": self.agent.move_crosshair_left
        }.get(self.effector_mode)
        to_run()

    def go_right(self, event):
        to_run = {
            "eye": self.agent.move_eye_right,
            "hand": self.agent.move_hand_right,
            "crosshair": self.agent.move_crosshair_right
        }.get(self.effector_mode)
        to_run()

    def go_up(self, event):
        to_run = {
            "eye": self.agent.move_eye_up,
            "hand": self.agent.move_hand_up,
            "crosshair": self.agent.move_crosshair_up
        }.get(self.effector_mode)
        to_run()

    def go_down(self, event):
        to_run = {
            "eye": self.agent.move_eye_down,
            "hand": self.agent.move_hand_down,
            "crosshair": self.agent.move_crosshair_down
        }.get(self.effector_mode)
        to_run()

    def toggle_mode(self, event):
        if self.effector_mode == "eye":
            self.effector_mode = "hand"
        elif self.effector_mode == "hand":
            self.effector_mode = "crosshair"
        elif self.effector_mode == "crosshair":
            self.effector_mode = "eye"

    def interact(self, event):
        reward = self.agent.interact()
        if reward == REWARD:
            print("we done finished!")
            scream()
            self.canvas.destroy()


def scream():
    """A silly, blocking method that actually makes a screaming monkey noise"""
    f = wave.open(r"./res/monkey_screaming.wav", "rb")
    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(f.getsampwidth()),
                    channels=f.getnchannels(),
                    rate=f.getframerate(),
                    output=True)
    data = f.readframes(1024)

    while data:
        stream.write(data)
        data = f.readframes(1024)

    stream.stop_stream()
    stream.close()

    p.terminate()


def run():
    """This is the method called from playground.py when given the "manual" argument"""
    state = State()
    agent = Agent(state)
    visualizer = Visualizer(state, agent)

    while True:
        visualizer.render()

