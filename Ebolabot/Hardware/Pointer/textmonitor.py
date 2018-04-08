from Tkinter import Tk, Text, INSERT, END

class monitor(object):
    def __init__(self, title):
        self.root = Tk()
        self.root.wm_title(title)
        self.text = Text(self.root)
        self.text.pack()
        self.root.update()

    def write(self, msg):
        self.text.insert(END, msg)
        self.text.see(END)
        self.root.update()

    def writeln(self, msg):
        self.text.insert(END, str(msg) + '\n')
        self.text.see(END)
        self.root.update()

    def update(self):
        self.root.update()

    def close(self):
       self.root.destroy 
