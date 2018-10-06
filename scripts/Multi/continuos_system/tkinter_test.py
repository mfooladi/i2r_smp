import Tkinter as tk
from Tkinter import *

root = Tk()

def quit(root):
    root.destroy()

tk.Button(root, text="Quit", command=lambda root=root:quit(root)).pack()
root.mainloop()