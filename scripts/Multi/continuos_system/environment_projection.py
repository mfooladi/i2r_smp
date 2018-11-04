from Tkinter import *
from PIL import Image, ImageDraw
# this is to test the top-down projector - calibrating the projection area
win = Tk()
win.geometry("{}x{}+{}+{}".format(1366, 768, 1000, 0))
win.wm_attributes('-fullscreen', True)
screen_x = 768.
screen_y = 1024.
quad1 = PhotoImage(file='background.gif')
quad1 = quad1.subsample(2, 2)

# frame = Frame(win, width=screen_y, height=screen_x, bg='#ffff00')
canvas = Canvas(win, width=screen_y, height=screen_x, background="#ffffff")
# canvas.create_oval(screen_y/2-2, screen_x/2-2, screen_y/2 + 2, screen_x/2 + 2, fill='black')
# canvas.create_oval(screen_y/2-20, screen_x/2-20, screen_y/2 + 20, screen_x/2 + 20)
# canvas.create_oval(screen_y/2-100, screen_x/2-100, screen_y/2 + 100, screen_x/2 + 100)
# canvas.create_line(0, 0, screen_y, screen_x)
# canvas.create_line(0, screen_x, screen_y, 0)
# canvas.create_line(screen_y/2, screen_x, screen_y/2, 0)
# canvas.create_line(0, screen_x/2, screen_y, screen_x/2)
# canvas.create_line(0, screen_x, screen_y, 0)
canvas.create_image(510, 384, image=quad1)
canvas.pack()
mainloop()