from Tkinter import *
import Tkinter as tk

points = list()
file = open('point.poly', 'w')
file.truncate(0)

exit_flag = False
def close_file():
	global file, exit_flag
	print 'file closed'
	exit_flag = True
	file.close()
	win.destroy()

def initWorld(canvas, x, y):
    global file, win, exit_flag, pixelw, pixelh, world_width, world_height, world_ymax, world_xmax
    radius = 5.
    colors = ["white", "blue", "yellow", "#FAA"]
    canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill = colors[2])
    print 'exit flag', exit_flag
    print 'y_c', - y / pixelh, y, pixelh, world_height, world_xmax

    file.write(str(round(- x / pixelw * world_width + world_ymax, 2)) + ' ' + str(round(- y / pixelh * world_height + world_xmax, 2))+'\n')

def callback(event):
    point = [event.x * world_height, event.y * world_width]
    points.append(point)
    print "clicked at", event.x, event.y
    initWorld(canvas, event.x, event.y)
	


def motion(event):
    x, y = event.x, event.y
    print('{}, {}'.format(x, y))


pixelw = 1366.
pixelh = 768.
framedelay = 1
# +y direction is to left and +x direction is upwards
world_xmin = -60. * 25.4 # mm
world_ymin = -70. * 25.4 # mm
world_xmax = 57. * 25.4 # mm
world_ymax = 76.25 * 25.4 # mm
world_width = world_ymax - world_ymin
world_height = world_xmax - world_xmin
win = Tk()
win.bind("<Button-1>", callback)

canvas = Canvas(win, width = pixelw, height = pixelh)
win.geometry("+{}+{}".format(2500, 100))
canvas.pack()
win.protocol("WM_DELETE_WINDOW", close_file)
win.after(framedelay, lambda: callback)
tk.mainloop()
