import matplotlib.pyplot as plt
from dublins import step
import numpy as np
import re


class TreePlot:
    """
    Debug code for A-star
    """

    def __init__(self, car, nsteps):
        self.nodes = {}
        self.car = car
        self.nsteps = nsteps

    def addNode(self, n):
        x = [n.parent.x]
        y = [n.parent.y]
        theta = n.parent.theta

        for i in range(self.nsteps):
            xn, yn, theta = step(self.car, x[-1], y[-1], theta, n.control, dt=0.045)
            x.append(xn)
            y.append(yn)

        p, = plt.plot(x, y, c=(0.3, 1, 0.3))
        plt.draw()
        plt.gcf().canvas.flush_events()

        self.nodes[(n.xd, n.yd, n.thetad)] = (p, n)

    def updateNode(self, n, nn):
        p, node = self.nodes[(n.xd, n.yd, n.thetad)]
        del self.nodes[(n.xd, n.yd, n.thetad)]
        self.nodes[(nn.xd, nn.yd, nn.thetad)] = (p, nn)

        x = [nn.parent.x]
        y = [nn.parent.y]
        theta = nn.parent.theta

        for i in range(self.nsteps):
            xn, yn, theta = step(self.car, x[-1], y[-1], theta, nn.control)
            x.append(xn)
            y.append(yn)

        p.set_xdata(x)
        p.set_ydata(y)

        plt.draw()
        plt.gcf().canvas.flush_events()

    def closeNode(self, n):
        p, node = self.nodes[(n.xd, n.yd, n.thetad)]
        p.set_color((1, 0.3, 0.3))

    def markBestPath(self, endNode):
        while endNode.parent is not None:
            p, node = self.nodes[(endNode.xd, endNode.yd, endNode.thetad)]
            p.set_color((0.8, 0, 0.8))
            p.set_linewidth(5)
            endNode = endNode.parent


def plotRegion(car):
    '''Plotta alla hinder, start och slut'''
    ax = plt.gca()
    for ob in car.obs:
        ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor=(0, 0, 0, 0), edgecolor="k", linewidth=2.5))

    plt.scatter([car.x0, car.xt], [car.y0, car.yt], marker="x", c="k")


def plotGrid(car, nx, ny):
    '''Plotta rutnatet. nx och ny ar antalet rutor i
    x- och y-led.'''
    size_x = (car.xub - car.xlb)/nx
    size_y = (car.yub - car.ylb)/ny

    for i in range(nx):
        plt.plot([(i+1)*size_x, (i+1)*size_x], [car.ylb, car.yub], c="b")

    for i in range(ny):
        plt.plot([car.xlb, car.xub], [(i+1)*size_y, (i+1)*size_y], c="b")

def plotObstacleCells(size_x, size_y, cells):
    '''Fyller de celler som ar markerade som hinder. size_x ar
    cellernas bredd och size_y ar cellernas hojd. cells ar en
    2d-lista dar cells[x][y] ar True om cellen ar ett hinder.'''
    for x in range(len(cells)):
        for y in range(len(cells[0])):
            if cells[x][y]:
                plt.gca().add_patch(plt.Rectangle((x*size_x, y*size_y), size_x, size_y, facecolor=(0.5, 0.5, 1)))

def plotMap(car, nx, ny, cells):
    '''Plottar rutnatet for A* med hinderceller markerade. nx
    och ny ar antalet celler i x- och y-led. cells ar en 2d-lista
    dar cells[x][y] ar True om cellen ar ett hinder.'''

    plt.ion()
    obstacleList = []
    f = open("/home/caroline/Documents/svea_starter/src/svea_core/scripts/team4_project/planner/obstacles.txt", "r")

    for line in f:
        lp = []
        reg = re.compile("\[[+-]?[0-9]+.[0-9]+,[ ][+-]?[0-9]+.[0-9]+\]")
        obs_coord = re.findall(reg, line)
        for p in obs_coord:
            o = p.strip("[").strip("]").replace(" ", "")
            o = o.split(",")
            lp.append((o))
        for i in range(0,len(lp)):
            if i == len(lp)-1:
                xc = np.linspace(float(lp[0][0]), float(lp[i][0]))
                yc = np.linspace(float(lp[0][1]), float(lp[i][1]))
                for k in range(0,len(xc)):
                    x = xc[k]
                    y = yc[k]
                    obstacleList.append((x,y,0.1))
            else:
                xc = np.linspace(float(lp[i+1][0]), float(lp[i][0]))
                yc = np.linspace(float(lp[i+1][1]), float(lp[i][1]))
                for k in range(0,len(xc)):
                    x = xc[k]
                    y = yc[k]
                    obstacleList.append((x,y,0.05))
    f.close()

    for (ox, oy, size) in obstacleList:
        plt.plot(ox, oy, "ok", ms=30 * size)

def plotPath(sx, sy, ex, ey):
    plt.plot([sx, ex], [sy, ey], c=(0.3, 1, 0.3))
    plt.draw()
    plt.gcf().canvas.flush_events()

def show(car):
    ax = plt.gca()
    #plt.axis([-23, 23, -18, 18])
    ax.set_xlim(-23,23)
    ax.set_ylim(-18,18)
    #ax.set_xlim(car.xlb, car.xub)
    #ax.set_ylim(car.ylb, car.yub)
    ax.set_aspect("equal")
    plt.show()
    plt.gcf().canvas.flush_events()

def wait():
    plt.ioff()
    plt.show()
