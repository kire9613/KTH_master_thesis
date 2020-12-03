# import matplotlib.pyplot as plt
import numpy as np

def plot_map(occ_map,start,end,path,obstacles):

    occ_map[start[1],start[0]] = 50 # BLUE
    occ_map[end[1],end[0]] = 50 # BLUE

    try:
        for coord in path:
            occ_map[coord[0],coord[1]] = 50 # BLUE
    except:
        print("Path is not given")
    try:
        for coord in obstacles:
            occ_map[coord[0],coord[1]] = 100 # BLUE
    except:
        print("Obstacles are not given")
    plt.imshow(occ_map)
    plt.colorbar()
    plt.show()
