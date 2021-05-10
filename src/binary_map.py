import rospy
import numpy as np

#define grid_resol as 1m so we can match the initial maps


def binaryGrid(grid, grid_resol):
    len_x, len_y = np.shape(grid)
    resol_x, resol_y = int(len_x/grid_resol), int(len_y/grid_resol)
    binaryGrid = np.zeros((resol_x,resol_y))
    for i in range(resol_x):
        for j in range(resol_y):
            
            copyMatrix = grid[i*grid_resol:i*grid_resol+grid_resol,j*grid_resol:j*grid_resol+grid_resol].copy()
            np.where(copyMatrix == -1, 100, copyMatrix)
            #np.where(copyMatrix >= 50, 100, copyMatrix)
            #np.where(copyMatrix < 50, 0, copyMatrix)
            copyMatrix = copyMatrix/100
            binaryGrid[int(i)][int(j)] = int(round(np.mean(copyMatrix, dtype=np.float64),0))
                
    return(binaryGrid)