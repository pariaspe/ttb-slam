import rospy
import numpy as np


def averageGrid(grid, grid_resol):
    len_x, len_y = np.shape(grid)
    resol_x, resol_y = int(len_x/grid_resol), int(len_y/grid_resol)
    averagedGrid = np.zeros((resol_x,resol_y))
    for i in range(resol_x):
        for j in range(resol_y):
            
            copyMatrix = grid[i*grid_resol:i*grid_resol+grid_resol,j*grid_resol:j*grid_resol+grid_resol].copy()
            
            averagedGrid[int(i)][int(j)] = np.mean(copyMatrix, dtype=np.float64)
                
    return(averagedGrid)

#for script testing
if __name__ == "__main__":
    resol = int(2)
    mapping = np.random.rand(10,10)
    print mapping
    averaged = averageGrid(mapping,resol)
    print averaged