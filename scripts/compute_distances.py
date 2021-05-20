import numpy as np

'''
returns the min distances between each node 
n is the number of nodes
weight_mat is the adjacency mat of the graph
'''
def floyd_warshall(n, weight_mat):
    distance = np.zeros((n,n), dtype = int)

    for i in range(n):
        for j in range(n):
            distance[i,j] = weight_mat[i,j]

    for k in range(n):
        for i in range(n):
            for j in range(n):
                distance[i,j] = min(distance[i,j], distance[i,k] + distance[k,j])

    return distance



if __name__=="__main__":
    pass