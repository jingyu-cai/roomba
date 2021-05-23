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
                if i != j:
                    distance[i,j] = min(distance[i,j], distance[i,k] + distance[k,j])

    return distance



if __name__=="__main__":
    weight_mat = np.genfromtxt('./map1_matrix.csv', delimiter=',')
    for i in range(weight_mat.shape[0]):
        for j in range(weight_mat.shape[1]):
            if weight_mat[i,j] == 0:
                weight_mat[i,j] = 1e9
    print(weight_mat)
    distance = floyd_warshall(weight_mat.shape[0], weight_mat)
    print(distance)
    np.save('distances.npy', distance)