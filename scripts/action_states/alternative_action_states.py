import numpy as np
import itertools
import os
import csv

from numpy.lib.function_base import diff

num_nodes = 9

bins = {}
objects = []
object_to_index = {}


def read_objects_and_bins():
    with open('objects.csv', newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
    for x in data:
        objects.append(x)
    
    with open('bins.csv', newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
    
    for x in data:
        bins[x[0]] = int(x[1])


def generate_states():
    ls_nodes = list(range(num_nodes))
    ls_complete = [ls_nodes]

    for i, obj in enumerate(objects):
        object_to_index[str(obj)] = i+1
        ls_complete.append([0,1])
    
    temp = itertools.product(*ls_complete)
    states = list([list(tup) for tup in temp])
    return states

    
def generate_action_matrix(states):
    n = len(states)
    action_matrix = np.ones((n,n), dtype=int) * (-1)
    
    for i, state in enumerate(states):
        for obj in objects:
            index = object_to_index[str(obj)]
            color = obj[1]
            if state[index] == 0:
                nextstate = state.copy()
                nextstate[index] = 1
                nextstate[0] = bins[color]
                state_index = states.index(nextstate)  
                if i != state_index:
                    action_matrix[i, state_index] = index -1 
    return action_matrix

def save_data(data, name):
    """ Saves a list to a csv file using numpy """

    np.savetxt(name, data, fmt='%5s', delimiter = ',')




if __name__=="__main__":
    read_objects_and_bins()

    states = generate_states()
    save_data(states, 'states.csv')

    action_matrix = generate_action_matrix(states)
    save_data(action_matrix, "action_matrix.csv")