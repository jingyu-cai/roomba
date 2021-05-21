import numpy as np
import itertools
import os

from numpy.lib.function_base import diff


# Initialize how many bins, dumbbells, balls, and obstacles there are, and
#   right now we are assuming that there will be exactly 1 colored object of
#   its kind, so there won't be 2 red dumbbells
# Note: index 0 = red, index 1 = blue, so BINS means there is 1 red bin 
#   and 1 blue bin
BINS = [1, 1]
DUMBBELLS = [1, 1]
BALLS = [1, 1]
OBSTACLES = [1, 1]

# TODO (optional): Is there a way to distinguish, for ex, 2 red dumbbells? 


def append_object_states(all_lists, objects):
    """ A helper function to help append all object states permutations to a
    list in generate_states """

    for i in range(sum(objects)):
        for j in range(objects[i]):
            all_lists.append([0, i + 1])


def generate_states(bins: list, dumbbells: list, balls: list, obstacles: list):
    """ This generates a matrix of states based on the distribution of objects """

    # Note: 0 = starting point/origin, 1 = red bin, 2 = blue bin

    # Create a list to save all the states below for permutation calculation,
    #   this list would contain robot states, dumbbell states for each dumbbell,
    #   ball states for each ball, obstacle states for each obstacle, in that order
    all_lists = []

    # Calculate the robot states based on origin + number of bins and append
    robot_states = [(i + 1) for i in range(len(bins))]
    robot_states.insert(0, 0)
    all_lists.append(robot_states)

    # Calculate the dumbbell, ball, and obstacle states and append
    append_object_states(all_lists, dumbbells)
    append_object_states(all_lists, balls)
    append_object_states(all_lists, obstacles)

    # Finally, we run a permutations algorithm on all_lists
    states = list(itertools.product(*all_lists))

    return states


def generate_actions(bins: list, dumbbells: list, balls: list, obstacles: list):
    """ This generates a matrix of actions based on the distribution of objects, and
    we always assume that the correct object would move to the correctly-colored bin """

    # Note: for column 1, 0 = dumbbell, 1 = ball, 2 = obstacle; for column 2,
    #   1 = red, 2 = blue; for column 3, 1 = red bin, 12= blue bin

    # Create a list to save all the actions below for permutation calculation,
    #   this list would contain the types of objects, the colors of the objects,
    #   and the different bins
    all_lists = []

    # Append the types of objects, which are pre-determined
    all_lists.append([0, 1, 2])

    # Find the minimum number of colors between bins, dumbbells, balls, and
    #   obstacles to ensure that we won't have an addition color that cannot
    #   be sorted (we can assume this since the colors will always be created
    #   in a red -> blue -> sth else order)
    num_of_colors = min(len(bins), len(dumbbells), len(balls), len(obstacles))

    # Append the types of object colors for the objects
    all_lists.append(np.arange(1, num_of_colors + 1))

    # We run a permutations algorithm on all_lists
    temp_actions = list(itertools.product(*all_lists))

    # Finally, we add a third column for bins, which are determined by the second
    #   column of the color of the object
    actions = []
    for action in temp_actions:
        action = list(action)
        action.append(action[1])
        actions.append(action)

    return actions


def is_single_difference(state_1, state_2):
    """ A helper function to check if there is exactly 1 difference between 2 
    states, and is used in generate_action_matrix to invalidate state transitions
    that have more than 1 difference """

    diff_counter = 0

    for i in range(len(state_1)):
        if state_1[i] != state_2[i]:
            diff_counter += 1

    return diff_counter == 1


def find_state_difference(state_1, state_2):
    """ A helper function that finds the index of difference between two states,
    which can be mapped onto the actions matrix; this assumes that there is
    indeed exactly 1 difference between the two """

    index_tracker = 0

    for i in range(len(state_1)):
        if state_1[i] != state_2[i]:
            index_tracker = i

    return index_tracker


def generate_action_matrix(states):
    """ This generates the action matrix based on states and actions """

    action_matrix = []

    for i in range(len(states)):

        action_matrix_row = []

        for j in range(len(states)):

            # You cannot transition between the same state
            if i == j:
                action_matrix_row.append(-1)

            # You cannot transition between two states that have more than 
            #   one difference
            elif not is_single_difference(states[i], states[j]):
                action_matrix_row.append(-1)

            # Use actions to find the index to transition the state if there's
            #   exactly one difference between the two
            else:
                state_index = find_state_difference(states[i], states[j])

                # Check if the transition is forward from first state to the second; 
                #   forward means that you can only move an object from the origin 
                #   to a bin, but not the reverse
                if states[i][state_index] < states[j][state_index]:
                    action_matrix_row.append(state_index - 1)
                else:
                    action_matrix_row.append(-1)
        
        action_matrix.append(action_matrix_row)
    
    return action_matrix


def save_data(data, name):
    """ Saves a list to a csv file using numpy """

    np.savetxt(name, data, fmt='%5s', delimiter = ',')


if __name__=="__main__":

    # Create and save the state space
    states = generate_states(BINS, DUMBBELLS, BALLS, OBSTACLES)
    save_data(states, "states.csv")

    # Create and save the action space
    actions = generate_actions(BINS, DUMBBELLS, BALLS, OBSTACLES)
    save_data(actions, "actions.csv")

    # Create and save the action matrix
    action_matrix = generate_action_matrix(states)
    save_data(action_matrix, "action_matrix.csv")
    