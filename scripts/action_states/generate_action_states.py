import numpy as np
import itertools
import os


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
    i = 0
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
    #   0 = red, 1 = blue; for column 3, 0 = red bin, 1 = blue bin

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
    all_lists.append(np.arange(num_of_colors))

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


def generate_action_matrix():
    """ This generates the action matrix based on states and actions """

    # TODO


def save_data(data, name):
    """ Saves a list to a csv file using numpy """

    np.savetxt(name, data, fmt='%5s', delimiter = ',')


if __name__=="__main__":

    # Create and save the states matrix
    states = generate_states(BINS, DUMBBELLS, BALLS, OBSTACLES)
    save_data(states, "states.csv")

    # Create and save the action matrix
    actions = generate_actions(BINS, DUMBBELLS, BALLS, OBSTACLES)
    save_data(actions, "actions.csv")
    