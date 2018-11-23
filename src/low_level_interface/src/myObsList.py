from numpy import *
import json
import copy
#fname = "line_obs.json"


def myObsList():
    fname = "my_obs.json"
    with open(fname) as f:
        obstacle_list = json.load(f)
    obs_lists = obstacle_list[0]
    for i in range(len(obs_lists)):
        if obs_lists[i][1] == 2.5:
            obs_lists[i][0] = obs_lists[i][0] - 6
        if obs_lists[i][0] == 15:
            obs_lists[i][0] = obs_lists[i][0] - 6
    copy1 = copy.deepcopy(obs_lists)
    for i in range(len(copy1)):
        if copy1[i][1] == 2.5:
            copy1[i][0] = copy1[i][0] + 13
        if copy1[i][0] == 9:
            copy1[i][0] = copy1[i][0] + 6
        obs_lists.append(copy1[i])

    return obs_lists


def sigmoid(x):
    return 1/(1 + exp(x))
