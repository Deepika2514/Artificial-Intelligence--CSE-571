#!/usr/bin/env python

import numpy as np
import collections
import heapq
import problem
import rospy
from std_msgs.msg import String
import argparse
import itertools
import time
from datetime import datetime
from timeit import default_timer as timer
from std_srvs.srv import Empty
import subprocess

rospy.init_node('Q_Learning')
# publisher = rospy.Publisher("/actions",String,queue_size =10)
publisher = rospy.Publisher("/results", String, queue_size=10)
# subscriber=rospy.Subscriber("/status",String,callback_status)
parser = argparse.ArgumentParser()
parser.add_argument('-task', help="Please mention Task to use. Default is task_2", metavar='t2', action='store',
                    dest='task', default="t2", type=str)
parser.add_argument('-ep', help="Please mention how many epsiodes you want to run. Default is 5", metavar=500,
                    action='store', dest='episodes', default=5, type=int)

q_initial = {}
gamma = 0.9
alpha = 0.3
actions = []
# global table_store_items
# global table_turtlebot
# global total_rows
# global columns
import json


def getQValue(state, action):
    # print("Inside getQValue")
    return q_initial.get((state, action), 0.0)


def learnQValue(state, action, reward, Actualvalue):
    # print("Inside learnQValue")
    oldQValue = q_initial.get((state, action), None)
    if oldQValue is None:
        q_initial[(state, action)] = reward
        # print("Initial Q value",state,action,q_initial[(state,action)])
    else:
        q_initial[(state, action)] = oldQValue + alpha * (Actualvalue - oldQValue)
        # print("Final Q value",state,action,q_initial[(state,action)])


def getValue(state):
    # print("Inside getVaue")
    maxQnew = max([getQValue(state, a) for a in actions])
    if problem.is_terminal_state():
        return 0
    return maxQnew


def chooseAction(state, epsilon):
    # Pick Action
    # print("Inside chooseAction")
    exploration = np.random.random()
    # print("epsilon:",epsilon)
    if exploration < epsilon:
        chosen_action = np.random.choice(actions)
    else:
        # print("Inside getPolicyAction")
        action_value_dict = {action: getQValue(state, action) for action in actions}
        chosen_action = sorted(action_value_dict, key=lambda x: action_value_dict[x], reverse=True)[0]

    return chosen_action


def update(state1, action1, reward, state2):
    # print("Inside update")
    maxQnew = getValue(state2)
    learnQValue(state1, action1, reward, reward + gamma * maxQnew)
    Q_value = getQValue(state1, action1)
    seq = " ( " + state1 + " ; " + action1 + " ; " + state2 + " ; " + str(reward) + " ) " + " : " + str(Q_value)
    publisher.publish(String(data=seq))


def Table_turtlebot(state):
    global books_no
    global trolly_no
    # print("inside turtlebot")
    state = state.replace("(", "")
    state = state.replace(")", "")
    columns = 4
    books_no = state.count('book')  # rows
    trolly_no = state.count('trolly')
    table_turtlebot = [[0 for x in range(columns)] for y in range(1)]
    state = state.split(';')
    # storing turtlebot location in one matrix
    for i in range(1):
        state[-1] = state[-1].split(',')
        for j in range(columns - 1):
            table_turtlebot[i][j] = state[-1][j]
    return table_turtlebot


def find_legal_actions():
    global table_turtlebot
    global total_rows
    global columns
    global actions
    global obj_dict
    global books_no
    global trolly_no

    book_list = []
    trolly_list = []
    x = np.float64(table_turtlebot[0][1])
    y = np.float64(table_turtlebot[0][2])
    # print(x)
    # print(y)
    for i in range(books_no):
        book_list.append("book_" + str(i + 1))
        # print(book_list)
    for i in range(trolly_no):
        trolly_list.append("trolly_" + str(i + 1))
        # print(trolly_list)

    for bin_name in trolly_list:
        # print("bin")
        # print(obj_dict["bins"][bin_name]["load_loc"])
        if [x, y] in obj_dict["bins"][bin_name]["load_loc"]:
            legal_actions = [action for action in actions if "place" in action]
            # print("legal actions for placing in trolly:",legal_actions)
            return legal_actions
    for book_name in book_list:
        if [x, y] in obj_dict["books"][book_name]["load_loc"]:
            legal_actions = [action for action in actions if "pick" in action]
            # print("legal actions for picking book:",legal_actions)
            return legal_actions
    legal_actions = [action for action in actions if "move" in action or "Turn" in action]
    # print("legal actions for move or turn actions",legal_actions)
    return legal_actions


def chooseAction_taskl(state, epsilon):
    # Pick Action
    global table_turtlebot
    # print("Inside chooseAction_task3")
    exploration = np.random.random()
    # print("epsilon:",epsilon)
    table_turtlebot = Table_turtlebot(state)
    legal_actions = find_legal_actions()

    if exploration < epsilon:
        chosen_action = np.random.choice(legal_actions)
    else:
        # print("Inside getPolicyAction")
        action_value_dict = {action: getQValue(state, action) for action in legal_actions}
        chosen_action = sorted(action_value_dict, key=lambda x: action_value_dict[x], reverse=True)[0]

    return chosen_action


def action_to_execute_func(action):
    # print("Action is",action)
    if "careful_pick" in action:
        action_split = action.split(" ")
        observation, current_state, reward = problem.execute_careful_pick(action_split[1])
        return observation, current_state, reward
    elif "normal_pick" in action:
        action_split = action.split(" ")
        observation, current_state, reward = problem.execute_normal_pick(action_split[1])
        return observation, current_state, reward
    elif "careful_place" in action:
        action_split = action.split(" ")
        observation, current_state, reward = problem.execute_careful_place(action_split[1], action_split[2])
        return observation, current_state, reward
    elif "normal_place" in action:
        action_split = action.split(" ")
        observation, current_state, reward = problem.execute_normal_place(action_split[1], action_split[2])
        return observation, current_state, reward
    elif "normal_moveF" in action:
        observation, current_state, reward = problem.execute_normal_moveF()
        return observation, current_state, reward
    elif "careful_moveF" in action:
        observation, current_state, reward = problem.execute_careful_moveF()
        return observation, current_state, reward
    elif "normal_TurnCW" in action:
        observation, current_state, reward = problem.execute_normal_TurnCW()
        return observation, current_state, reward
    elif "normal_TurnCCW" in action:
        observation, current_state, reward = problem.execute_normal_TurnCCW()
        return observation, current_state, reward
    elif "careful_TurnCW" in action:
        observation, current_state, reward = problem.execute_careful_TurnCW()
        return observation, current_state, reward
    else:
        observation, current_state, reward = problem.execute_careful_TurnCCW()
        return observation, current_state, reward


def t1(episodes):
    print("Inside task_1")


def t2(total_episodes):
    # print("Inside task_2")
    epsilon = 0.9
    epsilon_discount = 0.99
    start_time = time.time()
    # total_episodes = 2
    highest_reward = 0
    last_time_steps = np.ndarray(0)

    for x in range(total_episodes):
        # print("Inside x")
        done = False
        cumulated_reward = 0
        state_init = problem.get_current_state()
        # print("Initial state is ",state_init)
        # print("data type",type(state_init))
        if epsilon > 0.05:
            epsilon *= epsilon_discount
        for i in range(70000):
            # print("Inside i")
            # Pick an action based on the current state
            action = chooseAction(state_init, epsilon)
            # print("Action in x :",action)
            # Execute the action and get feedback
            observation, next_state, reward = action_to_execute_func(action)
            # print("Next state is",next_state)
            # print("reward is:",reward)
            cumulated_reward += reward

            update(state_init, action, reward, next_state)
            term = problem.is_terminal_state()
            # print("term:",term)
            if problem.is_terminal_state() == 0:
                state_init = next_state
            else:
                last_time_steps = np.append(last_time_steps, [int(i + 1)])
                # print("last time steps:",last_time_steps)
                # print("Terminal state is reached in",i)
                break
        print("Episode ", x + 1, "Reward :", cumulated_reward)
        problem.reset_world()
        # print("reset:",reset)
        # if reset == 1:
        #    print("Reset happened")
        # print("New state is:",problem.get_current_state())


def t3(episodes):
    # print("Inside task_3")
    global obj_dict
    global table_turtlebot
    epsilon = 0.9
    epsilon_discount = 0.99
    start_time = time.time()
    # total_episodes = 2
    highest_reward = 0
    last_time_steps = np.ndarray(0)
    # root_path="home/deepika/catkin_ws/src/reinforcement"
    # filename_1=root_path+"/books.json"
    # print(filename_1)
    # with open(filename_1) as file:
    #    obj_dict = json.load(file)
    state_init = problem.get_current_state()

    table_turtlebot = Table_turtlebot(state_init)

    for x in range(episodes):
        # print("Inside x")
        done = False
        cumulated_reward = 0
        state_init = problem.get_current_state()
        # print("Initial state is ",state_init)
        # print("data type",type(state_init))
        # table_turtlebot= Table_turtlebot(state_init)
        if epsilon > 0.05:
            epsilon *= epsilon_discount
        for i in range(20000):
            # print("Inside i")
            # Pick an action based on the current state
            action = chooseAction_taskl(state_init, epsilon)
            # print("Action in x :",action)
            # Execute the action and get feedback
            observation, next_state, reward = action_to_execute_func(action)
            # print("Observation:",observation)
            # print("Next state is",next_state)
            # print("reward is:",reward)
            cumulated_reward += reward
            update(state_init, action, reward, next_state)
            # term=problem.is_terminal_state()
            # print("term:",term)
            if problem.is_terminal_state() == 0:
                state_init = next_state
            else:
                last_time_steps = np.append(last_time_steps, [int(i + 1)])
                # print("last time steps:",last_time_steps)
                # print("Terminal state is reached in",i)
                break
        print("Episode ", x + 1, "Reward :", cumulated_reward)
        problem.reset_world()


if __name__ == "__main__":
    global table_store_items
    global table_turtlebot
    global total_rows
    global columns
    global books_no
    global trolly_no
    global obj_dict
    args = parser.parse_args()
    task = globals().get(args.task)
    print(task)
    episodes = args.episodes
    actions = problem.get_all_actions()
    root_path = "/home/deepika/catkin_ws/src/reinforcement"
    filename = root_path + "/books.json"
    print(filename)
    with open(filename) as file:
        obj_dict = json.load(file)
    print(episodes)
    if task is None:
        print("Incorrect task name.")
        exit(1)
    else:
        # print("Task is",task)
        task(episodes)
