#!/usr/bin/env python

import collections
import heapq
import problem
import rospy
from std_msgs.msg import String
import argparse
import itertools
from datetime import datetime
from timeit import default_timer as timer

rospy.init_node("Algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Default is BFS", metavar='bfs', action='store',
                    dest='algorithm', default="bfs", type=str)


def bfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []  # List with moves which takes the bot from start to goal
    frontier = collections.deque([init_state])
    explored = []
    child = []  # To store all [next_state,action,current_state] lists
    flag = 0
    # to get the next state, cost for an action on state_x use:
    # (nextstate, cost) = problem.get_successor(state, action)
    # Comparing the initial state with goal state

    if (init_state == goal_state):
        print("Already present in goal")
        return

    while frontier:
        if (flag == 1):
            break
        current_state = frontier.popleft()
        explored.append(current_state)
        for action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if nextstate not in frontier and nextstate not in explored and nextstate != -1:
                if nextstate == goal_state:
                    child.append([nextstate, action, current_state])  # This action takes the bot to the goal
                    flag = 1
                    break
                else:
                    child.append([nextstate, action, current_state])
                    frontier.append(nextstate)

    # A list of all the moves which gives (next_state,action,current_state)
    # In Last list next_node is the goal node
    # Retrace till we reach the Starting node (while loop)
    # Problem -- number of elements in the child list depends on the map
    # BFS gaurantees each node is visited only once

    present_node = child[-1][0]  # Goal node
    action_list.append(child[-1][1])
    previous_node = child[-1][2]
    current_child = []

    while (previous_node is not init_state):  # iterate till the Starting node is not reached
        # List comprehension
        current_child = [child_elem for child_elem in child if child_elem[0] == previous_node]
        action_list.append(current_child[0][1])
        previous_node = current_child[0][2]
        child.remove(current_child[0])  # to speed up the search

    # reversing the action list
    action_list.reverse()
    return action_list


def ucs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []  # List with moves which takes the bot from start to goal
    frontier = []
    explored = []
    child = []  # To store EXPLORED Nodes [next_state,action,current_state] lists
    cost = 0
    counter = itertools.count()  # unique sequence count
    count = next(counter)
    heapq.heappush(frontier, [cost, count, init_state])  # (cost,counter, state)
    entry_finder = []  # list with nodes that are in frontier (copy)
    entry_finder.append(init_state)
    # to get the next state, cost for an action on state_x use:

    # (nextstate, cost) = problem.get_successor(state, action)

    while frontier:
        current_cost, current_order, current_state = heapq.heappop(frontier)
        entry_finder.remove(current_state)

        # Algo Exit section
        if (current_state == goal_state):
            # return the solution here
            # child list last entry is the direction to goal node
            goal_child = [child_elem for child_elem in child if child_elem[0] == goal_state]
            present_node = goal_child[0][0]  # Goal node
            action_list.append(goal_child[0][1])
            previous_node = goal_child[0][2]
            current_child = []

            while (previous_node is not init_state):  # iterate till the Starting node is not reached
                # List comprehension
                current_child = [child_elem for child_elem in child if child_elem[0] == previous_node]
                action_list.append(current_child[0][1])
                previous_node = current_child[0][2]
                child.remove(current_child[0])  # to speed up the search
            # Exit from the other while loop of frontier
            # child <- nextstate,action,current_state
            break

        explored.append(current_state)
        for action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if nextstate not in entry_finder and nextstate not in explored and nextstate != -1:
                child.append([nextstate, action, current_state])
                count = next(counter)
                # get_successor functions gives cost btw current state and next state
                heapq.heappush(frontier, [current_cost + cost, count, nextstate])  # pay attention to list
                entry_finder.append(nextstate)
            elif nextstate in entry_finder:
                # existing_node_fronteir is a list of list
                existing_node_frontier = [node for node in frontier if node[2] == nextstate]
                if existing_node_frontier[0][0] > current_cost + cost:
                    count = next(counter)
                    frontier.remove(existing_node_frontier[0])
                    frontier.append([current_cost + cost, count, nextstate])
                    # nextstate will be present in child list as well, updating it
                    current_child = [child_elem for child_elem in child if child_elem[0] == nextstate]
                    # remove it from child list
                    child.remove(current_child[0])
                    # Making an updated entry
                    child.append([nextstate, action, current_state])

    # reversing the action list
    action_list.reverse()
    return action_list


def gbfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []

    # to get the next state, cost for an action on state_x use:
    # (nextstate, cost) = problem.get_successor(state, action)
    frontier = []
    explored = []
    child = []  # To store EXPLORED Nodes [next_state,action,current_state] lists
    heuristic_cost_init = (abs(goal_state.x - init_state.x) + abs(goal_state.y - init_state.y))
    counter = itertools.count()  # unique sequence count
    count = next(counter)
    flag = 0
    heapq.heappush(frontier, [heuristic_cost_init, count, init_state])  # (heuristic,counter, state)
    entry_finder = []  # list with nodes that are in frontier (copy)
    entry_finder.append(init_state)
    # to get the next state, cost for an action on state_x use:
    # (nextstate, cost) = problem.get_successor(state, action)
    while frontier:
        if (flag == 1):
            break
        current_heuristic_cost, current_order, current_state = heapq.heappop(frontier)
        # print("heuristic cost:",current_heuristic_cost)
        entry_finder.remove(current_state)
        explored.append(current_state)
        for action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if nextstate not in entry_finder and nextstate not in explored and nextstate != -1:
                if (nextstate == goal_state):
                    child.append([nextstate, action, current_state])  # This action takes the bot to the goal
                    flag = 1
                    break
                else:
                    count = next(counter)
                    child.append([nextstate, action, current_state])
                    heuristic_cost = (abs(goal_state.x - nextstate.x) + abs(goal_state.y - nextstate.y))
                    heapq.heappush(frontier, [heuristic_cost, count, nextstate])  # pay attention to list
                    entry_finder.append(nextstate)

    present_node = child[-1][0]  # Goal node
    action_list.append(child[-1][1])
    previous_node = child[-1][2]
    current_child = []

    while (previous_node is not init_state):  # iterate till the Starting node is not reached
        # List comprehension
        current_child = [child_elem for child_elem in child if child_elem[0] == previous_node]
        action_list.append(current_child[0][1])
        previous_node = current_child[0][2]
    # child.remove(current_child) #to speed up the search
    # reversing the action list
    action_list.reverse()

    return action_list


def astar():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []  # List with moves which takes the bot from start to goal
    frontier = []
    explored = []
    child = []  # To store EXPLORED Nodes [next_state,action,current_state] lists
    cost = 0
    counter = itertools.count()  # unique sequence count
    count = next(counter)
    heuristic_cost_init = (abs(goal_state.x - init_state.x) + abs(goal_state.y - init_state.y))
    heapq.heappush(frontier, [cost + heuristic_cost_init, count, init_state])  # (cost=cost+heuristic,counter, state)
    entry_finder = []  # list with nodes that are in frontier (copy)
    entry_finder.append(init_state)

    # to get the next state, cost for an action on state_x use:
    # (nextstate, cost) = problem.get_successor(state, action)
    while frontier:
        current_cost, current_order, current_state = heapq.heappop(frontier)
        entry_finder.remove(current_state)
        heuristic_current_state = (abs(goal_state.x - current_state.x) + abs(goal_state.y - current_state.y))
        current_cost = current_cost - heuristic_current_state
        # Algo Exit section
        if (current_state == goal_state):
            # print("A* exit condition reached\n")
            # return the solution here
            # child list last entry is the direction to goal node
            goal_child = [child_elem for child_elem in child if child_elem[0] == goal_state]
            present_node = goal_child[0][0]  # Goal node
            action_list.append(goal_child[0][1])
            previous_node = goal_child[0][2]
            current_child = []
            while (previous_node is not init_state):  # iterate till the Starting node is not reached
                # List comprehension
                current_child = [child_elem for child_elem in child if child_elem[0] == previous_node]
                action_list.append(current_child[0][1])
                previous_node = current_child[0][2]
                child.remove(current_child[0])  # to speed up the search
                # Exit from the other while loop of frontier
                # child <- nextstate,action,current_state
            break
        explored.append(current_state)
        # print('Explored: ',current_state)
        for action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if nextstate not in entry_finder and nextstate not in explored and nextstate != -1:
                child.append([nextstate, action, current_state])
                count = next(counter)
                # heuristic for next state
                heuristic_cost_next_state = (abs(goal_state.x - nextstate.x) + abs(goal_state.y - nextstate.y))
                # get_successor functions gives cost btw current state and next state
                heapq.heappush(frontier, [current_cost + cost + heuristic_cost_next_state, count,
                                          nextstate])  # pay attention to list# I have to substract heuristic from current_cost
                entry_finder.append(nextstate)
            elif nextstate in entry_finder:
                # existing_node_fronteir is a list of list
                existing_node_frontier = [node for node in frontier if node[2] == nextstate]
                if existing_node_frontier[0][0] > current_cost + cost + heuristic_cost_next_state:
                    count = next(counter)
                    frontier.remove(existing_node_frontier[0])
                    frontier.append([current_cost + cost + heuristic_cost_next_state, count, nextstate])
                    # nextstate will be present in child list as well, updating it
                    current_child = [child_elem for child_elem in child if child_elem[0] == nextstate]
                    # remove it from child list
                    child.remove(current_child[0])
                    # Making an updated entry
                    child.append([nextstate, action, current_state])

    # reversing the action list
    action_list.reverse()

    return action_list


def gbfshs():
    init_state = problem.get_initial_state()
    # print(init_state.x,init_state.y)
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    frontier = []
    explored = []
    child = []  # To store EXPLORED Nodes [next_state,action,current_state] lists
    heuristic_cost_init = ((goal_state.x - init_state.x) ** 2 + (
                goal_state.y - init_state.y) ** 2) ** 0.5  # Euclidean distance
    counter = itertools.count()  # unique sequence count
    count = next(counter)
    heapq.heappush(frontier, [heuristic_cost_init, count, init_state])  # (heuristic,counter, state)
    entry_finder = []  # list with nodes that are in frontier (copy)
    entry_finder.append(init_state)
    flag = 0
    # to get the next state, cost for an action on state_x use:
    # (nextstate, cost) = problem.get_successor(state, action)
    while frontier:
        if (flag == 1):
            break
        current_heuristic_cost, current_order, current_state = heapq.heappop(frontier)
        entry_finder.remove(current_state)
        explored.append(current_state)
        for action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if nextstate not in entry_finder and nextstate not in explored and nextstate != -1:
                if (nextstate == goal_state):
                    child.append([nextstate, action, current_state])  # This action takes the bot to the goal
                    flag = 1
                    break
                else:
                    count = next(counter)
                    child.append([nextstate, action, current_state])
                    heuristic_cost = ((goal_state.x - nextstate.x) ** 2 + (goal_state.y - nextstate.y) ** 2) ** 0.5
                    heapq.heappush(frontier, [heuristic_cost, count, nextstate])  # pay attention to list
                    entry_finder.append(nextstate)

    present_node = child[-1][0]  # Goal node
    action_list.append(child[-1][1])
    previous_node = child[-1][2]
    current_child = []

    while (previous_node is not init_state):  # iterate till the Starting node is not reached
        # List comprehension
        current_child = [child_elem for child_elem in child if child_elem[0] == previous_node]
        action_list.append(current_child[0][1])
        previous_node = current_child[0][2]
        # child.remove(current_child) #to speed up the search
        # reversing the action list

    action_list.reverse()
    return action_list


def astarhs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []  # List with moves which takes the bot from start to goal
    frontier = []
    explored = []
    child = []  # To store EXPLORED Nodes [next_state,action,current_state] lists
    cost = 0
    counter = itertools.count()  # unique sequence count
    count = next(counter)
    heuristic_cost_init = ((goal_state.x - init_state.x) ** 2 + (goal_state.y - init_state.y) ** 2) ** 0.5  # Euclidean
    heapq.heappush(frontier, [cost + heuristic_cost_init, count, init_state])  # (cost=cost+heuristic,counter, state)
    entry_finder = []  # list with nodes that are in frontier (copy)
    entry_finder.append(init_state)

    # to get the next state, cost for an action on state_x use:
    # (nextstate, cost) = problem.get_successor(state, action)
    while frontier:
        current_cost, current_order, current_state = heapq.heappop(frontier)
        entry_finder.remove(current_state)
        heuristic_current_state = ((goal_state.x - current_state.x) ** 2 + (goal_state.y - current_state.y) ** 2) ** 0.5
        current_cost = current_cost - heuristic_current_state
        # Algo Exit section
        if (current_state == goal_state):
            # print("A* exit condition reached\n")
            # return the solution here
            # child list last entry is the direction to goal node
            goal_child = [child_elem for child_elem in child if child_elem[0] == goal_state]
            present_node = goal_child[0][0]  # Goal node
            action_list.append(goal_child[0][1])
            previous_node = goal_child[0][2]
            current_child = []
            while (previous_node is not init_state):  # iterate till the Starting node is not reached
                # List comprehension
                current_child = [child_elem for child_elem in child if child_elem[0] == previous_node]
                action_list.append(current_child[0][1])
                previous_node = current_child[0][2]
                child.remove(current_child[0])  # to speed up the search
                # Exit from the other while loop of frontier
                # child <- nextstate,action,current_state
            break
        explored.append(current_state)
        # print('Explored: ',current_state)
        for action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, action)
            if nextstate not in entry_finder and nextstate not in explored and nextstate != -1:
                child.append([nextstate, action, current_state])
                count = next(counter)
                # heuristic for next state
                heuristic_cost_next_state = ((goal_state.x - nextstate.x) ** 2 + (
                            goal_state.y - nextstate.y) ** 2) ** 0.5
                # get_successor functions gives cost btw current state and next state
                heapq.heappush(frontier, [current_cost + cost + heuristic_cost_next_state, count,
                                          nextstate])  # pay attention to list# I have to substract heuristic from current_cost
                entry_finder.append(nextstate)
            elif nextstate in entry_finder:
                # existing_node_fronteir is a list of list
                existing_node_frontier = [node for node in frontier if node[2] == nextstate]
                if existing_node_frontier[0][0] > current_cost + cost + heuristic_cost_next_state:
                    count = next(counter)
                    frontier.remove(existing_node_frontier[0])
                    frontier.append([current_cost + cost + heuristic_cost_next_state, count, nextstate])
                    # nextstate will be present in child list as well, updating it
                    current_child = [child_elem for child_elem in child if child_elem[0] == nextstate]
                    # remove it from child list
                    child.remove(current_child[0])
                    # Making an updated entry
                    child.append([nextstate, action, current_state])

    # reversing the action list

    action_list.reverse()

    return action_list


# to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data=plan_str))


if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    # print("Search Algorithm :",algorithm)
    # print("Current time :", str(datetime.now()))
    if algorithm is None:
        print('Incorrect Algorithm name.')
        exit(1)
    # Time stamp here to get execution time of search algorithms
    start = timer()
    actions = algorithm()
    end = timer()
    print(end - start)  # Time in seconds
    exec_action_list(actions)



