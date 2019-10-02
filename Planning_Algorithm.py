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
import json
import subprocess

rospy.init_node("Planning")
publisher = rospy.Publisher("/actions", String, queue_size=10)
# subscriber=rospy.Subscriber("/status",String,callback_status)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Default is astar", metavar='astar', action='store',
                    dest='algorithm', default="astar", type=str)


# def callback_status(data)
#    if data.data=="idle":
def action_parser(token):
    moves_list = list()
    if (token[0] == "step" and token[5] == "MOVE"):
        moves_list.append(token[5])
        moves_list.append(token[6])
        moves_list.append(token[7])
        moves_list.append(token[8])
        # print(moves_list)
        return moves_list
    elif (token[1] == "MOVE"):
        moves_list.append(token[1])
        moves_list.append(token[2])
        moves_list.append(token[3])
        moves_list.append(token[4])
        # print(moves_list)
        return moves_list
    elif (token[1] == 'PICK'):
        moves_list.append(token[1])
        moves_list.append(token[2])
        moves_list.append(token[3])
        moves_list.append(token[4])
        moves_list.append(token[5])
        # print(moves_list)
        return moves_list
    elif (token[1] == 'PLACE'):
        moves_list.append(token[1])
        moves_list.append(token[2])
        moves_list.append(token[3])
        moves_list.append(token[4])
        moves_list.append(token[5])
        moves_list.append(token[6])
        moves_list.append(token[7])
        moves_list.append(token[8])
        moves_list.append(token[9])
        moves_list.append(token[10])
        # print(moves_list)
        return moves_list
    else:
        print("Nothing matched")


def astar(init_state, goal_state):
    # init_state = problem.get_initial_state()
    # goal_state = problem.get_book_state("book_40")
    print("goal state:", goal_state)
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
        # print("inside frontier")
        current_cost, current_order, current_state = heapq.heappop(frontier)
        entry_finder.remove(current_state)
        heuristic_current_state = (abs(goal_state.x - current_state.x) + abs(goal_state.y - current_state.y))
        current_cost = current_cost - heuristic_current_state
        # Algo Exit section
        if (current_state == goal_state):
            # print("condition failed\n")
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
            # print("nextstate:",nextstate)
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


# to execute a plan action_list = <list of actions>, use:
# def exec_action_list(action_list):
#   plan_str = '_'.join(action for action in action_list)
#   publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    # print("Search Algorithm :",algorithm)
    # print("Current time :", str(datetime.now()))
    if algorithm is None:
        print("Incorrect Algorithm name.")
        exit(1)
    # Time stamp here to get execution time of planning algorithms
    start = timer()
    #####New by me
    root_path = "/home/deepika/catkin_ws/src/Planning"

    subprocess.call(
        '/home/deepika/catkin_ws/src/Planning/ff -o /home/deepika/catkin_ws/src/Planning/domain.pddl -f /home/deepika/catkin_ws/src/Planning/problem.pddl > /home/deepika/catkin_ws/src/Planning/Plan.txt',
        shell=True)
    filename_1 = root_path + "/books.json"
    with open(filename_1) as file:
        obj_dict = json.load(file)
    # print(type(obj_dict))
    # print("object dictionary:",obj_dict)
    # print(filename)
    moves_list = list()
    filename = "/home/deepika/catkin_ws/src/Planning/Plan.txt"
    flag = 0
    with open(filename) as f:
        for line in f:
            # print(line)
            token = line.strip().split(' ')
            if (flag == 0 and token[0] == "ff:" and token[1] == "found"):
                print(line)
                flag = 1
                continue
            if (flag == 1 and token[0] != '' and token[0] != 'time' and token[1] != 'seconds'):
                moves_list = action_parser(token)
                print("move_list:", moves_list)
                if (flag == 1 and moves_list[0] == "MOVE"):
                    init_state_var = moves_list[1]
                    goal_state_var = moves_list[2]
                    if "INIT" in init_state_var and "BOOK" in goal_state_var:
                        init_state = problem.get_initial_state()
                        # print("initial state of robot:",init_state)
                        l = [int(s) for s in goal_state_var.split('_') if s.isdigit()]
                        book_name = "book_" + str(l[0])
                        # print(book_name)
                        book_state = obj_dict["books"][book_name]["load_loc"][0]
                        goal_state = problem.State(book_state[0], book_state[1], "EAST")
                        print(type(goal_state))
                        actions = astar(init_state, goal_state)
                        print("actions 1st:", actions)
                        problem.execute_move_action(actions)
                    elif "BOOK" in init_state_var and "TROLLY" in goal_state_var:
                        m = [int(s) for s in init_state_var.split('_') if s.isdigit()]
                    book_name = "book_" + str(m[0])
                    book_state = obj_dict["books"][book_name]["load_loc"][0]
                    init_state = problem.State(book_state[0], book_state[1], "EAST")
                # print("initial state of robot:",init_state)
                n = [int(s) for s in goal_state_var.split('_') if s.isdigit()]
                bin_name = "trolly_" + str(n[0])
                # print(book_name)
                bin_state = obj_dict["bins"][bin_name]["load_loc"][0]
                goal_state = problem.State(bin_state[0], bin_state[1], "EAST")
                actions = astar(init_state, goal_state)
                print("actions 2nd:", actions)
                problem.execute_move_action(actions)
            else:
                p = [int(s) for s in init_state_var.split('_') if s.isdigit()]
            bin_name = "trolly_" + str(p[0])
            bin_state = obj_dict["bins"][bin_name]["load_loc"][0]
            init_state = problem.State(bin_state[0], bin_state[1], "EAST")
            # print("initial state of robot:",init_state)
            r = [int(s) for s in goal_state_var.split('_') if s.isdigit()]
            book_name = "book_" + str(r[0])
            # print(book_name)
            book_state = obj_dict["books"][book_name]["load_loc"][0]
            goal_state = problem.State(book_state[0], book_state[1], "EAST")
            actions = astar(init_state, goal_state)
            print("actions 3rd:", actions)
            problem.execute_move_action(actions)
    if (flag == 1 and moves_list[0] == 'PLACE'):
        book_name = moves_list[-1].lower()
        bin_name = moves_list[-2].lower()
        b = problem.execute_place_action(book_name, bin_name, goal_state)
        print(b)
    if (flag == 1 and moves_list[0] == 'PICK'):
        book_name = moves_list[-1].lower()
        print(book_name)
        # print("goal state:",goal_state.x,goal_state.y,goal_state.orientation)
        a = problem.execute_pick_action(book_name, goal_state)
        print(a)
end = timer()
print("total time taken is:", end - start)