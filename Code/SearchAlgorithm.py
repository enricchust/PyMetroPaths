# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1638322'
__group__ = 'DL11'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2022 - 2023
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """

    pathExpanded = []

    for x in map.connections[path.last]:
        pathAct = copy.deepcopy(path)
        #we can't equal to path because it altere the memory so we use copy.deepcopy
        pathAct.add_route(x)
        pathExpanded.append(pathAct)

    #print_list_of_path(pathExpanded)

    return pathExpanded


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    rep = []
    for pathAct in path_list:
        for i in pathAct.route:
            count = 0
            for j in pathAct.route:
                if i == j:
                    count += 1
        if(count < 2):
            rep.append(pathAct)

    return rep


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    list_of_path.pop(0)
    list_of_path = expand_paths + list_of_path

    return list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """

    llista = [Path(origin_id)]

    while(llista[0].last != destination_id or len(llista) == 0):
        act = llista[0]
        exp = expand(act, map)
        exp = remove_cycles(exp)
        llista = insert_depth_first_search(exp, llista)

    if(len(llista) != 0):
        return llista[0]
    else:
        return print("No existeix Solucio")


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """

    # the same than the function of insert depth but in the inverse order
    list_of_path.pop(0)
    list_of_path = list_of_path + expand_paths

    return list_of_path


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    llista = [Path(origin_id)]

    while (llista[0].last != destination_id or len(llista) == 0):
        act = llista[0]
        exp = expand(act, map)
        exp = remove_cycles(exp)
        llista = insert_breadth_first_search(exp, llista)

    if (len(llista) != 0):
        return llista[0]
    else:
        return print("No existeix Solucio")


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """

    if type_preference == 0:
        for actPath in expand_paths:
            actPath.update_g(1)

    elif type_preference == 1:
        for actPath in expand_paths:
            lastStop = actPath.penultimate
            nextStop = actPath.last

            cost = map.connections[lastStop][nextStop]
            actPath.update_g(cost)

    elif type_preference == 2:
        for actPath in expand_paths:
            lastStop = actPath.penultimate
            nextStop = actPath.last

            liniaLast = map.stations[lastStop]['line']
            liniaNext = map.stations[nextStop]['line']

            if liniaNext == liniaLast:
                velocity = map.velocity[liniaLast]
                time = map.connections[lastStop][nextStop]

                cost = velocity * time

                actPath.update_g(cost)

    elif type_preference == 3:
        for actPath in expand_paths:
            lastStop = actPath.penultimate
            nextStop = actPath.last

            if map.stations[lastStop]['line'] == map.stations[nextStop]['line']:
                actPath.update_g(0)
            else:
                actPath.update_g(1)

    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """

    newList = list_of_path + expand_paths

    return sorted(newList, key=lambda path: path.g)

def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    llista = [Path(origin_id)]

    while llista[0].last != destination_id or len(llista) == 0:
        act = llista[0]
        exp = expand(act, map)
        exp = remove_cycles(exp)
        exp = calculate_cost(exp, map, type_preference)
        llista = insert_cost(exp, llista[1:])

    if (len(llista) != 0):
        return llista[0]
    else:
        return print("No existeix Solucio")


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """


    if type_preference == 0:
        for actPath in expand_paths:
            if actPath.last == destination_id:
                actPath.update_h(0)
            else:
                actPath.update_h(1)

    if type_preference == 1:
        coorDest = (map.stations[destination_id]['x'], map.stations[destination_id]['y'])
        maxVelocity = max(map.velocity.values())

        for actPath in expand_paths:
            coorAct = (map.stations[actPath.last]['x'], map.stations[actPath.last]['y'])
            distance = euclidean_dist(coorAct, coorDest)
            heuristic = distance / maxVelocity

            actPath.update_h(heuristic)

    if type_preference == 2:
        coorDest = (map.stations[destination_id]['x'], map.stations[destination_id]['y'])

        for actPath in expand_paths:
            coorAct = (map.stations[actPath.last]['x'], map.stations[actPath.last]['y'])
            heuristic = euclidean_dist(coorAct, coorDest)

            actPath.update_h(heuristic)

    if type_preference == 3:
        for actPath in expand_paths:
            if map.stations[actPath.last]['line'] != map.stations[destination_id]['line']:
                actPath.update_h(1)
            else:
                actPath.update_h(0)

    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for actPath in expand_paths:
        actPath.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """

    for actPath in expand_paths:
        if actPath.last in visited_stations_cost.keys():
            if visited_stations_cost[actPath.last] > actPath.g:
                visited_stations_cost[actPath.last] = actPath.g
                for auxPath in list_of_path:
                    if auxPath.last == actPath.last:
                        list_of_path.remove(auxPath)
            else:
                expand_paths.remove(actPath)
        else:
            visited_stations_cost[actPath.last] = actPath.g

    return expand_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """

    newList = list_of_path + expand_paths

    return sorted(newList, key=lambda path: path.f)

def coord2station(coord, map):
    """
        From coordinates, it searches the closest stations.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """

    dFin = float("inf")
    linies = []

    for id, station in map.stations.items():
        xy = [station['x'], station['y']]
        dAct = euclidean_dist(coord, xy)

        if(dAct < dFin):
            dFin = dAct
            linies = []
            linies.append(id)

        elif(dAct == dFin):
            linies.append(id)

    return linies


def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    llista = [Path([origin_id])]
    vsc = {}

    while llista[0].last != destination_id and len(llista) != 0:
        last = llista.pop(0)
        exp = expand(last, map)
        exp = remove_cycles(exp)
        exp = calculate_cost(exp, map, type_preference)
        exp = calculate_heuristics(exp, map, destination_id, type_preference)
        exp = update_f(exp)
        exp, llista, vsc = remove_redundant_paths(exp, llista, vsc)
        llista = insert_cost_f(exp, llista)

    if (len(llista) != 0):
        return llista[0]
    else:
        return print("No existeix Solucio")