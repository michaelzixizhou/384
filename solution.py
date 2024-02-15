#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import (
    sokoban_goal_state,
    SokobanState,
    Direction,
    PROBLEMS,
)  # for Sokoban specific classes and problems


def _hungarian_matching(objects1, objects2):
    """
    Perform Hungarian matching between two lists of objects.

    Args:
    - objects1 (list): List of coordinates tuples (x, y) for the first set of objects.
    - objects2 (list): List of coordinates tuples (x, y) for the second set of objects.

    Returns:
    - matching (dict): Dictionary mapping each object index from the first list to the index of the object it is assigned to from the second list.
    """
    total_edge_weight = 0

    # Calculate the distances between all pairs of objects
    distances = []
    for i, obj1 in enumerate(objects1):
        row = []
        for j, obj2 in enumerate(objects2):
            row.append(abs(obj1[0] - obj2[0]) + abs(obj1[1] - obj2[1]))
        distances.append(row)

    # Initialize an array to track which objects are assigned to which objects
    assignment = [-1] * len(objects1)

    # Iterate through each object and find the closest available object
    for _ in range(len(objects1)):
        min_val = float("inf")
        min_obj1_idx = -1
        min_obj2_idx = -1
        for i in range(len(objects1)):
            if assignment[i] == -1:
                for j in range(len(objects2)):
                    if distances[i][j] < min_val:
                        min_val = distances[i][j]
                        min_obj1_idx = i
                        min_obj2_idx = j
        assignment[min_obj1_idx] = min_obj2_idx
        total_edge_weight += min_val

    # Create a dictionary to store the matching
    # matching = {obj1_idx: obj2_idx for obj1_idx, obj2_idx in enumerate(assignment)}

    return total_edge_weight


def _is_dead_end(state):
    """
    Determines whether the current state represents a situation where further progress towards the goal is impossible.

    Args:
    - state (SokobanState): The Sokoban state.

    Returns:
    - True if the state is a dead end, False otherwise.


    """

    def is_surrounded_by_obstacles(box_x, box_y):
        """
        Check if the box at the given position is completely surrounded by obstacles, walls, or other boxes.

        Args:
        - box_x (int): The x-coordinate of the box.
        - box_y (int): The y-coordinate of the box.

        Returns:
        - True if the box is surrounded, False otherwise.
        """

        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        sides_blocked = 0
        for dx, dy in directions:
            new_x, new_y = box_x + dx, box_y + dy
            if (
                (new_x, new_y) in state.obstacles
                or (new_x, new_y) in state.boxes
                or (
                    new_x < 0
                    or new_x >= state.width
                    or new_y < 0
                    or new_y >= state.height
                )
            ):
                sides_blocked += 1

        return sides_blocked >= 3

    def has_no_storage_along_edge(box_x, box_y):
        """
        Checks if a box is positioned at the edge of the grid with no adjacent storage spaces along that edge.

        Args:
        - box_x (int): The x-coordinate of the box.
        - box_y (int): The y-coordinate of the box.

        Returns:
        - bool: True if the box is positioned at the edge with no adjacent storage spaces, False otherwise.
        """
        if (box_x == 0 or box_x == state.width - 1) or (
            box_y == 0 or box_y == state.height - 1
        ):
            if (
                (
                    box_x == 0
                    and all(
                        (box_x, box_y) not in state.storage
                        for box_y in range(state.height)
                    )
                )
                or (
                    box_x == state.width - 1
                    and all(
                        (box_x, box_y) not in state.storage
                        for box_y in range(state.height)
                    )
                )
                or (
                    box_y == 0
                    and all(
                        (box_x, box_y) not in state.storage
                        for box_x in range(state.width)
                    )
                )
                or (
                    box_y == state.height - 1
                    and all(
                        (box_x, box_y) not in state.storage
                        for box_x in range(state.width)
                    )
                )
            ):
                return True
        return False

    # Iterate over each box position
    for box_x, box_y in state.boxes:
        if is_surrounded_by_obstacles(box_x, box_y) or has_no_storage_along_edge(
            box_x, box_y
        ):
            return True  # Dead end found

    return False  # No dead end found


# SOKOBAN HEURISTICS
def heur_alternate(state: SokobanState):
    # IMPLEMENT
    """a better heuristic"""
    """INPUT: a sokoban state"""
    """OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal."""
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.

    """
    Explanation: we can reduce this problem to a matching problem in a bipartite graph, 
    where the graph is partitioned between boxes and storage, where we evaluate for 
    minimum cost. 
    
    We can also introduce another matching between robots and boxes, such that 
    we obtain a better heuristic score if the robots are closer to the boxes. 

    The sum of these two matching problems is what we return.  

    See also: https://en.wikipedia.org/wiki/Blossom_algorithm
    """
    total_score = 0

    # Deadend checks
    # Prune paths
    # if _is_dead_end(state):
    #     # print(state.state_string())
    #     return math.inf
    # #
    total_score += _hungarian_matching(state.boxes, state.robots)
    total_score += _hungarian_matching(state.boxes, state.storage)

    return total_score  # TODO:


def heur_zero(state: SokobanState):
    """Zero Heuristic can be used to make A* search perform uniform cost search"""
    return 0


def heur_manhattan_distance(state: SokobanState):
    # IMPLEMENT
    """admissible sokoban puzzle heuristic: manhattan distance"""
    """INPUT: a sokoban state"""
    """OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal."""
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    total_manhattan_distance = 0

    for box in state.boxes:
        min_distance = math.inf
        for goal in state.storage:
            distance = abs(goal[0] - box[0]) + abs(goal[1] - box[1])
            if distance < min_distance:
                min_distance = distance

        total_manhattan_distance += min_distance

    return total_manhattan_distance  # CHANGE THIS


def fval_function(sN: sNode, weight: float):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    f = sN.gval + weight * sN.hval

    return f


# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT
    """Provides an implementation of weighted a-star, as described in the HW1 handout"""
    """INPUT: a sokoban state that represents the start state and a timebound (number of seconds)"""
    """OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object"""
    """implementation of weighted astar algorithm"""

    se = SearchEngine("custom")

    se.init_search(
        initial_state,
        sokoban_goal_state,
        heur_fn,
        (lambda sN: fval_function(sN, weight)),
    )

    return se.search(timebound)  # CHANGE THIS


def _get_total_gval(state: StateSpace):
    curr_state = state
    total_gval = 0

    while curr_state:
        total_gval += curr_state.gval

        curr_state = curr_state.parent

    return total_gval


def iterative_astar(
    initial_state, heur_fn, weight=1, timebound=5
):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    """Provides an implementation of realtime a-star, as described in the HW1 handout"""
    """INPUT: a sokoban state that represents the start state and a timebound (number of seconds)"""
    """OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object"""
    """implementation of iterative astar algorithm"""

    # prune nodes if the g(node) + h(node) exceeds
    # the cost of the best goal path found so far
    se = SearchEngine("custom")

    start_time = os.times()[4]

    path = None
    best_gval = math.inf
    best_path = None
    best_path_stats = None

    while os.times()[4] - start_time < timebound - 0.1:
        remaining_time = timebound - (os.times()[4] - start_time)
        fvalfunc = lambda sN: fval_function(sN, weight)

        se.init_search(initial_state, sokoban_goal_state, heur_fn, fvalfunc)
        path, stats = se.search(remaining_time, (math.inf, math.inf, best_gval))
        if weight * 0.5 >= 1:
            weight *= 0.5
        if not path:
            continue

        curr_total_gval = _get_total_gval(path)
        if curr_total_gval < best_gval:
            best_gval = curr_total_gval
            best_path = path
            best_path_stats = stats

    print(f"{best_gval}, {weight}")

    return best_path, best_path_stats  # CHANGE THIS


def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    """Provides an implementation of anytime greedy best-first search, as described in the HW1 handout"""
    """INPUT: a sokoban state that represents the start state and a timebound (number of seconds)"""
    """OUTPUT: A goal state (if a goal is found), else False"""
    """implementation of iterative gbfs algorithm"""

    se = SearchEngine("custom")

    start_time = os.times()[4]
    cost_bound = None
    best_gval = math.inf
    best_path = None
    best_path_stats = None


    se.init_search(initial_state, sokoban_goal_state, heur_fn)

    while os.times()[4] - start_time < timebound - 0.1:
        remaining_time = timebound - (os.times()[4] - start_time)
        path, stats = se.search(remaining_time, cost_bound)
        
        if path:
            best_gval = min(best_gval, _get_total_gval(path))
            cost_bound = [best_gval, math.inf, math.inf]
            best_path = path
            best_path_stats = stats

    
    return best_path, best_path_stats  # CHANGE THIS
