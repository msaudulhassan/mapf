import heapq
import math

[RIGHT, UP, LEFT, DOWN, STAY] = [0, 1, 2, 3, 4]

def move(loc, action):
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (0, 0)]
    return loc[0] + directions[action][0], loc[1] + directions[action][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent, goal_loc):
    constraint_table = {}
    inf_constraints = {}
    goal_constraint = None
    
    def add_to_constraint_table(timestep, constraint_loc, constraint_type):
        constraint = {'loc': constraint_loc, 'type': constraint_type}

        if timestep in constraint_table:
            constraint_table[timestep].append(constraint)
        else:
            constraint_table[timestep] = []
            constraint_table[timestep].append(constraint)

    def add_to_inf_constraints(timestep, constraint_loc, constraint_type):
        inf_constraints[constraint_loc] = {'timestep': timestep, 'type': constraint_type}

    for constraint in constraints:
        if constraint['agent'] == agent:
            timestep = constraint['timestep']
            constraint_loc = constraint['loc']
            if 'type' in constraint:
                constraint_type = constraint['type']
            else:  # For backwards compatibility
                num_locs = len(constraint['loc'])
                constraint_type = 'vertex' if num_locs == 1 else 'edge' if num_locs == 2 else None

            add_to_constraint_table(timestep, constraint_loc, constraint_type)

            if constraint_type == 'inf':
                add_to_inf_constraints(timestep, constraint_loc, constraint_type)

            if constraint_type == 'vertex' and constraint_loc[0] == goal_loc:
                goal_constraint = timestep if goal_constraint is None or goal_constraint < timestep else goal_constraint

    return constraint_table, inf_constraints, goal_constraint


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location
  

def path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def get_path(goal_node):
    """ For backwards compatibility """
    return path(goal_node)


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    if next_time in constraint_table:
        constraints = constraint_table[next_time]  # list of constraints on the agent
                                                   # at the this timestep
        for constraint in constraints:
            if constraint['type'] == 'vertex' and next_loc == constraint['loc'][0]:
                # vertex constraint violated
                return True
            elif constraint['type'] == 'edge' and \
                    curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                # edge constraint violated
                return True

    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def is_better(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def compare_nodes(n1, n2):
    """ For backwards compatibility """
    return is_better(n1, n2)


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):

    # INITIALIZATION ########################
    constraint_table, inf_constraints, goal_constraint = build_constraint_table(constraints, agent, goal_loc)

    frontier = []
    explored_set = {}


    # HELPER FUNCTIONS ########################
    def breaks_constraint(node):
        node_loc = node['loc']
        parent_loc = node['parent']['loc']
        node_timestep = node['timestep']

        if is_constrained(parent_loc, node_loc, node_timestep, constraint_table):
            # returns True if the agent is constrained to be not in node_loc at node_timestep
            return True

        if node_loc in inf_constraints:
            # returns True if there is a inf constraint that the agent might be breaking
            inf_constraint = inf_constraints[node_loc]
            if node_timestep >= inf_constraint['timestep']:
                return True
        
        return False


    def max_path_length():
        planned_path_lengths = math.ceil(len(constraints)/2)
        env_size = sum([row.count(False) for row in my_map])
        return env_size + planned_path_lengths


    def exceeds_time_limit(node):
        cushion = 0
        max_timesteps = max_path_length() + cushion
        if node['timestep'] > max_timesteps:
            return True


    def create_node(loc, g_val, h_val, parent, timestep):
        return {'loc': loc,
                'g_val': g_val,
                'h_val': h_val,
                'parent': parent,
                'timestep': timestep}


    def create_child(parent, child_loc):
        return create_node(loc = child_loc,
                           g_val = parent['g_val'] + 1,
                           h_val = h_values[child_loc],
                           parent = parent,
                           timestep = parent['timestep'] + 1)

    def location(node):
        return node['loc']

    def is_goal(node):
        if node['loc'] == goal_loc and (goal_constraint is None or \
            node['timestep'] > goal_constraint):
            return True

        return False

    def out_of_map(loc):
        return loc[0] < 0 or loc[0] >= len(my_map) \
                    or loc[1] < 0 or loc[1] >= len(my_map[0])

    def is_blocked(loc):
        return my_map[loc[0]][loc[1]]

    def already_explored(node):
        return (node['loc'], node['timestep']) in explored_set

    def fetch_from_explored(node):
        return explored_set[(node['loc'], node['timestep'])]

    def update_explored_set(node):
        explored_set[(node['loc'], node['timestep'])] = node

    def update_frontier(node):
        push_node(frontier, node)


    # A* ALGORITHM ########################
    root = create_node(loc = start_loc,
                       g_val = 0,
                       h_val = h_values[start_loc],
                       parent = None,
                       timestep = 0)

    update_explored_set(root)
    update_frontier(root)

    while len(frontier) > 0:
        node = pop_node(frontier)

        if is_goal(node):
            return path(node)

        if exceeds_time_limit(node):
            print('>>>>>>>>>>>>>> TIMELIMIT FOR PATHS EXCEEDED <<<<<<<<<<<<<<')
            return path(node)

        for action in [RIGHT, UP, LEFT, DOWN, STAY]:
            new_loc = move(location(node), action)

            if not (out_of_map(new_loc) or is_blocked(new_loc)):
                child = create_child(parent = node, child_loc = new_loc)

                if breaks_constraint(child):
                    continue

                # update explored set and frontier
                if already_explored(child):
                    existing_node = fetch_from_explored(node)
                    if is_better(child, existing_node):
                        update_explored_set(child)
                        update_frontier(child)
                else:
                    update_explored_set(child)
                    update_frontier(child)

    return None  # Failed to find solutions