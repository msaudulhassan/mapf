import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost, a_star

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    def vertex_collision(t):
        # Returns vertex collision, otherwise returns None
        agent1_curr_loc = get_location(path1, t)
        agent2_curr_loc = get_location(path2, t)
        if agent1_curr_loc == agent2_curr_loc:
            return {'loc': [agent1_curr_loc], 'timestep': t}
        return None

    def edge_collision(t):
        # Returns edge collision (in a way that blames Agent 1 for
        # the collision), otherwise returns None
        agent1_prev_loc = get_location(path1, t-1)
        agent2_prev_loc = get_location(path2, t-1)
        agent1_curr_loc = get_location(path1, t)
        agent2_curr_loc = get_location(path2, t)
        if agent1_prev_loc == agent2_curr_loc and agent2_prev_loc == agent1_curr_loc:
            return {'loc': [agent1_prev_loc, agent1_curr_loc], 'timestep': t}
        return None

    # Check for vertex collision at the starting location
    if vertex_collision(t=0) is not None:
        return vertex_collision(t=0)
    # Check for vertex and edge collisions on the whole path
    for t in range(1, max(len(path1), len(path2))):
        if vertex_collision(t) is not None:
            return vertex_collision(t)
        if edge_collision(t) is not None:
            return edge_collision(t)

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for agent1 in range(len(paths)):
        next_agent = agent1+1
        for agent2 in range(next_agent, len(paths)):
            collision = detect_collision(paths[agent1], paths[agent2])
            if collision is not None:
                collisions.append({'a1': agent1, 'a2': agent2, \
                                   'loc': collision['loc'], \
                                   'timestep': collision['timestep']})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    VERTEX_COLLISION = 0
    EDGE_COLLISION = 1

    def collision_type():
        if len(collision['loc']) == 1:
            return VERTEX_COLLISION
        return EDGE_COLLISION

    def constraint(agent, reverse_constraint):
        agent_id = collision[f"a{agent}"]
        collision_loc = collision['loc'].copy()
        if reverse_constraint:
            collision_loc.reverse()
        return {'agent': agent_id,
                'loc': collision_loc,
                'timestep': collision['timestep']}

    constraints = []
    if collision_type() == VERTEX_COLLISION:
        constraints.append(constraint(agent = 1, reverse_constraint = False))
        constraints.append(constraint(agent = 2, reverse_constraint = False))
    if collision_type() == EDGE_COLLISION:
        constraints.append(constraint(agent = 1, reverse_constraint = False))
        constraints.append(constraint(agent = 2, reverse_constraint = True))

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    [NORMAL, DIAGNOSTIC] = [0, 1]  # Modes

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.mode = CBSSolver.NORMAL

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))    

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        if self.mode == CBSSolver.DIAGNOSTIC:
            print(">> Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        if self.mode == CBSSolver.DIAGNOSTIC:
            print("\n> Expand node {}\n====================".format(id))
        return node

    def print_node(self, node):
        print("Paths:")
        for agent,path in enumerate(node['paths']):
            print(f"Agent {agent}: {path}")

        print("\nCollision[0]:")
        print(node['collisions'][0] if node['collisions'] else 'No Collisions!')

        print("\nConstraints for Collision[0]:")
        print(node['constraints'])

    def find_solution(self, disjoint=True, mode=None):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        if mode is None:
            self.mode = CBSSolver.NORMAL
        else:
            self.mode = mode

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])

            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while self.open_list:
            node = self.pop_node()
            if self.mode == CBSSolver.DIAGNOSTIC:
                self.print_node(node)

            collisions = detect_collisions(node['paths'])
            if not collisions:
                self.print_results(node)
                return node['paths']

            first_collision = collisions[0]
            constraints = standard_splitting(first_collision)

            for constraint in constraints:
                child = {'cost': 0,
                        'constraints': [],
                        'paths': [],
                        'collisions': []}
                child['constraints'] = node['constraints'].copy()
                child['constraints'].append(constraint)

                child['paths'] = node['paths'].copy()
                agent_id = constraint['agent']
                new_path = a_star(self.my_map,
                              self.starts[agent_id], self.goals[agent_id],
                              self.heuristics[agent_id],
                              agent_id, child['constraints'])

                if new_path is not None:
                    child['paths'][agent_id] = new_path
                    child['collisions'] = detect_collisions(child['paths'])
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)


        print('\n No Solution Found! \n')
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        return None


    def print_results(self, node):
        print("\n Found a solution! \n")
        print("Constraints:")
        for constraint in node['constraints']:
            print(constraint)
        CPU_time = timer.time() - self.start_time
        print("\nCPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))