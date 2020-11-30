import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        start_time = timer.time()
        result = []
        constraints = []

        # constraints = [{'agent': 0, 'loc': [(1,5)], 'timestep': 4, 'type': 'vertex'}]  # For Task 1,2
        # constraints = [{'agent': 0, 'loc': [(1,5)], 'timestep': 4}]  # For Task 1,2
        # constraints = [{'agent': 1, 'loc': [(1,2), (1,3)], 'timestep': 1, 'type': 'edge'}]  # For Task 1,3
        # constraints = [{'agent': 1, 'loc': [(1,2), (1,3)], 'timestep': 1}]  # For Task 1,3
        # constraints = [{'agent': 0, 'loc': [(1,5)], 'timestep': 6, 'type': 'vertex'},
        #                {'agent': 0, 'loc': [(1,5)], 'timestep': 15, 'type': 'vertex'},
        #                 {'agent': 0, 'loc': [(1,5)], 'timestep': 9, 'type': 'vertex'}]  # For Task 1.4
        # constraints = [{'agent':1, 'loc': [(1,3), (1,4)], 'timestep': 2},
        #        {'agent':1, 'loc': [(1,3)], 'timestep': 2},
        #        {'agent':1, 'loc': [(1,2)], 'timestep': 2},
        #        {'agent': 0, 'loc': [(1,5)], 'timestep': 6, 'type': 'vertex'}]  # For Task 1.5

        # agent_priorities = [1,0]  # Bad ordering for exp2_1.txt
        agent_priorities = range(self.num_of_agents)  # Linear ordering

        for ind,i in enumerate(agent_priorities):  # Find path for each agent          
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            for j in range(ind+1,len(agent_priorities)):
                agent = agent_priorities[j]
                for timestep in range(len(path)-1):
                    curr_loc = path[timestep]
                    next_loc = path[timestep + 1]
                    constraints.append({'agent': agent, 'loc': [curr_loc],
                                        'timestep': timestep,
                                        'type': 'vertex'})
                    constraints.append({'agent': agent, 'loc': [next_loc, curr_loc],
                                        'timestep': timestep + 1,
                                        'type': 'edge'})
                constraints.append({'agent': agent, 'loc': path[-1],
                                    'timestep': len(path)-1,
                                    'type': 'inf'})
        
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)

        return result
