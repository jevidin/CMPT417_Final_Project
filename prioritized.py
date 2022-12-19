import time as timer
from single_agent_planner import compute_heuristics, ida_star, a_star, get_sum_of_cost


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
        # Task 1.5 set of constraints to find collision free paths.
        # constraints = [{'agent': 1, 'loc': [(1,3)], 'timestep': 2}, 
        #                 {'agent': 1, 'loc': [(1,4)], 'timestep': 2}, 
        #                 {'agent': 1, 'loc': [(1,2)], 'timestep': 2}]

        print(self.my_map)
        for i in range(self.num_of_agents):  # Find path for each agent
            path = ida_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            # path = ida_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
            #              i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            ##############################
            for idx in range(len(path)):
                for ag in range(i+1, self.num_of_agents):
                    if idx + 1 == len(path):
                        constraints.append({'agent': ag, 'loc': [path[idx]], 'timestep': idx, 'goal':True}) # If this is the last node in the path, then it is the goal node
                    else:
                        constraints.append({'agent': ag, 'loc': [path[idx]], 'timestep': idx, 'goal':False})
                    if idx < len(path) - 1: 
                        constraints.append({'agent': ag, 'loc': [path[idx+1], path[idx]], 'timestep': idx+1, 'goal':False})

         # for j in range (len(path) + 1):

            #         newStep = len(path)
            #         newStep += j

            #         constraints.append({'agent' : i, 'loc' : [path[len(path) - 1]], 'timestep' : newStep, 'goal':True})
                
            # for k in range(len(path)):

            #         if i == j:
            #             continue
            #         else:

            #             constraints.append({'agent' : i, 'loc' : [path[k]], 'timestep' : k, 'goal': False})

            #             if k < 1:
            #                 continue
            #             else:
            #                 constraints.append({'agent' : i, 'loc' : [path[k], path[k-1]], 'timestep' : k, 'goal': False})
            


        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
