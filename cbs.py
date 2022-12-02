import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    # pathA, pathB = pad_path(path1, path2)
    pathA, pathB = path1, path2
    long = max(len(path1), len(path2))
    for timestep in range(long):
        s_pos = get_location(pathA, timestep)
        l_pos = get_location(pathB, timestep)
        if s_pos == l_pos:
            return {'loc':[s_pos], 'timestep':timestep, 'goal':False}

        next_s_pos = get_location(pathA, timestep+1)
        next_l_pos = get_location(pathB, timestep+1)
        if s_pos == next_l_pos and l_pos == next_s_pos:
            return {'loc':[s_pos, next_s_pos], 'timestep':timestep+1, 'goal':False}

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)-1):
        for j in range(i+1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)
    
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
    constraints = []
    if len(collision['loc']) < 2:
        constraints.append(
            {
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'goal':collision['goal']
            }
        )
        constraints.append(
            {
                'agent': collision['a2'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'goal':collision['goal']
            }
        )
    else:
        constraints.append(
            {
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'goal':False
            }
        )
        # reversed() returns interator, not list: https://realpython.com/python-reverse-list/
        constraints.append(
            {
                'agent': collision['a2'],
                'loc': list(reversed(collision['loc'])),
                'timestep': collision['timestep'],
                'goal':False
            }
        )
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
    constraints = []
    agents = [collision['a1'], collision['a2']]
    rand_choice = random.randint(0,1)
    agent_choice = agents[rand_choice]
    loc = collision['loc']
    if rand_choice == 1: # If we ended up choosing the second agent, reverse the locations for edge collision
        loc = list(reversed(collision['loc']))
    constraints.append(
        {
            'agent': agent_choice,
            'loc': loc,
            'timestep': collision['timestep'],
            'goal':False,
            'positive':True
        }
    )
    constraints.append(
        {
            'agent': agent_choice,
            'loc': loc,
            'timestep': collision['timestep'],
            'goal':False,
            'positive':False
        }
    )

    return constraints

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

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
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

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

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            p = self.pop_node()
            if not p['collisions']:
                return p['paths']
            collision = p['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            for constraint in constraints:
                p_constraints = p['constraints'].copy()
                q = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
                q['constraints'] = p_constraints + [constraint]
                q['paths'] = p['paths'].copy()
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, q['constraints'])

                if path:
                    no_path = False
                    q['paths'][agent] = path
                    if constraint['positive']: # Task 4: when there is a positive constraint, re plan all other agents with a negative constraint which makes them avoid that positive constraint position.
                        violating_agents = paths_violate_constraint(constraint, q['paths']) # Find all other agents which now have to be re planned with new negative constraints to avoid this positive constraint
                        for ag in violating_agents:
                            new_path = a_star(self.my_map, self.starts[ag], self.goals[ag], self.heuristics[ag], ag, q['constraints']) # Run A* again, the build_constraint_table function has been updated to account for positive constraints of other agents
                            if new_path is None:
                                no_path = True
                                break
                            else:
                                q['paths'][ag] = new_path
                                
                        if no_path:
                            continue
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)
                else:
                    raise BaseException('No Solutions')
        self.print_results(root)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
