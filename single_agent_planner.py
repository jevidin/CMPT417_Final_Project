import heapq
from pdb import set_trace as bp
def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


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
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # Build dictionary indexed by timestep where each timestep has an array of constraints
    constraint_table = dict()
    for con in constraints:
        if (not 'positive' in con.keys()):
            con['positive'] = False
        if con['agent'] == agent:
            if con['timestep'] in constraint_table:
                constraint_table[con['timestep']].append(con)
            else:
                constraint_table[con['timestep']] = [con]
        
        # Task 4: If there is a positive constraint that does not belong to the current agent, then a negative constraint needs to be created for the current agent to avoid that location at that timestep
        if con['agent'] != agent and con['positive']:
            if len(con['loc']) < 2:
                new_con = {'agent':agent,
                        'loc':con['loc'],
                        'timestep':con['timestep'],
                        'positive':False}
            else:
                new_con = {'agent':agent,
                            'loc':list(reversed(con['loc'])),
                            'timestep':con['timestep'],
                            'positive':False}
            if new_con['timestep'] in constraint_table:
                constraint_table[new_con['timestep']].append(new_con)
            else:
                constraint_table[new_con['timestep']] = [new_con]
    return constraint_table

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # Returns 1 if positive constraint, 0 if negative constraint, and -1 if not constrained
    if next_time in constraint_table:
        for con in constraint_table[next_time]:
            if con['loc'] == [next_loc] or con['loc'] == [curr_loc, next_loc]:
                if con['positive']:
                    return 1
                else:
                    return 0
    return -1

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    earliest_goal_timestep = 0
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 't_val': 0, 'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], root['t_val'])] = root
    map_size = len(my_map) * len(my_map[0])
    while len(open_list) > 0:
        if len(closed_list) > map_size*5:
            break
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        # Check for future constraints in the constraint table.
        if curr['loc'] == goal_loc:
            future_goal_constraint = False
            for timestep in constraint_table:
                if timestep > curr['t_val']:
                    for con in constraint_table[timestep]:
                        if con['loc'] == [goal_loc] and not con['positive']:
                            future_goal_constraint = True
            if not future_goal_constraint:
                return get_path(curr)

        positive_found = False
        for dir in range(4): # This for loop first checks all directions for positive constraints, and pushes the first one found. This ensures that no nodes are pushed to open before a positive constraint is pushed.
            child_loc = move(curr['loc'], dir)
            if is_constrained(curr['loc'], child_loc, curr['t_val']+1, constraint_table) == 1: # If positive constraint, move to that location and break so we don't expand other nodes
                positive_found = True
                if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
                    continue
                child = {'loc': child_loc,
                        'g_val': curr['g_val'] + 1,
                        'h_val': 0, # Make sure that this node is the one that gets popped and expanded because it's a positive constraint
                        't_val': curr['t_val'] + 1,
                        'parent': curr}
                if (child['loc'], child['t_val']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['t_val'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['t_val'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['t_val'])] = child
                    push_node(open_list, child)
                break
        if positive_found:
            continue

        for dir in range(5): # Only if there are no positive constraints from pervious for loop in all directions do we check for normal node expansion
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    't_val': curr['t_val'] + 1,
                    'parent': curr}

            if is_constrained(curr['loc'], child['loc'], child['t_val'], constraint_table) == 0: # If constrained then don't push node to open_list
                continue
            else: # This part is for prioritized planning in task 2, prevents agents from moving on top of another agent which is in its goal position
                constrained = False
                for con in constraints:
                    if [child_loc] == con['loc'] and con['goal'] and agent == con['agent']:
                        constrained = True
                if constrained:
                    continue

            if (child['loc'], child['t_val']) in closed_list:
                existing_node = closed_list[(child['loc'], child['t_val'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['t_val'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['t_val'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
