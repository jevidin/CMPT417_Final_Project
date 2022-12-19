#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                if my_map[x][y]:
                    print(f"IN WALL AT FOR {x} {y}")
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    # print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            else:
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        if my_map[sx][sy]:
            print(f"SKIP IN WALL START {sx} {sy}")
            continue
        if my_map[gx][gy]:
            print(f"SKIP IN WALL GOAL {gx} {gy}")
            continue
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals, num_agents


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--idcbs', action='store_true', default=False,
                        help='Run IDCBS')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--output', type=str, default="results",
                        help='The name of output file')
    parser.add_argument('--repeat', action='store_true', default=False,
                        help='Rerun CBS cumulatively for each agent')

    args = parser.parse_args()


    result_file = open(f"{args.output}.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, goals, num_agents = import_mapf_instance(file)
        # print_mapf_instance(my_map, starts, goals)
        break_for = False
        for ag in range(num_agents+1):
            if not args.repeat:
                ag = num_agents
                break_for = True
            if args.solver == "CBS":
                print(f"***Run CBS*** AGENTS: {ag}")
                cbs = CBSSolver(my_map, starts[:ag], goals[:ag])
                paths, surplus, time = cbs.find_solution(args.disjoint, args.idcbs)
            elif args.solver == "Independent":
                print(f"***Run Independent*** AGENTS: {ag}")
                solver = IndependentSolver(my_map, starts[:ag], goals[:ag])
                paths = solver.find_solution()
            elif args.solver == "Prioritized":
                print(f"***Run Prioritized*** AGENTS: {ag}")
                solver = PrioritizedPlanningSolver(my_map, starts[:ag], goals[:ag])
                paths = solver.find_solution()
            else:
                raise RuntimeError("Unknown solver!")
            if break_for:
                break

            cost = get_sum_of_cost(paths)
            result_file.write("{},{},{},{},{}\n".format(file, cost, surplus, ag, time))


        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
