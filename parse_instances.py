from pathlib import Path
import sys
def parse_coordinates(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    # rows, columns = [int(x) for x in line.split(' ')]
    # rows = int(rows)
    # columns = int(columns)
    while line:
        line = f.readline()
        linearr = line.split(' ')
        if line:
            coords = linearr[4:8]
            start_y = coords[0]
            start_x = coords[1]
            goal_y = coords[2]
            goal_x = coords[3]
            result_file.write("{} {} {} {}\n".format(start_x, start_y, goal_x, goal_y))
    return

scen = sys.argv[1]
result_file = open(f"{scen[:-5]}.txt", "w", buffering=1)
parse_coordinates(scen)
result_file.close()