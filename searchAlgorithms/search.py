import sys
from queue import PriorityQueue

def read_maze(filename):
    with open(filename, 'r') as file:
        maze = []
        for line in file:
            maze.append(list(line.strip()))
        return maze

def print_maze(maze):
    for row in maze:
        print("".join(row))

def find_start(maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == 'S':
                return (i, j)
    return None

def find_goal(maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == 'G':
                return (i, j)
    return None

def valid_move(maze, move):
    i, j = move
    if i < 0 or i >= len(maze) or j < 0 or j >= len(maze[0]):
        return False
    if maze[i][j] == '%':
        return False
    return True

def manhattan_distance(current, goal):
    return abs(goal[0] - current[0]) + abs(goal[1] - current[1])

def euclidian_distance(current, goal):
    return ((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2) ** 0.5


def depth_first_search(maze, start, goal):
    stack = []
    stack.append((start, []))
    visited = set()
    visited.add(start)
    while stack:
        (i, j), path = stack.pop()
        if (i, j) == goal:
            return path
        moves = [(i-1, j), (i, j-1), (i+1, j), (i, j+1)]
        for move in moves:
            if move not in visited and valid_move(maze, move):
                stack.append((move, path + [move]))
                visited.add(move)
    return None

def greedy_best_first_search(maze, start, goal, heuristic):
    priority_queue = PriorityQueue()
    priority_queue.put((0, start, []))
    visited = set()
    visited.add(start)
    while not priority_queue.empty():
        _, (i, j), path = priority_queue.get()
        if (i, j) == goal:
            return path
        moves = [(i-1, j), (i, j-1), (i+1, j), (i, j+1)]
        for move in moves:
            if move not in visited and valid_move(maze, move):
                if heuristic == 'euclidian':
                    priority = euclidian_distance(move, goal)
                elif heuristic == 'manhattan':
                    priority = manhattan_distance(move, goal)
                else:
                    priority = 0
                priority_queue.put((priority, move, path + [move]))
                visited.add(move)
    return None

def a_star_search(maze, start, goal, heuristic):
    priority_queue = PriorityQueue()
    priority_queue.put((0, start, 0, []))
    visited = set()
    visited.add(start)
    while not priority_queue.empty():
        f, (i, j), g, path = priority_queue.get()
        if (i, j) == goal:
            return path
        moves = [(i - 1, j), (i, j - 1), (i + 1, j), (i, j + 1)]
        for move in moves:
            if move not in visited and valid_move(maze, move):
                if heuristic == 'euclidian':
                    h = euclidian_distance(move, goal)
                elif heuristic == 'manhattan':
                    h = manhattan_distance(move, goal)
                else :
                    h = 0
                f = g + h
                priority_queue.put((f, move, g + 1, path + [move]))
                visited.add(move)
    return None

def iterative_deepening_search(maze, start, goal, heuristic):
    for depth in range(0, sys.maxsize):
        result = depth_limited_search(maze, start, goal, depth, heuristic)
        if result != 'cutoff':
            return result
    return 'cutoff'

def children(maze, node):
    row, col = node
    children = []
    if row > 0 and maze[row - 1][col] != '%':
        children.append((row - 1, col))
    if row < len(maze) - 1 and maze[row + 1][col] != '%':
        children.append((row + 1, col))
    if col > 0 and maze[row][col - 1] != '%':
        children.append((row, col - 1))
    if col < len(maze[0]) - 1 and maze[row][col + 1] != '%':
        children.append((row, col + 1))
    return children



def depth_limited_search(maze, start, goal, depth, heuristic):
    if start == goal:
        return [start]
    cutoff_occurred = False
    explored = set()
    frontier = [[start]]
    while frontier:
        path = frontier.pop(0)
        node = path[-1]
        if len(path) >= depth:
            cutoff_occurred = True
            continue
        explored.add(node)
        for child in children(maze, node):
            if child in explored:
                continue
            new_path = path + [child]
            frontier.append(new_path)
            if child == goal:
                return new_path
    if cutoff_occurred:
        return 'cutoff'
    else:
        return 'failure'


def main():
    method = sys.argv[2]
    heuristic = sys.argv[4]
    filename = sys.argv[5]
    maze = read_maze(filename)
    start = find_start(maze)
    goal = find_goal(maze)

    if method == 'dfs':
        path = depth_first_search(maze, start, goal)
    elif method == 'greedy':
        path = greedy_best_first_search(maze, start, goal, heuristic)
    elif method == 'astar':
        path = a_star_search(maze, start, goal, heuristic)
    elif method == 'iterative':
        path = iterative_deepening_search(maze, start, goal, heuristic)
    else:
        print('Error: Invalid method')
        return

    if path is None:
        print('No solution found')
    else:
        for i, j in path:
            maze[i][j] = '•'
        print_maze(maze)
        print('Solution cost:', len(path))
        print('Nodes expanded:', len(path))

if __name__ == "__main__":
    main()
