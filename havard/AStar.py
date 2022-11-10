# 7x7 grid, used as an example
test_grid = [
  [ 0, 0, 0, 0, 0, 0, 0 ],
  [ 0, 0, 1, 1, 1, 1, 0 ],
  [ 0, 1, 1, 1, 0, 0, 0 ],
  [ 0, 1, 0, 0, 0, 0, 1 ],
  [ 0, 1, 0, 0, 0, 1, 1 ],
  [ 0, 0, 0, 0, 1, 1, 1 ],
  [ 0, 0, 0, 1, 1, 1, 0 ]
]

start_pos = {'x': 1, 'y': 6}
end_pos = {'x': 4, 'y': 0}

test_grid_medium = [
  [ 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1 ],
  [ 0, 0, 0, 1, 1, 1, 1, 1, 1, 0 ,0, 1 ],
  [ 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1 ],
  [ 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1 ],
  [ 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0 ],
  [ 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0 ],
  [ 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1 ],
  [ 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1 ],
  [ 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0 ],
  [ 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0 ],
  [ 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0 ],
  [ 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 ]
]

start_pos_medium = {'x': 2, 'y': 0}
end_pos_medium_1 = {'x': 10, 'y': 8}
end_pos_medium_2 = {'x': 7, 'y': 0}

def solve_astar(grid, start, end):
  '''
  Uses the A* algorithm to find the optimal path through the maze.

  Input:
    grid - A 2D array, where 0 is empty space and 1 is an obstacle
    start - The coordinates in the grid of the current location
    end - The coordinates in the grid of the destination

  Returns:
    path, cost - 
  '''
  board = _AStarPathBoard(grid, start, end)
  return board.solve()

class _AStarPathNode:
  def __init__(self, position, obstacle):
    '''
    G Cost is cost/distance from start to here
    H Cost is cost/distance from here to finish
    F Cost is G Cost + H Cost

    Cost calculation: 14 * diagonal steps + 10 * straight steps

    Source is used to backtrack from goal to start to return the path

    Position is the x,y-coordinate of the node in the grid
    Obstacle is a boolean for whether self node is an obstacle or not (probably unneeded)
    '''
    self.g_cost = float('inf')
    self.h_cost = float('inf')
    self.f_cost = float('inf')

    self.source = {'x': 0, 'y': 0}

    self.position = position
    self.obstacle = obstacle

  def set_cost(self, g_cost, h_cost, source):
    '''
    If the new f cost is smaller than the current f cost,
    then the new path being offered is better, so take it
    '''
    new_f_cost = g_cost + h_cost
    if new_f_cost < self.f_cost:
      self.g_cost = g_cost
      self.h_cost = h_cost
      self.f_cost = new_f_cost

      self.source = source


  def run_scan(self, board, goal, own_index):
    x_range = range(-1, 2) if self.source['x'] == 1 else range(1, -2, -1)
    y_range = range(-1, 2) if self.source['y'] == 1 else range(1, -2, -1)

    for y in y_range:
      for x in x_range:
        if x == 0 and y == 0: continue # Make sure not checking self

        new_position = {
          'x': self.position['x'] + x,
          'y': self.position['y'] + y
        }

        if new_position['x'] < 0 or new_position['y'] < 0 or new_position['y'] >= len(board.grid) or new_position['x'] >= len(board.grid[new_position['y']]):
          continue # Next position is outside the grid

        if board.node_is_closed_at_position(new_position):
          continue # The node has already been checked

        if board.grid[new_position['y']][new_position['x']].obstacle:
          continue # That node is an obstacle

        # If going diagonally, make sure we don't cut corners
        if x != 0 and y != 0:
          if board.grid[new_position['y']][self.position['x']].obstacle or board.grid[self.position['y']][new_position['x']].obstacle:
            continue

        # Get distance to next node
        distance = {
          'x': abs(goal['x'] - new_position['x']),
          'y': abs(goal['y'] - new_position['y'])
        }

        # Calculate new g and h cost
        new_g_cost = self.g_cost + 10
        if x != 0 and y != 0: new_g_cost += 4

        new_h_cost = 0
        if distance['x'] >= distance['y']:
          new_h_cost = distance['y'] * 14
          new_h_cost += (distance['x'] - distance['y']) * 10
        else:
          new_h_cost = distance['x'] * 14
          new_h_cost += (distance['y'] - distance['x']) * 10


        # Return true if we have reached the goal
        if new_h_cost == 0: return True
        else:
          board.grid[new_position['y']][new_position['x']].set_cost(new_g_cost, new_h_cost, {'x': -x, 'y': -y})

          # Prevent duplicate open nodes
          if not board.node_is_open_at_position(new_position):
            board.open_path_nodes.append(new_position)

    board.open_path_nodes.pop(own_index)
    board.closed_path_nodes.append(self.position)

    return False

class _AStarPathBoard:
  def __init__(self, grid, start_pos, end_pos):
    self.start = start_pos
    self.goal = end_pos

    self.grid = self.get_grid_from_array(grid)

    self.open_path_nodes = []
    self.closed_path_nodes = []

    self.grid[start_pos['y']][start_pos['x']].set_cost(0, 0, {'x': 0, 'y': 0})
    self.open_path_nodes.append(start_pos)

    self.path = []

  def get_grid_from_array(self, grid):
    return_grid = []
    for y in range(len(grid)):
      return_grid.append([])
      for x in range(len(grid[y])):
        return_grid[-1].append(_AStarPathNode({'x': x, 'y': y}, grid[y][x] == 1))

    return return_grid

  def solve(self):
    while len(self.open_path_nodes) > 0:
      top_path_node = self.grid[self.open_path_nodes[0]['y']][self.open_path_nodes[0]['x']]
      min_f_cost = top_path_node.f_cost
      min_h_cost = top_path_node.h_cost
      best_node = top_path_node
      best_index = 0

      for i in range(1, len(self.open_path_nodes)):
        current_node = self.grid[self.open_path_nodes[i]['y']][self.open_path_nodes[i]['x']]
        if current_node.f_cost < min_f_cost or (current_node.f_cost <= min_f_cost and current_node.h_cost < min_h_cost):
          min_f_cost = current_node.f_cost
          min_h_cost = current_node.h_cost
          best_node = current_node
          best_index = i

      if best_node.run_scan(self, self.goal, best_index):
        self.path = [self.goal]

        pos = best_node.position
        while pos['x'] != self.start['x'] or pos['y'] != self.start['y']:
          self.path.append(pos)
          move = self.grid[pos['y']][pos['x']].source
          pos = {'x': pos['x'] + move['x'], 'y': pos['y'] + move['y']}

        self.path.append(self.start)
        self.path = list(reversed(self.path))

        return self.path, best_node.f_cost

    return [] # Could not find a path!

  def node_is_open_at_position(self, position):
    for node in self.open_path_nodes:
      if node['x'] == position['x'] and node['y'] == position['y']: return True

    return False

  def node_is_closed_at_position(self, position):
    for node in self.closed_path_nodes:
      if node['x'] == position['x'] and node['y'] == position['y']: return True

    return False

def display_grid_and_path(grid, start_pos, end_pos, path):
  for y in range(len(grid)):
    for x in range(len(grid[y])):
      inPath = False
      for pos in path:
        if pos['x'] == x and pos['y'] == y: inPath = True

      if grid[y][x] == 1:
        print('\u001b[40m ', end='')
      elif start_pos['x'] == x and start_pos['y'] == y:
        print('\u001b[46mS', end='')
      elif end_pos['x'] == x and end_pos['y'] == y:
        print('\u001b[46mE', end='')
      elif inPath:
        print('\u001b[44m ', end='')
      else:
        print('\u001b[47m ', end='')
    print('\u001b[0m')

if __name__ == '__main__':
  grids = [test_grid, test_grid_medium, test_grid_medium]
  start_positions = [start_pos, start_pos_medium, start_pos_medium]
  end_positions = [end_pos, end_pos_medium_1, end_pos_medium_2]
  for i in range(3):
    grid = grids[i]
    start = start_positions[i]
    end = end_positions[i]
    print()

    path, cost = solve_astar(grid, start, end)
    print(f'Path: {path}')
    print(f'Cost: {cost}')
    display_grid_and_path(grid, start, end, path)

