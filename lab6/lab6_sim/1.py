'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
import copy
import math
import random
import argparse
from PIL import Image
import numpy as np
import sys
from pprint import pprint
import heapq

g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 3 # 2m wide
g_MAP_SIZE_Y = 3 # 1.5m tall
g_MAP_RESOLUTION_X = 0.25 # Each col represents cm
g_MAP_RESOLUTION_Y = 0.25 # Each row represents cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [[0 for _ in range(g_NUM_X_CELLS)] for _ in range(g_NUM_Y_CELLS)] # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)

def rev(n):
  return n[1], n[0]

def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i, row in enumerate(map_array):
    for j, col in enumerate(map_array):
      map_array[i][j] = int(random.random()*1.25)
  return new_map

def load_img(img_filename):
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
    grid = np.zeros([800, 1200])
    return grid

  img = Image.open(img_filename)
  grid = np.zeros((img.height, img.width,3), dtype=np.uint8)
  for y in range(img.height):
    for x in range(img.width):
      p = img.getpixel((x, y))
      grid[y, x] = [int(i) for i in p]
  #print(grid[0])
  return grid

def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
      grid = np.zeros([800,1200])
      return grid

  img = Image.open(img_filename)

  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height

  grid = np.zeros([img.height, img.width])
  for y in range(img.height):
      for x in range(img.width):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

  return grid


def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map
  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(i // g_MAP_RESOLUTION_X), int(j // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  if vertex_source == vertex_dest:
    return 0, [vertex_dest]

  global g_WORLD_MAP
  m = copy.deepcopy(g_WORLD_MAP)
  for i, row in enumerate(m):
    for j, col in enumerate(row):
      if col == 1:
        m[i][j] = 10000
      if col == 0:
        m[i][j] = 1000
  j1, i1 = vertex_source
  j2, i2 = vertex_dest

  ### check if source/dest valid ###
  try:
    if m[i1][j1] > 1000:
      print("Starting index on a wall")
      return 1000, []
  except:
    print("Starting index out of bound")
    return 1000, []
  try:
    if m[i2][j2] > 1000:
      print("Target index on a wall")
      return 1000, []
  except:
    print("Target index out of bound")
    return 1000, []

  # start of magic algorithm
  parent = {}
  cost = {}
  cost[i1,j1] = 0
  parent[i1,j1] = i1,j1

  #a heapq with smallest val always in front
  h = []
  heapq.heappush(h, [0, [i1,j1]])

  row_size = len(m)
  col_size = len(m[0])
  while len(h) > 0:
    #always poping lowest cost coord from queue
    temp = heapq.heappop(h)
    if temp == (i2, j2):
      return cost[vertex_dest], reconstruct_path(parent, (i1, j1), (i2, j2))
    i, j, c = temp[1][0], temp[1][1], temp[0]

    #put surrounding tile to list
    neighbors = []
    if i > 0: neighbors.append([i - 1, j])
    if j > 0: neighbors.append([i, j - 1])
    if i < row_size - 1: neighbors.append([i + 1, j])
    if j < col_size - 1: neighbors.append([i, j + 1])

    #update all neighbors
    for ni,nj in neighbors:
      #do not expand parent or wall
      if parent[i,j] == (ni,nj) or m[ni][nj] > 1000:
        pass

      else:
        #expand if there is lower cost, or never expanded
        if (ni,nj) in cost:
          if cost[ni,nj] > c+1:
            cost[ni, nj] = c+1
            parent[ni,nj] = i,j
            if [c+1, [ni, nj]] not in h:
              heapq.heappush(h, [c+1, [ni, nj]])
        else:
          cost[ni, nj] = c + 1
          parent[ni, nj] = i, j
          if [c + 1, [ni, nj]] not in h:
            heapq.heappush(h, [c + 1, [ni, nj]])

  path = reconstruct_path(parent, (i1, j1), (i2, j2))

  if len(path) == 0:
    return 1000, path
  else:
    return cost[i2,j2], path

def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''

  # TODO: Insert your code here
  final_path = []
  if dest_vertex in prev:
    temp = dest_vertex
    final_path.append((dest_vertex[1],dest_vertex[0]))
    while prev[temp] != temp:
      final_path = [(prev[temp][1],prev[temp][0])] + final_path
      temp = prev[temp]

  return final_path

def find_heading(prev, curr):
  #return Right, Left, Up, Down, " "
  x1,y1 = prev
  x2,y2 = curr
  if x2 > x1: return "R"
  if x2 < x1: return "L"
  if y2 > y1: return "U"
  if y2 < y1: return "D"
  return " "


def find_waypoint(path):
  if len(path) <= 1:
    return path

  heading = " "
  prev = path[0]
  waypoints = []
  for p in path:
    h = find_heading(prev, p)
    if h != heading:
      waypoints.append(prev)
      heading = h
    prev = p
  waypoints.append(path[-1])

  return waypoints

def pixle_to_coord(path, max_xy, max_pixle):
  x, y = max_xy
  p_x, p_y = max_pixle
  new_path = []
  r_x, r_y = x/p_x, y/p_y
  for i,j in path:
    new_path.append([i*r_x, j*r_y])
  return new_path

def render_map(map_array,start, dest):

  l = len(map_array)
  w = len(map_array[0])
  for i, row in enumerate(map_array[::-1]):
    r = str(l-i-1)+": "
    for j, col in enumerate(row):
      if col == 1: r += "["
      else: r += " "

      if (j,l-i-1) == start: r += "S"
      elif (j,l-i-1) == dest: r += "T"
      elif col == 0: r += "."
      else: r += " "

      if col == 1: r += "]"
      else: r += " "
    print r
  r = "r/c "
  for j in range(w):
    r += str(j)+": "
  print r

def render_map_2(data, path):

  h, w = len(data), len(data[0])
  count = 4
  for y in range(h):
    for x in range(w):
      if (x,y) in path:
        count -= 1
        if count > 0:
          data[y, x] = [255, 0, 0]
        else:
          data[y, x] = [0, 255, 0]
          if count <= -3:
            count = 4

  img = Image.fromarray(data, 'RGB')
  img.save('path.png')
  img.show()

def render_map_3(data):
  h, w = len(data), len(data[0])

  grid = np.zeros((h, w, 3), dtype=np.uint8)
  for y in range(h):
    for x in range(w):
      if data[y][x] == 1:
        grid[y, x] = [0 for _ in grid[y, x]]
      else:
        grid[y, x] = [255 for _ in grid[y, x]]

  img = Image.fromarray(grid, 'RGB')
  img.save('grid.png')
  img.show()


def part_1():
  global g_WORLD_MAP

  #setup start, dest
  start, dest = (0,0), (7,8)
  print ""
  print("Part 1 (start,dest is hard coded in part_1):")
  print "Starting from:", start, " Ending at:", dest
  m = create_test_map(g_WORLD_MAP)
  render_map(m,start, dest)

  cost, path = get_travel_cost(start, dest)
  if cost == 1000:
    cost = -1
  print "Cost(-1 when not possible):", cost
  print "Path:", path
  print "Waypoint", find_waypoint(path)

def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP

  g_src_coordinates = (args.src_coordinates[0], args.src_coordinates[1])
  g_dest_coordinates = (args.dest_coordinates[0], args.dest_coordinates[1])
  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)
  '''
  TODO -
  1) Compute the g_WORLD_MAP -- depending on the resolution, you need to decide if your cell is an obstacle cell or a free cell.
  2) Run Dijkstra's to get the plan
  3) Show your plan/path on the image
  Feel free to add more helper functions
  '''
  #### Your code goes here ####
  l, w = len(pixel_grid), len(pixel_grid[0])
  start = int(float(g_src_coordinates[0])/1.8*w), l-1-int(float(g_src_coordinates[1])/1.2*l)
  dest = int(float(g_dest_coordinates[0])/1.8*w), l-1-int(float(g_dest_coordinates[1])/1.2*l)
  #print ""
  print("Part 2:")
  print "Image Size:", w, l
  print "Starting from:", ((g_src_coordinates[0]),(g_src_coordinates[1])), \
    "Ending at:", ((g_dest_coordinates[0]),(g_dest_coordinates[1]))

  print("Grid map shown in black and white form")
  print("Path drawn on picture with alternating red/green line")

  for i, row in enumerate(pixel_grid):
    for j, col in enumerate(row):
      if col > 160:
        pixel_grid[i][j] = 1
      else:
        pixel_grid[i][j] = 0

  g_WORLD_MAP = pixel_grid

  #display obstacle map generated by image
  #render_map_3(pixel_grid)

  #image for display purpose
  image = load_img(args.obstacles)
  cost, path = get_travel_cost(start, dest)

  #print waypoints
  wp = find_waypoint(path)
  wp = pixle_to_coord(wp, [1.8, 1.2], [w, l])
  print "Waypoints:", wp

  #render map with path
  render_map_2(image, path)
  if len(path) == 0:
    cost = -1
  print "Cost (in pixels)(-1 if not possible):", cost

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[3, 1], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[1, 3], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  # print ("This is the name of the script: ", sys.argv[0])
  # print ("Number of arguments: ", len(sys.argv))
  #print ("The arguments are: ", sys.argv)
  # print (args)

  #part_1()
  part_2(args)
