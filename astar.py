import pygame
import math
from queue import PriorityQueue

WIDTH = 375
HIEGHT = 600
WIN = pygame.display.set_mode((WIDTH, HIEGHT))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Spot:
	def __init__(self, row, col, width, hieght, total_rows, total_col):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * hieght
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.hieght = hieght
		self.total_rows = total_rows
		self.total_col = total_col

	def get_pos(self):
		return self.row, self.col

	def is_closed(self):
		return self.color == RED

	def is_open(self):
		return self.color == GREEN

	def is_barrier(self):
		return self.color == BLACK

	def is_start(self):
		return self.color == ORANGE

	def is_end(self):
		return self.color == TURQUOISE
		
	def is_path(self):
		return self.color == PURPLE

	def reset(self):
		self.color = WHITE

	def make_start(self):
		self.color = ORANGE

	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = GREEN

	def make_barrier(self):
		self.color = BLACK

	def make_end(self):
		self.color = TURQUOISE

	def make_path(self):
		self.color = PURPLE

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.hieght))

	def update_neighbors(self, grid):
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.col < self.total_col - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
			self.neighbors.append(grid[self.row][self.col - 1])

	def __lt__(self, other):
		return False


def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, start, draw, ROWS):
	path = "*###*"
	past_dif = 0
	curt_dif = 0
	last_spot = 0
	present_spot = 0
	start_no = spot_number(start,ROWS)
	end = spot_number(current,ROWS)
	while current in came_from:
		present_spot = spot_number(current,ROWS)
		if present_spot == end:
			path += str(present_spot)
			path += "*"
		elif present_spot - end == 1 or end - present_spot == 1 or present_spot - end == 25 or end - present_spot == 25:
			curt_dif = present_spot - last_spot
		elif present_spot - start_no == 1 or start_no - present_spot == 1 or present_spot - start_no == 25 or start_no - present_spot == 25:
			curt_dif = start_no - present_spot
			if past_dif != curt_dif:
				path += str(present_spot)
				path += "*"
		else:
			curt_dif = present_spot - last_spot
			if past_dif != curt_dif:
				path += str(last_spot)
				path += "*"
		past_dif = curt_dif
		last_spot = present_spot
		current = came_from[current]
		current.make_path()
		draw()
	print(path)

def spot_number(box,TOTAL_COL):
	return (box.row+1) + ((box.col)*TOTAL_COL)
	
def spot_rc(spot_no,TOTAL_COL,grid):
	row = spot_no % TOTAL_COL
	col = spot_no // TOTAL_COL
	if row == 0:
		row = 25
		col -= col
	return grid [row -1] [col]
	

def algorithm(draw, grid, start, end, ROWS):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_pos(), end.get_pos())

	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[2]
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end, start, draw, ROWS)
			end.make_end()
			start.make_start()
			return True

		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()

		draw()

		if current != start:
			current.make_closed()

	return False


def make_grid(rows, col, width, hieght):
	grid = []
	gap1 = width // rows
	gap2 = hieght // col
	for i in range(rows):
		grid.append([])
		for j in range(col):
			spot = Spot(i, j, gap1, gap2, rows, col)
			grid[i].append(spot)

	return grid


def draw_grid(win, rows, col, width, hieght):
	gap1 = width // rows
	gap2 = hieght // col
	for i in range(col):
		pygame.draw.line(win, GREY, (0, i * gap1), (width, i * gap1))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap2, 0), (j * gap2, hieght))


def draw(win, grid, rows, col, width, hieght):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, col, width, hieght)
	pygame.display.update()


def get_clicked_pos(pos, rows, col, width, hieght):
	gap1 = width // rows
	gap2 = hieght // col
	
	y, x = pos

	row = y // gap1
	col = x // gap2

	return row, col
	
def get_spot(path):
	a = len(path) - 1
	last = 0
	present = 0
	while a >= 0:
		b = path[a]
		if b == "*":
			present = a
			if last == 0:
				last = present
			else: 
				print(path[present + 1:last])
				last = present
		a -= 1


def main(WIN, WIDTH, HIEGHT):
	ROWS = 25
	COL = 40
	grid = make_grid(ROWS, COL, WIDTH, HIEGHT)
	
	s = int(input("Enter Start Box : "))
	start = spot_rc(s,ROWS,grid)
	end = None

	run = True
	while run:
		draw(WIN, grid, ROWS, COL, WIDTH, HIEGHT)
		start.make_start()
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

			if pygame.mouse.get_pressed()[0]: # LEFT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, COL, WIDTH, HIEGHT)
				spot = grid[row][col]
				if not start and spot != end:
					start = spot
					start.make_start()

				elif not end and spot != start:
					end = spot
					end.make_end()

				elif spot != end and spot != start:
					spot.make_barrier()

			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, COL, WIDTH, HIEGHT)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None
				elif spot == end:
					end = None

			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					for row in grid:
						for spot in row:
							spot.update_neighbors(grid)

					algorithm(lambda: draw(WIN, grid, ROWS, COL, WIDTH, HIEGHT), grid, start, end, ROWS)

				"""if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, COL, WIDTH, HIEGHT)"""

	pygame.quit()

main(WIN, WIDTH, HIEGHT)
