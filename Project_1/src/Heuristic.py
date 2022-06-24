from collections import defaultdict


class Heuristic:

	mincost_total_path = 0
	level = 0
	find_goal = False
	path = []

	# Constructor
	def __init__(self,root,grid,goal):
		# default dictionary to store graph
		self.graph = defaultdict(list)
		self.grid = grid
		self.root = root
		self.goal = goal
		self.cost_graph = dict() 


	# change goal by tuple() coordinate
	def set_new_goal(self, goal):
		self.goal = goal


	# function to add an edge to graph
	def addNode(self, u, v):
		self.graph[u].append(v)
		cost = self.calculate_cost(v)
		# print("child:{} child_cost:{}".format(v,cost))
		if cost in self.cost_graph:
			self.cost_graph[cost].append(v)
		else:
			self.cost_graph[cost] = [v]
		# check for see if find goal or not
		if (str(self.goal) in self.grid[v]) or (self.goal == v):
			self.find_goal = True
		
		
	# increase level in graph
	def increase_level(self):
		self.level = self.level + 1


	# calculate_cost of each node
	def calculate_cost(self,child):
		if 'b' in self.grid[child]:
			ch_cost = int(self.grid[child].split('b')[0])
		elif 'p' in self.grid[child]:
			ch_cost = int(self.grid[child].split('p')[0])
		elif 'r' in self.grid[child]:
			ch_cost = int(self.grid[child].split('r')[0])
		else:
			ch_cost = int(self.grid[child])
		return ch_cost + self.level
	

	# find chosen next generation wich has minimum cost
	def next_generation(self):
		mincost = min([key for key in self.cost_graph])
		# print("mincost:{}".format(mincost))
		self.mincost_total_path = mincost
		# print("cost graph:{}".format(self.cost_graph))
		nxg = self.cost_graph[mincost]
		del self.cost_graph[mincost]
		# return next generation
		return nxg



