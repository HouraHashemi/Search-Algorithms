from collections import defaultdict
# This class represents a directed graph using
# adjacency list representation

class DFS_Graph:

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

	# fchange goal by tuple() coordinate
	def set_new_goal(self, goal):
		self.goal = goal

	# function to add an edge to graph
	def addEdge(self, u, v):
		self.graph[u].append(v)

	# A function used by DFS
	def DFSUtil(self, v, visited, claster):

		# Mark the current node as visited
		visited.add(v)
		# print(v)
				
		#check for goal/ if goal is (b or p) or (x,x)
		if (str(self.goal) in self.grid[v]) or (self.goal == v) :
			# print("this goal:", claster)
			find_goal = True
			node = v
			# agent stand before get the goal
			self.path.append(node)
			while (node!=self.root):
				self.path.append(claster[node])
				# print("node:{}\nclaster[node]:{}".format(node,claster[node]))
				node = claster[node]
		

		# Recur for all the vertices
		# adjacent to this vertex
		for neighbour in self.graph[v]:
			if neighbour not in visited:
				claster[neighbour] = v
				if self.find_goal == True :
					break	
				self.DFSUtil(neighbour, visited, claster)
		

	# The function to do DFS traversal. It uses recursive DFSUtil()
	def DFS(self, v):
		self.level = self.level + 1
		# child : parent
		claster = {} 
		# Create a set to store visited vertices
		visited = set()
		# Call the recursive helper function to print DFS traversal
		self.path = []
		self.DFSUtil(v, visited, claster)
		return self.path