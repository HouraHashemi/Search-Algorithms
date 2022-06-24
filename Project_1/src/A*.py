import os
import DFS
import numpy as np
import Heuristic
from time import sleep


class butterFinderAStar:

	info = {'r':[], 'b':[], 'p':[], 'x':[] }

	graph = {}
	file  = []
	data  = []
	dimantion  = []

	cost = 0
	depth = 0

	# read file and creade data of it
	def __init__(self,file_name):
		self.file_name = file_name
		self.file = open(self.file_name, "r")
		self.dimantion  = self.file.readline().split()
		self.data = [self.file.readline().split() for line in range(int(self.dimantion[0]))]
		self.grid = {}
		self.load_grid()

	# create grid from data
	def load_grid(self):		
		for row in range(int(self.dimantion[0])):
			for col in range(int(self.dimantion[1])):
				self.grid[(row,col)] = self.data[row][col]
				if "r" in self.data[row][col]:
					self.info['r'].append((row,col))
				if "b" in self.data[row][col]:
					self.info['b'].append((row,col))
				if "p" in self.data[row][col]:
					self.info['p'].append((row,col))
				if "x" in self.data[row][col]:
					self.info['x'].append((row,col))

	# find goal by type and root
	def find_goal(self,root,goal,forbidden):
		g = DFS.DFS_Graph(root,self.grid, goal)
		heur = Heuristic.Heuristic(root,self.grid, goal)

		level = 0
		parent = root
		already_expanded = {parent,}
		edge_list = {parent,}
		path = []

		block_list = []
		for fo in forbidden:
			if type(fo)==tuple:
				block_list = block_list + [fo]
			else:
				block_list = block_list + self.info[fo]
		# print("block list: ".format(block_list))

		while ( (len(already_expanded)+len(block_list)) < (int(self.dimantion[0])*int(self.dimantion[1])) ):
			# print("level:{}".format(level))
			# temp_children: for expanded children which should replace
			temp_children = set()
			# find all of children of new parents
			for edge in edge_list:
				if heur.find_goal == True:
					break
				parent = edge
				expanded = self.expand(parent,already_expanded,block_list)
				temp_children.update(expanded)
				# add to graph
				heur.increase_level()
				for ex in expanded:
					g.addEdge(parent,ex)
					heur.addNode(parent,ex)
					if heur.find_goal == True:
						break
			# heuristic class filtered expanded node
			nxg = heur.next_generation()
			# print("next_generation:{}".format(nxg))

			# print("----------------------")
			# update edge list by chosen leafs
			edge_list = nxg
			already_expanded.update(temp_children)

			if heur.find_goal == True:
				break
			# print("already_expanded:{}".format(already_expanded))
			level = level + 1
			# print("======================")

		# print("already_expanded:{}".format(already_expanded)) 
		path = g.DFS(root)
		if(path!=[]):
			path = path[::-1]

		self.cost = self.cost + heur.mincost_total_path	
		self.depth = self.depth + level

		return path


	# expand nodes by parents and ignore items include block list type
	def expand(self,parent,already_expanded,block_list):
			# print("parent: ".format(parent))
			children = set()
			temp_child = [(parent[0],parent[1]-1),(parent[0]-1,parent[1]),(parent[0],parent[1]+1),(parent[0]+1,parent[1])]
			for child in temp_child:
				# print("child: ".format(child))
				if (child[0]>=0 and child[0]<int(self.dimantion[0]) and child[1]<int(self.dimantion[1]) and child[1]>=0 and (child not in list(already_expanded))):
					if child not in block_list:
						children.add(child)
			# print("-------------------------")
			# print("children: ".format(children))
			return children	


	# search path by coordinate and concat sg-gp
	def move_path(self,start_to_goal, goal_to_point):
		total_path = start_to_goal[:-1]

		agent_position = start_to_goal[-2]
		goal_position = goal_to_point[0]
		next_move = goal_to_point[1]
		# print("total_path:{} agent_position:{} goal_position:{} next_move:{}".format(total_path,agent_position,goal_position,next_move))

		step = 0
		while (step != len(goal_to_point)-1):
			# check if agent and nextmove in same direction
			if (agent_position[0]==next_move[0]) or (agent_position[1]==next_move[1]):
				# print("total_path:{} agent_position:{} goal_position:{} next_move:{}".format(total_path,agent_position,goal_position,next_move))
				total_path.append(goal_position)
				agent_position = goal_position
				goal_position = next_move
				if step+2 < len(goal_to_point):
					next_move = goal_to_point[step + 2]
				# print("total_path:{} agent_position:{} goal_position:{} next_move:{}".format(total_path,agent_position,goal_position,next_move))
				step = step + 1

			else:
				new_agent_position = tuple()
				# agent and nextmove in same row
				if goal_position[0]==next_move[0]:
					# |        ||agent   ||        |
					# |nextmove||goal    ||        |
					# |        ||agent   ||        |
					if goal_position[1] < next_move[1]:
						new_agent_position = (next_move[0],next_move[1]-2)
					# |        ||agent   ||        |
					# |        ||goal    ||nextmove|
					# |        ||agent   ||        |
					else:
						new_agent_position = (next_move[0],next_move[1]+2)

				# agent and nextmove in same column
				elif goal_position[1]==next_move[1]:
					# |        ||        ||        |
					# |agent   ||goal    ||agent   |
					# |        ||nextmove||        |
					if goal_position[0] < next_move[0]:
						new_agent_position = (next_move[0]-2,next_move[1])
					# |        ||nextmove||        |
					# |agent   ||goal    ||agent   |
					# |        ||        ||        |
					else:
						new_agent_position = (next_move[0]+2,next_move[1])
				
					# print("new_agent_position:{}".format(new_agent_position))
				if (new_agent_position[0]<0 or new_agent_position[0]>int(self.dimantion[0]) or new_agent_position[1]>int(self.dimantion[1]) or new_agent_position[1]<0 or self.grid[new_agent_position] == "x"):
					# add header (-1,-1) to show cant find any path
					total_path = [(-1,-1)] + total_path
					break
				
				extra_path = self.find_goal(agent_position,new_agent_position,['x','p','b',goal_position])
				# print("extra_path:{}".format(extra_path))
				total_path = total_path + extra_path[1:]
				agent_position = new_agent_position
				# print("total_path:{} agent_position:{} goal_position:{} next_move:{}".format(total_path,agent_position,goal_position,next_move))

		if total_path[-1]!=goal_to_point[-2]:
			total_path = [(-1,-1)] + total_path

		return total_path


	# search path by direction [L,U,D,R]
	def direction_path(self,coordinatet_path):
		direc_path = []
		move = {(-1,0):"D", (0,1):"L", (1,0):"U", (0,-1):"R",(0,0):"N"}
		for c in range(len(coordinatet_path)-1):
			row = coordinatet_path[c][0]-coordinatet_path[c+1][0]
			col = coordinatet_path[c][1]-coordinatet_path[c+1][1]
			if (row,col)!=(0,0):
				direc_path.append(move[(row,col)])
		return direc_path


	# action | mp: move-path | cp: cordinate-path 
	def play_motion(self,mp,cp):

		rmap = self.grid
		for n in range(len(cp)):
			# cp[n] is the position robot going to get it
			if "b" in rmap[cp[n]]:
				glp = tuple()
				# find the direction of butter which is going to move by robot
				if   mp[n-1]=="R":
					glp = (cp[n][0],cp[n][1]+1)
				elif mp[n-1]=="L":
					glp = (cp[n][0],cp[n][1]-1)
				elif mp[n-1]=="D":
					glp = (cp[n][0]+1,cp[n][1])
				elif mp[n-1]=="U":
					glp = (cp[n][0]-1,cp[n][1])
				# robot moves the butter
				rmap[glp] = rmap[glp] + "b"
				rmap[cp[n]] = rmap[cp[n]].split("b")[0]
			# robot moves and clear cell behind it
			rmap[cp[n]] = rmap[cp[n]]+"r"
			rmap[cp[n-1]] = rmap[cp[n-1]].split("r")[0]
			# print the map
			self.print_map()
			sleep(1)
			print("----------------------------------")
			os.system('cls' if os.name == 'nt' else 'clear')


	# print graphical schematic
	def print_map(self):
		for row in range(int(self.dimantion[0])):
			for col in range(int(self.dimantion[1])):
				if   "r" in self.grid[(row,col)]:
					print("🤖",end="")
				elif "b" in self.grid[(row,col)]:
					print("🍰",end="")
				elif "p" in self.grid[(row,col)]:
					print("😀",end="")
				elif "x" in self.grid[(row,col)]:
					print("⬛",end="")
				else:
					print("🔲",end="")
			print("")

#---------------------------------------------------------
# END OF CLASS
#---------------------------------------------------------


if __name__ == "__main__":

	emoji = ["🍹","🍰", "🍕","🔲","⬛","⬜","🤖","🍪","🧂","☕"]

	a_star = butterFinderAStar("input//test3.txt")
	print(np.array(a_star.data))
	print("____________________________________\n")

	tot = []

	for b in range(len(a_star.info['b'])):
		a_star.print_map()
		print("====================================")
		robot  = a_star.info['r'][0]
		butter = a_star.info['b'][b]
		person = a_star.info['p'][b]

		robot_to_butter_path  = []
		butter_to_person_path = []
	
		robot_to_butter_path = a_star.find_goal(robot,'b',['x','p'])
		print("ROBOT TO BUTTER PATH: {}".format(robot_to_butter_path))
		print("------------------------------------")

		butter_to_person_path = a_star.find_goal(butter,'p',['x','b'])
		print("BUTTER TO PERSON PATH: {}".format(butter_to_person_path))
		print("------------------------------------")

		tot = a_star.move_path(robot_to_butter_path,butter_to_person_path)
		print("TOTAL PATH: {}".format(tot))
		print("====================================")

		# check for exist any path
		if tot[0]==(-1,-1):
			print("{} CAN NOT FIND ANY PATH FOR ROBOT{}-BUTTER{}-PERSON{}".format("⚠️ ",robot,butter,person))
		else:
			dp = a_star.direction_path(tot)
			print("DIRECTIONAL PATH: {}".format(dp))
			print("TOTAL COST: {}".format(a_star.cost))
			print("DEPTH: {}".format(a_star.depth))
			print("🔔 ] THE PATH IS FOUND.START? [Y/N]")
			run = input("🔅 ] ")
			if run == "y" or run == "Y":
				os.system('cls' if os.name == 'nt' else 'clear')
				a_star.play_motion(dp,tot)

				# clear cell that robot was there
				a_star.grid[robot_to_butter_path[0]] = a_star.grid[robot_to_butter_path[0]].split('r')[0]
				# move robot to current destination cell 
				a_star.grid[tot[-1]] = a_star.grid[tot[-1]]+'r'
				a_star.info['r'][0] = tot[-1]

				# clear cell that butter was there
				a_star.grid[butter_to_person_path[0]] = a_star.grid[butter_to_person_path[0]].split('b')[0]
				# move butter to current destination cell 
				a_star.grid[butter_to_person_path[-1]]=a_star.grid[butter_to_person_path[-1]]+'b'
				a_star.info['b'][b] = butter_to_person_path[-1]
			else:
				print("[OK]")		
		print("___________________________________\n\n")

		a_star.depth = 0
		a_star.cost  = 0



