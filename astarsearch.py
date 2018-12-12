from Queue import PriorityQueue


##Sunanth Sakthivel 
##CS541, Programming 1

##Program should only be run on python 2.7.
##run program by: python astar.py
##follow the prompts to run the search and get results.

##this is the state node that will carry information such as the path, parent,
##its current value. This node can be expanded to children which lead to
##other paths.
class state_info(object):
    def __init__(self, search, value, parent, goal):
        self.goal = goal
        self.search = search
        self.value = value
        self.parent = parent
        self.children = []
        self.dist = 0

        if not parent:
            self.path = [value]
        else:
            self.path = parent.path[:]
            self.path.append(value)

        ##Based on selection, the respective search alg will be used to calc
        ##the evaluation function. F(x) = g(x) + h(x) for A* searches and
        ##F(x) = h(x) for greedy best first searches.
        self.g = self.getg()
        if self.search == 1:
            self.dist = self.misplaced() + self.g
        elif self.search == 2:
            self.dist = self.manhattan() + self.g
        elif self.search == 3:
            self.dist = self.rowpluscol() + self.g
        elif self.search == 4:
            self.dist = self.misplaced()
        elif self.search == 5:
            self.dist = self.manhattan()
        elif self.search == 6:
            self.dist = self.rowpluscol()

    ##this is to get the g(x) value for the current node.
    def getg(self):
        if self.parent == 0:
            return 0
        else:
            return self.parent.g + 1

    ##this is to get the h(x) with misplaced tiles heuristic, not including blank tile
    def misplaced(self):
        if self.value == self.goal:
            return 0
        
        dist = 0
        for x in range(0,9):
            if self.value[x] != self.goal[x] and self.value[x] != 'b':
                dist += 1
        return dist

    ##this is to get the h(x) with the manhattan distance heuristic, not including blank tile
    def manhattan(self):
        if self.value == self.goal:
            return 0

        temp = self.value.replace('b','9')
        templist = list(temp)
        tiles = list(map(int,templist))
        dist = 0
        
        for i in range(0,9):
            if tiles[i] != 9:
                dist += abs(abs((tiles[i]-1) - i%3) + abs((tiles[i]-1)//3 - i//3))
        
        return dist

    ##this is to get the h(x) with the tiles in wrong row plus tiles in wrong col
    ##heuristic, not including blank tile
    def rowpluscol(self):
        if self.value == self.goal:
            return 0
        temp = self.value.replace('b','9')
        templist = list(temp)
        tiles = list(map(int,templist))

        row = 0
        for i in range(0,9):
            if tiles[i] != 9: 
                if i >= 0 and i <= 2:
                    if not (tiles[i] >= 1 and tiles[i] <= 3):
                        row += 1
                elif i > 2 and i <= 5:
                    if not (tiles[i] >= 4 and tiles[i] <= 6):
                        row += 1
                elif i > 5 and i <= 8:
                    if not (tiles[i] >= 7 and tiles[i] <= 9):
                        row += 1

        col = 0
        for i in range(0,9):
            if tiles[i] != 9:
                if i%3 != (tiles[i] - 1)%3:
                    col += 1
                    
        dist = row + col
        return dist

    ##blank tile moving right
    def moveright(self,blank):
        temp = list(self.value)
        temp[blank] = self.value[blank+1]
        temp[blank+1] = 'b'
        val = "".join(temp)
        return val

    ##blank tile moving left
    def moveleft(self,blank):
        temp = list(self.value)
        temp[blank] = self.value[blank-1]
        temp[blank-1] = 'b'
        val = "".join(temp)
        return val

    ##blank tile moving up
    def moveup(self,blank):
        temp = list(self.value)
        temp[blank] = self.value[blank-3]
        temp[blank-3] = 'b'
        val = "".join(temp)
        return val

    ##blank tile moving down
    def movedown(self,blank):
        temp = list(self.value)
        temp[blank] = self.value[blank+3]
        temp[blank+3] = 'b'
        val = "".join(temp)
        return val

    ##This function expands the node such that there is now children for the
    ##expanded node. If node is at goal state then no expansion made.
    def expandnode(self):
            if self.value == self.goal:
                return
            
            ##figure out where blank tile is to determine different poss states.
            blank = 0
            temp = ''
            for x in range(0,9):
                if self.value[x] == 'b':
                    blank = x
                    break
            
            ##below are all possible expansions that take place depending where
            ##the blank tile is currently located.
            if blank == 0:
            ##down and right
               val = self.moveright(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.movedown(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 1:
            ##left, down, and right
               val = self.moveright(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.movedown(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveleft(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 2:
            ##left and down
               val = self.movedown(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveleft(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 3:
            ##up, right and down
               val = self.movedown(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveright(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveup(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 4:
            ##left, up, right, and down
               val = self.moveleft(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveup(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveright(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.movedown(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 5:
            ##up, left and down
               val = self.movedown(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveleft(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveup(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 6:
            ##up and right
               val = self.moveup(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveright(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 7:
            ##left, up and right
               val = self.moveleft(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveup(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveright(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
            elif blank == 8:
            ##up and left
               val = self.moveup(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)
               val = self.moveleft(blank)
               child = state_info(self.search,val,self,self.goal)
               self.children.append(child)


##this class essentially uses a priority queue and repeatedly expands the 
##suitable nodes untill a solution is found for a selected search.
class Search_Alg:
    def __init__(self,start,goal,search):
        self.search = search
        self.visited = []   ##need visited list to make sure no infinite loops
        self.priorityqueue = PriorityQueue()
        self.start = start
        self.path = []
        self.goal = goal

    def calculate(self):
        countid = 0 ##need countid to deal with priorityqueue tiebreaks
        firststate = state_info(self.search, self.start, 0, self.goal)
        self.priorityqueue.put((0,countid,firststate))
        while(self.priorityqueue.qsize() and not self.path):
            close_child = self.priorityqueue.get()[2]
            close_child.expandnode()
            self.visited.append(close_child.value)
            ##this signifies the end of the path and no other choice is suitable.
            if not close_child.children:
                self.path = close_child.path
                break
            ##if queue ever reaches above 30000 its safe to say there is no 
            ##solution so we can exit; this signifies a very large expansion.
            if self.priorityqueue.qsize() > 30000:
                self.path = []
                break
            else:
                for child in close_child.children:
                    if child.value not in self.visited:
                        countid += 1
                        self.priorityqueue.put((child.dist,countid,child))
        if not self.path:
            print "No path found"
        return self.path



##this is the main function that will prompt the user to enter in a valid 
##start state. The user is then asked to choose which search alg to use.
if __name__ == "__main__":

    print 'Enter valid start state (ex: b12345678) or type "default" for 5 default starting states:'
    start = raw_input()

    print 'Enter number for corresponding type of search:'
    print '1. A* search with misplaced tile hueristic'
    print '2. A* search with manhattan distance hueristic'
    print '3. A* search with # tiles out of row + # tiles out of col hueristic'
    print '4. Greedy best first search with misplaced tile hueristic'
    print '5. Greedy best first search with manhattan distance hueristic'
    print '6. Greedy best first search with # tiles out of row + # tiles out of col hueristic'
    search = int(raw_input())
    
    goal = "12345678b"

    print 'starting search please wait (impossible sol will take longer time to process)'
    print ' '

    if start != 'default':
        a1 = Search_Alg(start,goal,search)
        getpath = a1.calculate()



        for i in xrange(len(getpath)):
            print "(" + getpath[i] + ")->", 
        
        print ' '
        print "TOTAL STEPS: %d"  %(len(getpath) - 1)
    ##if default is chosen then automated testing of 5 different start states
    ##is tested with solutions for the choosen search.
    else:
        start1 = 'b12345678'
        start2 = '41563278b'
        start3 = '8146b5237'
        start4 = '1762b5438'
        start5 = '43671852b'

        a1 = Search_Alg(start1,goal,search)
        getpath = a1.calculate()
        for i in xrange(len(getpath)):
            print "(" + getpath[i] + ")->",
        print ' '
        print "Total STEPS: %d" %(len(getpath) - 1)
        print '-------------------------------------'
        a2 = Search_Alg(start2,goal,search)
        getpath = a2.calculate()
        for i in xrange(len(getpath)):
            print "(" + getpath[i] + ")->",
        print ' '
        print "Total STEPS: %d" %(len(getpath) - 1)
        print '-------------------------------------'
        a3 = Search_Alg(start3,goal,search)
        getpath = a3.calculate()
        for i in xrange(len(getpath)):
            print "(" + getpath[i] + ")->",
        print ' '
        print "Total STEPS: %d" %(len(getpath) - 1)
        print '-------------------------------------'
        a4 = Search_Alg(start4,goal,search)
        getpath = a4.calculate()
        for i in xrange(len(getpath)):
            print "(" + getpath[i] + ")->",
        print ' '
        print "Total STEPS: %d" %(len(getpath) - 1)
        print '-------------------------------------'
        a5 = Search_Alg(start5,goal,search)
        getpath = a5.calculate()
        for i in xrange(len(getpath)):
            print "(" + getpath[i] + ")->",
        print ' '
        print "Total STEPS: %d" %(len(getpath) - 1)
        print '-------------------------------------'








