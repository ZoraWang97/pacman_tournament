from captureAgents import CaptureAgent
from util import nearestPoint
import random, util
from game import Directions
from game import Actions


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = ' OffensiveAgent', second = ' DefensiveAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########


################################### Offender ##############################

class OffensiveAgent(CaptureAgent):

  def registerInitialState(self, gameState):

    CaptureAgent.registerInitialState(self, gameState)
    self.walls = gameState.getWalls().asList()
    self.start = gameState.getAgentPosition(self.index)
    if self.red:
      self.homeDoors= [(int(gameState.data.layout.width/2)-1,y) for y in range (1, gameState.data.layout.height-1) if not gameState.hasWall(int(gameState.data.layout.width/2)-1,y)]
    else:
      self.homeDoors =[(int(gameState.data.layout.width/2),y) for y in range (1, gameState.data.layout.height-1) if not gameState.hasWall(int(gameState.data.layout.width/2),y)]


  def nearestFoodPos(self,gameState):
    foods = [f for f in self.getFood(gameState).asList()]
    foodsDistance = [self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), f) for f in foods]
    nrstFood = [f for f, fd in zip(foods, foodsDistance) if fd == min(foodsDistance)]
    if len(nrstFood) != 0:
      return nrstFood[0]
    else:
      return None
      
  def nearestCapsulePos(self,gameState):
    capsules = self.getCapsules(gameState)
    capsulesDistance = [self.getMazeDistance(gameState.getAgentState(self.index).getPosition(),c) for c in capsules]
    nrstCapsule = [c for c, cd in zip(capsules, capsulesDistance) if cd == min(capsulesDistance)] 
    if len(nrstCapsule) != 0:
      return nrstCapsule[0]
    else:
      return None

  def nearestDoorPos(self,gameState):
    homeDoorsDistance = [self.getMazeDistance(gameState.getAgentPosition(self.index),d) for d in self.homeDoors]
    nearestDoors = [d for d, dd in zip(self.homeDoors,homeDoorsDistance) if dd == min(homeDoorsDistance)] 
    return nearestDoors[0]

  def getOpOpponents (self,gameState):
    opponents = [gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    opOpponents = [o for o in opponents if o.getPosition() != None]
    return opOpponents

  def getOpDefenders(self,gameState):
    opponents = [gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    opDefenders = [d for d in opponents if d.getPosition() != None and not d.isPacman]
    return opDefenders

  def getOpOffenders(self,gameState):
    opponents = [gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    opOffenders = [o for o in opponents if o.getPosition() != None and o.isPacman]
    return opOffenders

  def getSafeSuccessors(self, gameState, myPos):
    successors = []
    for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
      x,y = myPos
      dx, dy = Actions.directionToVector(action)
      nx, ny = int(x + dx), int(y + dy)
      walls = gameState.getWalls().asList()
      #walls + [x,y for x in range(0,)if x == 0 or y = 0 or x == gameState.data.layout.width or y == gameState.data.layout.height]
      if (nx, ny) not in walls: #and nx in range(0,gameState.data.layout.width) and ny in range(0,gameState.data.layout.height-1):
        successors.append(((nx,ny),action,1))
    return successors

  def opHeuristic(self,pos,gameState):
    heuristics = []
    if len(self.getOpOpponents(gameState)) != 0:  #gameState.getAgentState(self.index).isPacman and
      for d in self.getOpOpponents(gameState):
        if self.getMazeDistance(pos,d.getPosition()) < 1:
          heuristics.append(800)
        elif self.getMazeDistance(pos,d.getPosition()) < 2:
          heuristics.append(300)
        elif self.getMazeDistance(pos,d.getPosition()) < 3:
          heuristics.append(80)
        else:
          heuristics.append(0)
      return max(heuristics)
    else:
      return 0

  def aStarSearchOp(self,gameState,goal):
    myPos = gameState.getAgentPosition(self.index)
    asPQueue = util.PriorityQueue() 
    visited = []
    asPQueue.push((myPos,[]),0)
    while not asPQueue.isEmpty():
      (node,action) = asPQueue.pop()
      if node == goal and len(action)!=0:
        return action[0]
      if node not in visited:
        visited.append(node)
        successors = self.getSafeSuccessors(gameState,node)
        for (nodeN,actionN,costN) in successors:
          if nodeN not in visited:
            asPQueue.push((nodeN, action+[actionN]),len(action+[actionN])+self.opHeuristic(nodeN,gameState))#+util.manhattanDistance(myPos,goal)
    while goal != gameState.getAgentPosition(self.index):
      return self.aStarSearchOp(gameState,gameState.getAgentPosition(self.index))
    return Directions.STOP


  def aStarSearchMH(self,gameState,goal):
    myPos = gameState.getAgentPosition(self.index)
    asPQueue = util.PriorityQueue() 
    visited = []
    asPQueue.push((myPos,[]),0)
    while not asPQueue.isEmpty():
      (node,action) = asPQueue.pop()
      if node == goal and len(action)!=0:
        return action[0]
      if node not in visited:
        visited.append(node)
        successors = self.getSafeSuccessors(gameState,node)
        for (nodeN,actionN,costN) in successors:
          if nodeN not in visited:
            asPQueue.push((nodeN, action+[actionN]),len(action+[actionN])+util.manhattanDistance(myPos,goal))
    while goal != gameState.getAgentPosition(self.index):
      return self.aStarSearchMH(gameState,gameState.getAgentPosition(self.index))
    return Directions.STOP


  def chooseAction(self,gameState):
    actions = gameState.getLegalActions(self.index)
    myPos = gameState.getAgentPosition(self.index)
    nearestCapsulePos = self.nearestCapsulePos(gameState)
    nearestFoodPos = self.nearestFoodPos(gameState)
    nearestDoor = self.nearestDoorPos(gameState)
    defenders = self.getOpDefenders(gameState)
    offenders = self.getOpOffenders(gameState)
    opponents = self.getOpOpponents(gameState)
    offendersDistance = [self.getMazeDistance(myPos,o.getPosition()) for o in offenders]
    defendersDistance = [self.getMazeDistance(myPos,d.getPosition()) for d in defenders]
    opponentsDistance = [self.getMazeDistance(myPos,o.getPosition()) for o in opponents]




    # when I am scared and the opponents are near by                                   
    if gameState.getAgentState(self.index).scaredTimer > 5 and len(opponentsDistance)!=0 and min(opponentsDistance) < 5:
      # find the nearestplace away from the pacman
      # thePacman = [o for o,od in zip(opponents,opponentsDistance) if o.isPacman]
      # pacmanPos = thePacman[0].getPosition()
      #do the action to be away from the pacman and 

      return self.aStarSearchOp(gameState,nearestDoor)
                                        #nearestDoor

    # when in home and defenders nearby
  # if gameState.getAgentPosition(self.index) in self.homeDoors and.....
    if not gameState.getAgentState(self.index).isPacman and len(defendersDistance)!=0 and min(defendersDistance) <5:
      if not gameState.getAgentState(self.index).scaredTimer > 0: # and len(self.getOpOffenders(gameState)) != 0 
        # remainDoors = [d for d in self.homeDoors if d != gameState.getAgentPosition(self.index)]
        # remainDoorsDistance = [self.getMazeDistance(gameState.getAgentPosition(self.index),d) for d in remainDoors]
        # nearestRemainDoor = [d for d, dd in zip(remainDoors,remainDoorsDistance) if dd == min(remainDoorsDistance)]
        # return self.aStarSearchOp(gameState,nearestRemainDoor[0])

        #return self.aStarSearchOp(gameState,offenders[0].getPosition())
        #return self.aStarSearchOp(gameState,offenders[0].getPosition())
        #return self.aStarSearchOp(gameState,self.homeDoors[0])

        if self.nearestCapsulePos(gameState) != None:
          return self.aStarSearchOp(gameState,nearestCapsulePos)
        else:
          return self.aStarSearchOp(gameState,(self.homeDoors)[0])

      return Directions.STOP

    # when I am in opponents' place and the defenders are near by
    if len(defendersDistance) != 0 and min(defendersDistance) <5 and gameState.getAgentState(self.index).isPacman :
      # if the defenders are scared, I can keep finding foods
      if False not in [d.scaredTimer > 10 for d in defenders]:
        if len(self.getFood(gameState).asList()) >2:
          if nearestCapsulePos != None and self.getMazeDistance(myPos,nearestCapsulePos) <= self.getMazeDistance(myPos,nearestFoodPos):
              return self.aStarSearchMH(gameState,nearestCapsulePos)
          return self.aStarSearchMH(gameState,nearestFoodPos)
        else:
            return self.aStarSearchMH(gameState,nearestDoor)
      else:# if the defenders are not scared or will not be scared soon
        if nearestCapsulePos != None :
          #if there is capsule and I can get it before the defender get me
          if len(defenders)<2 and self.getMazeDistance(myPos,nearestCapsulePos) < min(defendersDistance) and self.getMazeDistance(myPos,nearestCapsulePos)<self.getMazeDistance(myPos,nearestDoor) :
            #get the capsule and make the defender scared
            return self.aStarSearchMH(gameState,nearestCapsulePos)
        # if nearestFoodPos != None and self.getMazeDistance(myPos,nearestFoodPos)+self.getMazeDistance(nearestFoodPos,nearestDoor)<= min(defendersDistance):
        #   return self.aStarSearchMH(gameState,nearestFoodPos)

        return self.aStarSearchOp(gameState,nearestDoor)

    # # when in self place but in scare, keep away from the opponents
    # if gameState.getAgentState(self.index).scaredTimer > 5 and min(opponentsDistance) <= 3 and not gameState.getAgentState(self.index).isPacman:
    #   return self.aStarSearchOp(gameState,nearestFoodPos)

    # when offender near by on my way from home to food and I'm not scared
    if len(offendersDistance) != 0 and min(offendersDistance)<2 and not gameState.getAgentState(self.index).isPacman:
      return self.aStarSearchMH(gameState,offenders[0].getPosition())


    # when there are more than 2 food left, keep getting foods or capsules
    if len(self.getFood(gameState).asList()) >2:
      # but if time is running out and is carrying some food, return to home door first.
      if gameState.data.timeleft < 180 and gameState.getAgentState(self.index).numCarrying > 0:
        # when the score is still negative and the way to get food could still possible to make it back home
        if self.getScore(gameState)<0 and (self.getMazeDistance(nearestFoodPos,nearestDoor)+self.getMazeDistance(nearestFoodPos,nearestDoor))*10 < gameState.data.timeleft:
          return self.aStarSearchMH(gameState,nearestFoodPos)
        # otherwise, go back home directly
        return self.aStarSearchMH(gameState,nearestDoor)
      if nearestCapsulePos != None and not gameState.getAgentState(self.index).isPacman:
        return self.aStarSearchMH(gameState,nearestCapsulePos)
      if nearestCapsulePos != None and self.getMazeDistance(myPos,nearestCapsulePos) <= self.getMazeDistance(myPos,nearestFoodPos):
          return self.aStarSearchMH(gameState,nearestCapsulePos)
      return self.aStarSearchMH(gameState,nearestFoodPos)
    else:
        return self.aStarSearchMH(gameState,nearestDoor)


################################### Defender ##############################

class DefensiveAgent(CaptureAgent):
  
    def registerInitialState(self, gameState):
        self.start = (gameState.getAgentPosition(self.index))
        CaptureAgent.registerInitialState(self, gameState)
        self.homePos = []

        if not self.red:
            posX = int (gameState.data.layout.width / 2) 
        else:
            posX = int(gameState.data.layout.width / 2) -1
        for posY in range(1, gameState.data.layout.height):
            if not gameState.hasWall(posX, posY):
                self.homePos.append((posX, posY))

    def chooseAction(self, gameState):
        """
        Picks among the actions with the highest Q(s,a).
        """
        actions = gameState.getLegalActions(self.index)


        # start = time.time()
        values = [self.evaluate(gameState, a) for a in actions]


        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        return random.choice(bestActions)

    def getSuccessor(self, gameState, action):
        """
        Finds the next successor which is a grid position (location tuple).
        """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def evaluate(self, gameState, action):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        my_posision = successor.getAgentState(self.index).getPosition()
        (x, y) = self.start
        enemy_position =(gameState.data.layout.width-1-x, gameState.data.layout.height-1-y)
        food_list = self.getFoodYouAreDefending(successor).asList()
        if len(food_list) !=0:
            distance_list = []
            for i in food_list:
                distance = self.getMazeDistance(enemy_position, i)
                distance_list.append(distance)
            min_distance = min(distance_list)
            for food_position in food_list:
                dis = self.getMazeDistance(enemy_position, food_position)
                if min_distance == dis:
                    enemy_position = food_position
                    break
            dis_home_list = []
            for i in self.homePos:
                dis_home_list.append(self.getMazeDistance(enemy_position,i))
            min_dis_home = min(dis_home_list)
            for i in self.homePos:
                distance_home = self.getMazeDistance(enemy_position, i)
                if distance_home == min_dis_home:
                    target_posion = i
            features['distanceToHome'] = self.getMazeDistance(my_posision, target_posion)



        if successor.getAgentState(self.index).isPacman:
            features['defense'] = 0
        else:
            features['defense'] = 1

        enemy_list = []
        for i in self.getOpponents(successor):
            enemy_list.append(successor.getAgentState(i))
        invaders_list = []
        for i in enemy_list:
            if i.isPacman and i.getPosition() != None:
                invaders_list.append(i)
        features['invadersNum'] = len(invaders_list)
        if len(invaders_list) > 0:
            dist_invaders = []
            for i in invaders_list:
                dist_invaders.append(self.getMazeDistance(my_posision, i.getPosition()))

            features['invaderDist'] = min(dist_invaders)

        if action == Directions.STOP:
            features['stop']=1


        return features

    def getWeights(self, gameState, action):

        return {'stop': -10, 'distanceToHome': -5,'defense':100, 'invadersNum':-5000,'invaderDist':-1500}