#!/usr/bin/env python

import numpy as np

class BiRRTNode:
    # the startTree argument represents whether the node is in the start tree or the goal tree
    # parents should be defined such that, at the end
    # the start node has no parent, the goal node has no parent
    # and when we find either a node from each tree close to eachother
    # or a node that can connect the trees that's within `maxNodeDistance`, backpropogate from those
    # nodes to connect each tree
    def __init__(self, pose, startTree=True):
        self.pose = pose
        self.parent = None
        self.startTree = startTree

    def deltaClone(self, delta):
        clone = BiRRTNode(self.pose + delta, self.startTree)
        clone.parent = self
        return clone

class BiRRT:
    # assumes that the current state of fetch.getPose() is the starting pose
    # goalPose is a set of poses, not a 3D point
    # which means you have to do the inverse kinematics beforehand
    def __init__(self, fetch, goalPose, maxNodeDistance=0.01, connectionRate=0.02, maxProjections=1e10):
        self.fetch = fetch
        self.start = fetch.getPoses()  # TODO: extend getPose to also get the (x, y, theta) of the base
        self.goal = np.array(goalPose)  # TODO: put the goal (x, y, theta) in the goalPose

        if not self.fetch.isPoseValid(self.start):
            raise ValueError("Invalid start pose (how did this happen)")

        if not self.fetch.isPoseValid(self.goal):
            raise ValueError("Invalid goal pose")

        self.startTree = [BiRRTNode(self.start, startTree=True)]
        self.goalTree = [BiRRTNode(self.goal, startTree=False)]
        self.maxNodeDistance = maxNodeDistance
        self.connectionRate = connectionRate
        self.solution = None
        self.steps = 0
        self.maxProjections = maxProjections

    def generateNode(self):
        randomPose = self.fetch.getRandomPoses()
        while not self.fetch.isPoseValid(randomPose):
            randomPose = self.fetch.getRandomPoses()

        return BiRRTNode(randomPose, startTree=None)

    def closestNodeInTree(self, targetNode, tree):
        bestDistance = None
        closestNode = None

        for node in tree:
            distance = np.linalg.norm(targetNode.pose - node.pose)
            if bestDistance is None or distance < bestDistance:
                bestDistance = distance
                closestNode = node

        return closestNode, bestDistance

    # return the projected node, or None if the projected node is not valid
    def projectNode(self, nodeToAdd, existingNode):
        poseDelta = nodeToAdd.pose - existingNode.pose
        poseDeltaMagnitude = np.linalg.norm(poseDelta)

        if poseDeltaMagnitude < self.maxNodeDistance:
            if self.fetch.isPoseValid(nodeToAdd.pose):
                return nodeToAdd
        else:
            scaledPoseDelta = poseDelta / poseDeltaMagnitude * self.maxNodeDistance
            newPose = existingNode.pose + scaledPoseDelta

            if self.fetch.isPoseValid(newPose):
                return BiRRTNode(newPose)

        return None

    def getTreePath(self, node):
        isInStartTree = node.startTree
        path = []
        while node is not None:
            path.append(node.pose)
            node = node.parent

        # if this node is from the start tree, reverse our list
        if isInStartTree:
            return path[::-1]
        return path

    # return a list of poses if we can connect, otherwise return false
    def connectTree(self, startNode, endNode, intermediateNode=None):
        if intermediateNode is None:
            # start and end nodes too far away
            if np.linalg.norm(startNode.pose - endNode.pose) > self.connectionRate:
                return False
            return self.getTreePath(startNode) + self.getTreePath(endNode)
        else:
            if (np.linalg.norm(startNode.pose - intermediateNode.pose) > self.connectionRate
                or np.linalg.norm(endNode.pose - intermediateNode.pose) > self.connectionRate):

                return False
            return self.getTreePath(startNode) + [intermediateNode.pose] + self.getTreePath(endNode)

    def randomConnect(self):
        bestDistance = None
        closestStartNode = None
        closestGoalNode = None
        print("Trying a random connection")

        for startNode in self.startTree:
            for goalNode in self.goalTree:
                distance = np.linalg.norm(startNode.pose - goalNode.pose)

                if bestDistance is None or distance < bestDistance:
                    closestStartNode = startNode
                    closestGoalNode = goalNode
                    bestDistance = distance

        if bestDistance is not None:
            print("The distance is", bestDistance)
            if bestDistance < self.maxNodeDistance:
                path = self.connectTree(closestStartNode, closestGoalNode)

                if path is not False:
                    print(len(self.startTree))
                    print(len(self.goalTree))
                    # for node in self.startTree:
                    #     print(node.parent)
                    self.solution = path
            else:
                # try to just make a straight line connecting the start to the goal
                deltaPose = closestGoalNode.pose - closestStartNode.pose
                deltaPose = deltaPose / np.linalg.norm(deltaPose) * self.maxNodeDistance

                startDone = False
                goalDone = False
                count = 0
                ranOnce = False
                numProjections = 0

                # while np.linalg.norm(closestStartNode.pose - closestGoalNode.pose) >= self.maxNodeDistance and (startDone is False or goalDone is False):
                # originally had this adding til it couldn't, but that would create suboptimal paths
                # although maybe consider finding a nicer way to add this back
                while (numProjections < self.maxProjections and (startDone is False or goalDone is False)
                    and np.linalg.norm(closestStartNode.pose - closestGoalNode.pose) >= self.maxNodeDistance):

                    numProjections += 1

                    if not startDone:
                        closestStartNode = closestStartNode.deltaClone(deltaPose)
                        if self.fetch.isPoseValid(closestStartNode.pose):
                            self.startTree.append(closestStartNode)
                            count += 1
                        else:
                            startDone = True

                    if not goalDone:
                        closestGoalNode = closestGoalNode.deltaClone(-1 * deltaPose)
                        if self.fetch.isPoseValid(closestGoalNode.pose):
                            self.goalTree.append(closestGoalNode)
                            count += 1
                        else:
                            goalDone = True

                print("Projected", count, "nodes")

                return count > 0
                    


        return False

        print("Tried to connect trees; best distance was", bestDistance)

    def step(self):
        if self.solution:
            print("Done – please stop calling step")
            return

        self.steps += 1

        # Drawing straight to the goal initially makes some gross solutions
        # if self.steps == 1 or np.random.random() < self.connectionRate:
        if np.random.random() < self.connectionRate:
            return self.randomConnect()

        nodeToAdd = self.generateNode()
        self.addNodeToTree(nodeToAdd)

    # return true if we've added a node to the trees; update self.solution if we found a path
    def addNodeToTree(self, nodeToAdd):

        closestStartNode, bestStartDistance = self.closestNodeInTree(nodeToAdd, self.startTree)
        closestGoalNode, bestGoalDistance = self.closestNodeInTree(nodeToAdd, self.goalTree)

        # if this node is close to both of these nodes and can serve as a means to connect the tree
        if bestStartDistance < self.maxNodeDistance and bestGoalDistance < self.maxNodeDistance:
            print("Trying to connect through an intermediate node")
            tree = self.connectTree(closestStartNode, closestGoalNode, nodeToAdd)
            # connected successfully
            if isinstance(tree, list):
                print("Finished!")
                self.solution = tree
                return False

        successes = 0
        # add projection to both 
        for closestNode in [closestStartNode, closestGoalNode]:
            projectedNodeToAdd = self.projectNode(nodeToAdd, closestNode)

            # projection was not valid; don't add
            if projectedNodeToAdd is None:
                continue

            projectedNodeToAdd.parent = closestNode
            projectedNodeToAdd.startTree = closestNode.startTree

            if projectedNodeToAdd.startTree is True:
                self.startTree.append(projectedNodeToAdd)
                successes += 1
            elif projectedNodeToAdd.startTree is False:
                self.goalTree.append(projectedNodeToAdd)
                successes += 1
            else:
                raise ValueError("Invalid startTrees set")

        return successes > 0