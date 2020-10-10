from pygcode import Line, Machine
import numpy as np
import math


class Trajectory:
    def __init__(self, accelLimit, velocityLimit):
        self.accelLimit = accelLimit
        self.velocityLimit = velocityLimit
        self.closeEnough = False
        self.finished = False
        self.position = [0, 0]
        self.velocity = [0, 0]
        self.traj = []
        self.pathTakenX = []
        self.pathTakenY = []
        self.distanceLog = []
        self.resolution = 0.1  # resolution for path discretization and distance to update target
        self.stepNumber = -1
        self.timeResolution = 0.01

    def readGcode(self, filepath):
        m = Machine()
        with open(filepath, 'r') as fh:
            for line_text in fh.readlines():
                self.traj.append((m.pos.values['X'], m.pos.values['Y']))
                line = Line(line_text)
                m.process_block(line.block)

    def discretizePath(self):
        discretizedPath = []
        for point in self.traj:
            if len(discretizedPath) > 0:
                currentPoint = discretizedPath[-1]
                delta_X = point[0] - currentPoint[0]
                delta_Y = point[1] - currentPoint[1]
                distance = math.sqrt(delta_X ** 2 + delta_Y ** 2)
                nPoints = round(distance / self.resolution)
                if nPoints > 0:
                    x_points = np.linspace(0, delta_X, nPoints)
                    y_points = np.linspace(0, delta_Y, nPoints)
                    x_points = x_points + currentPoint[0]
                    y_points = y_points + currentPoint[1]
                    discretizedPath = discretizedPath + list(zip(x_points, y_points))
            else:
                discretizedPath = [self.traj[0]]

            self.traj = discretizedPath
        self.setTarget()

    def setTarget(self):
        self.stepNumber += 1
        self.target = list(self.traj[self.stepNumber])

        return None

    def calculateTrajVector(self):
        deltaX = self.target[0] - self.position[0]
        deltaY = self.target[1] - self.position[1]
        return deltaX, deltaY

    def moveTowardsTarget(self):
        if self.isCloseToTarget():
            self.setTarget()
        else:
            self.__setVelocity()
            self.__setPosition()
        return None

    def isCloseToTarget(self):
        delta_X = self.target[0] - self.position[0]
        delta_Y = self.target[1] - self.position[1]
        distance = math.sqrt(delta_X ** 2 + delta_Y ** 2)
        if distance < self.resolution:
            return True
        else:
            return False
        return None

    def __setAccel(self):
        self.accelLimit = None

    def __setVelocity(self):
        deltaX, deltaY = self.calculateTrajVector()

        distance = math.sqrt(deltaX ** 2 + deltaY ** 2)

        if distance == 0:
            targetVelocityX = 0.0
            targetVelocityY = 0.0
        else:
            targetVelocityX = deltaX / distance * self.velocityLimit
            targetVelocityY = deltaY / distance * self.velocityLimit

        self.distanceLog.append(deltaX)
        # if deltaY == 0:
        #     targetVelocityY = 0
        #
        #     if deltaX == 0:
        #         targetVelocityX = 0
        #     else:
        #         targetVelocityX = self.velocityLimit
        # elif deltaX > deltaY:
        #     targetVelocityX = self.velocityLimit
        #     targetVelocityY = (deltaY/deltaX)*self.velocityLimit
        # else:
        #     targetVelocityX = (deltaX/deltaY)*self.velocityLimit
        #     targetVelocityY = self.velocityLimit
        #
        #
        if self.velocity[0] < targetVelocityX:
            if (targetVelocityX - self.velocity[0]) > self.accelLimit:
                self.velocity[0] += (self.accelLimit*self.timeResolution)
            else:
                self.velocity[0] = targetVelocityX
        if self.velocity[0] > targetVelocityX:
            if (self.velocity[0] - targetVelocityX) > self.accelLimit:
                self.velocity[0] -= (self.accelLimit*self.timeResolution)
            else:
                self.velocity[0] = targetVelocityX

        if self.velocity[1] < targetVelocityY:
            if (targetVelocityY - self.velocity[1]) > self.accelLimit:
                self.velocity[1] += (self.accelLimit*self.timeResolution)
            else:
                self.velocity[1] = targetVelocityY
        if self.velocity[1] > targetVelocityY:
            if (self.velocity[1] - targetVelocityY) > self.accelLimit:
                self.velocity[1] -= (self.accelLimit*self.timeResolution)
            else:
                self.velocity[1] = targetVelocityY

    def __setPosition(self):
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]
        self.pathTakenX.append(self.position[0])
        self.pathTakenY.append(self.position[1])
