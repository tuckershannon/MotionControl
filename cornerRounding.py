from pygcode import Line, Machine
import numpy as np
import math
import matplotlib.pyplot as plt


class CornerRoundedPath:
    def __init__(self, filepath, resolution, cornerRadius, accelLimit, velocityLimit):
        self.cornerRadius = cornerRadius
        self.filepath = filepath
        self.traj = []
        self.xPos = []
        self.yPos = []
        self.r1PosX = []
        self.r1PosY = []
        self.r2PosX = []
        self.r2PosY = []
        self.resolution = resolution
        self.readGcode(filepath)
        self.discretizePath()
        self.roundPath2()
        self.accelLimit = accelLimit
        self.velocityLimit = velocityLimit
        self.closeEnough = False
        self.finished = False
        self.position = [0, 0]
        self.velocity = [0, 0]
        self.pathTakenX = []
        self.pathTakenY = []
        self.distanceLog = []
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
                nPoints = round(distance / self.resolution) + 1
                if nPoints > 0:
                    x_points = np.linspace(0, delta_X, nPoints)
                    y_points = np.linspace(0, delta_Y, nPoints)
                    x_points = x_points + currentPoint[0]
                    y_points = y_points + currentPoint[1]
                    for i in range(0, len(x_points)):
                        if discretizedPath[-1] != [x_points[i], y_points[i]]:
                            discretizedPath.append([x_points[i], y_points[i]])
                    # discretizedPath = discretizedPath + list(zip(x_points, y_points))
            else:
                discretizedPath = [self.traj[0]]

        self.traj = discretizedPath

    def startCorner(self, p1, p2):
        x = p2[0] - p1[0]
        y = p2[1] - p1[0]
        d = math.sqrt(x ** 2 + y ** 2)
        if d <= self.cornerRadius:
            return True
        return False

    def radiusCenters(self, p1, p2, currentPos):
        x = p2[0] - p1[0]
        y = p2[1] - p1[0]
        distance = math.sqrt(x ** 2 + y ** 2)

        if distance > 0:
            r1 = [currentPos[0] + (-y / distance) * self.cornerRadius,
                  currentPos[1] + (x / distance) * self.cornerRadius]
            r2 = [currentPos[0] + (y / distance) * self.cornerRadius,
                  currentPos[1] + (-x / distance) * self.cornerRadius]
        else:
            return currentPos, currentPos
        return r1, r2

    def roundPath(self):
        discretizedPath = []
        for i in range(0, len(self.traj)):
            point = self.traj[i]
            if len(discretizedPath) > 0:
                currentPoint = discretizedPath[-1]

                delta_X = point[0] - currentPoint[0]
                delta_Y = point[1] - currentPoint[1]
                distance = math.sqrt(delta_X ** 2 + delta_Y ** 2)
                nPoints = round(distance / self.resolution) + 1
                if nPoints > 0:
                    x_points = np.linspace(0, delta_X, nPoints)
                    y_points = np.linspace(0, delta_Y, nPoints)
                    x_points = x_points + currentPoint[0]
                    y_points = y_points + currentPoint[1]
                    for x in range(0, nPoints):
                        r1, r2 = self.radiusCenters(currentPoint, point, [x_points[x], y_points[x]])
                        # if self.startCorner(r1,[x_points[x],y_points[x]]):

                        self.xPos.append(x_points[x])
                        self.yPos.append(y_points[x])
                        self.r1PosX.append(r1[0])
                        self.r1PosY.append(r1[1])
                        self.r2PosX.append(r2[0])
                        self.r2PosY.append(r2[1])
                        discretizedPath.append([x_points[x], y_points[x]])
                    # discretizedPath = discretizedPath + list(zip(x_points, y_points))
            else:
                discretizedPath = [self.traj[0]]

        self.traj = discretizedPath

    def roundPath2(self):
        import numpy as np

        refinements = 2
        coords = np.array(self.traj)

        for _ in range(refinements):
            L = coords.repeat(2, axis=0)
            R = np.empty_like(L)
            R[0] = L[0]
            R[2::2] = L[1:-1:2]
            R[1:-1:2] = L[2::2]
            R[-1] = L[-1]
            coords = L * 0.75 + R * 0.25

        self.traj = coords
        for pos in coords:
            self.xPos.append(pos[0])
            self.yPos.append(pos[1])

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


