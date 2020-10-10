from cornerRounding import CornerRoundedPath
import matplotlib.pyplot as plt



path = CornerRoundedPath("sample.gcode", 1, 0.2,0.1,1)
# plt.plot(path.xPos, path.yPos)



path.setTarget()
for x in range(0,1000):
    path.moveTowardsTarget()

plt.plot(path.pathTakenX,path.pathTakenY)
plt.show()