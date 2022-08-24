import sys
import os, math
import numpy as np
import pybullet as p
import pybullet_data as pd
import random
CURRENT_PATH = os.path.abspath(__file__)
BASE = os.path.dirname(os.path.dirname(CURRENT_PATH)) 
ROOT = os.path.dirname(BASE) 
sys.path.insert(0,os.path.dirname(CURRENT_PATH))
from xml_parser import parse_frame_dump, list2array

# path to component
# if you want to add new components, the directory structure should keep the same
# the unit in the original .obj is mm, the unit in pybullet is meter, so do
# a simple conversion first
PATH_COMP = os.path.join(BASE, 'components')
COMP_LIST = os.listdir(PATH_COMP)

def random_choice_welding_spot(torch=0):
    '''
    from components list select a random component and from this component
    select one welding spot randomly
    args:
    torch: 0 is MRW510_10GH, 1 is TAND_GERAD_DD
    return:
    component: str
    welding spot with torch, position, normals, rotation matrix: array
    '''
    comp = random.choice(COMP_LIST)
    path_xml = os.path.join(PATH_COMP, comp, comp+'.xml')
    all_spots = list2array(parse_frame_dump(path_xml))[:,3:].astype(float)
    spots_with_torch0 = all_spots[(all_spots[:,0]==torch),:]
    current_spot = spots_with_torch0[np.random.choice(spots_with_torch0.shape[0], size=1, replace=False), :][0,1:19]
    return comp, current_spot

def show_welding_target(position):
    visual_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.4, rgbaColor=[1,0,0,1])
    point_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_id,
                basePosition=position,
                )
# connect to GUI
physicsClient = p.connect(p.GUI)

# load a component into pybullet
current_comp = COMP_LIST[0]
object_id=p.loadURDF(
    '../components/'+current_comp+'/model.urdf', 
    basePosition=[0,0,0],  #r:x
    baseOrientation=[0,0,0,1], 
    useFixedBase=True
)
# load welding info from xml file
current_xml = os.path.join(PATH_COMP, current_comp, current_comp+'.xml')
# return welding info in dict format
all_info = parse_frame_dump(current_xml)
# convert the dict into ndarray for easy processing
all_spots = list2array(all_info)
# select one welding spot from it
# mind the unit
current_spot_position = (all_spots[1,4:7].astype(float))/1000
# show this welding spot
# show_welding_target(current_spot_position)

# # default cam
# p.resetDebugVisualizerCamera( 
#     cameraDistance=5, 
#     cameraPitch=-35, 
#     cameraYaw=50, 
#     cameraTargetPosition=[0,0,0]
# )
p.resetDebugVisualizerCamera( 
    cameraDistance=23, 
    cameraPitch=-35, 
    cameraYaw=50, 
    cameraTargetPosition=[0,0,0]
)

# p.setAdditionalSearchPath(pd.getDataPath())

# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

#p.loadURDF("samurai.urdf")
# p.loadURDF("r2d2.urdf", current_spot_position)

rayFrom = []
rayTo = []
rayIds = []
useGui = True

numRays = 128

rayLen = 20

rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]

replaceLines = True

for i in range(numRays):
    # rayFrom.append([0, 0, 1])
    # rayFrom.append([current_spot_position[0], current_spot_position[1], 0])
    rayFrom.append([5,5, 0])
    rayTo.append([
        rayLen * math.sin(2. * math.pi * float(i) / numRays),
        rayLen * math.cos(2. * math.pi * float(i) / numRays), 
        0
    ])
    if (replaceLines):
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))
    else:
        rayIds.append(-1)

if (not useGui):
    timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "rayCastBench.json")


results = p.rayTestBatch(rayFrom, rayTo)

for i in range(0, len(results)):
    hitObjectUid = results[i][0]
    
    if (hitObjectUid < 0):
        hitPosition = [0, 0, 0]
        p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
    else:
        hitPosition = results[i][3]
        p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])


# numSteps = 10
# if (useGui):
#     numSteps = 327680

# for i in range(numSteps):
#     p.stepSimulation()
#     for j in range(8):
#         results = p.rayTestBatch(rayFrom, rayTo, j + 1)

#   #for i in range (10):
#   #	p.removeAllUserDebugItems()

#     if (useGui):
#         if (not replaceLines):
#             p.removeAllUserDebugItems()

#         for i in range(numRays):
#             hitObjectUid = results[i][0]

#             if (hitObjectUid < 0):
#                 hitPosition = [0, 0, 0]
#                 p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
#             else:
#                 hitPosition = results[i][3]
#                 p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])

# start simulator
while True:
    p.stepSimulation()