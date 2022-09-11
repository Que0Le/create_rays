#!/usr/bin/env python3
import random
from xml.dom.minidom import Element
import pybullet as p
import xml.etree.ElementTree as ET
import numpy as np
import os
import shutil
import csv
import math
import pandas as pd
from pathlib import Path


NAHT_DIR = "naht2"
COMP_DIR = "components"
TEST_OBJ = "201910204483"
TEST_NAHT = "PgmDef_34_0"  # has 2 Punkt, so we take first one.


def setup_pb():
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=11.40,
        cameraPitch=-40.40,
        cameraYaw=-4.8,
        cameraTargetPosition=[6.50, 7.59, -3.33]
    )


def show_welding_target(position):
    visual_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1]
    )
    point_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_id,
        basePosition=position,
    )


def visualize_torch(torchname: str, position: np.array):
    if True:  # torchname == TORCH_0:
        p.loadURDF(
            # f"./{TORCH_DIR}/{TORCH_0}.urdf",
            "./torches/MRW510_10GH.urdf",
            basePosition=np.array(
                [float(coord)/1000 for coord in position.split(" ")])
        )


setup_pb()
obj_dir = TEST_OBJ
path_xml = os.path.join(COMP_DIR, obj_dir, obj_dir+'.xml')
object_id = p.loadURDF(
    './components/'+obj_dir+'/model.urdf',
    basePosition=[0, 0, 0],  # r:x
    baseOrientation=[0, 0, 0, 1],
    useFixedBase=True
)
position = "10815.50000000005 6229.00000000147 -4.894131961918902e-07"
norm1 = "1.471045507628332e-15 -1.000000000000003 -3.33066907387547e-16"
norm2 = "-2.334869827394095e-16 -7.771561172376102e-16 1.000000000000041"

# terrain_mass = 0
terrain_visual_shape_id = -1
terrain_position = [0, 0, 0]
terrain_orientation = [0, 0, 0, 1]
terrain_collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="./torches/MRW510_10GH.obj",
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH | p.GEOM_CONCAVE_INTERNAL_EDGE,
    # meshScale=[0.5, 0.5, 0.5]
    meshScale=[0.01, 0.01, 0.01]
)
p.createMultiBody(
    # terrain_mass, 
    baseCollisionShapeIndex=terrain_collision_shape_id, 
    baseVisualShapeIndex=terrain_visual_shape_id,
    basePosition=np.array(
                [float(coord) for coord in norm1.split(" ")]
            ), 
    baseOrientation=np.array(
                [float(coord) for coord in norm2.split(" ")]
            ),
)

# show_welding_target(pos)
# visualize_torch(
#     "MRW510_10GH",
#     pos
# )

while True:
    p.stepSimulation()
