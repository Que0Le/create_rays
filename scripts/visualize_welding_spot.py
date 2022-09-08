#!/usr/bin/env python3
import random
from xml.dom.minidom import Element
import pybullet as p
import xml.etree.ElementTree as ET
import numpy as np
import os, shutil
import csv, math
import pandas as pd
from pathlib import Path
from typing import List

NAHT_DIR = "naht2"
COMP_DIR = "components"
TEST_OBJ = "201910204483"
TEST_NAHT = "PgmDef_34_0" # has 2 Punkt, so we take first one.


def setup_pb():
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera( 
        cameraDistance=11.40, 
        cameraPitch=-40.40, 
        cameraYaw=-4.8, 
        cameraTargetPosition=[6.50,7.59,-3.33]
    )

def show_welding_target(position: np.array, size: float = 0.1, rgbaColor: List = [1,0,0,1]):
    # print(position)
    visual_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=size, rgbaColor=rgbaColor)
    point_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_id,
        basePosition=position/1000,
    )

def visualize_welding_spot(
    Punkt_Pos: str, rgbaColor: List = [1,0,0,1], size: float = 0.1,
    ZRotLock: str = "", WkzWkl: str = "", WkzName: str = "",
    Punkt_Fl_Norm1: str = "", Punkt_Fl_Norm2: str = "", 
    Frame_Pos: str = "", Frame_Xvek: str = "", Frame_Yvek: str = "", Frame_Zvek: str = ""
):
    show_welding_target(
        position=np.array([float(coord) for coord in Punkt_Pos.split(" ")]),
        rgbaColor=rgbaColor, size=size
    )

def get_top_scoring_indices_for_method(scorings: np.array, method: str, nbr_values: int):
    """ return 'nbr_values' indices of most similar scorings, ascending (last index is most similar)"""
    if method == "L2_Norm":
        # Smaller is bettter
        ind = np.argpartition(a=scorings, kth=nbr_values)[:nbr_values]
        return ind[np.argsort(a=-1*scorings[ind])]
    elif method == "difflib_SequenceMatcher":
        # greater is more similar
        ind = np.argpartition(a=scorings, kth=-1*nbr_values)[-1*nbr_values:]
        return ind[np.argsort(a=scorings[ind])]
    else:
        return np.array([])

    """ Test
    a = np.array([9, 4, -4, 3, 3, -9, 0, 4, 6, 0])
    # index       0  1   2  3  4   5  6  7  8  8

    ind = get_top_scoring_indices_for_method(scorings=a, method="L2_Norm", nbr_values=5)
    print(ind)
    print(a[ind])
    # [4 9 6 2 5]
    # [ 3  0  0 -4 -9]
    ind = get_top_scoring_indices_for_method(scorings=a, method="cosine_similarity", nbr_values=5)
    print(ind)
    print(a[ind])
    # [3 7 1 8 0]
    # [3 4 4 6 9]
    """

setup_pb()
obj_dir = TEST_OBJ
path_xml = os.path.join(COMP_DIR, obj_dir, obj_dir+'.xml')
object_id = p.loadURDF(
    './components/'+obj_dir+'/model.urdf',
    basePosition=[0, 0, 0],  # r:x
    baseOrientation=[0, 0, 0, 1],
    useFixedBase=True
)

import pandas as pd
thres_L2_Norm = 1
thres_cosine_similarity = 1
thres_difflib_SequenceMatcher = 1

result_predicted_file = pd.read_csv(f"result_predict.csv", sep=';')
welding_point_filenames = result_predicted_file["Punkt"].to_numpy()

colors = [[1,0,0,1], [0,0,1,1]]
i_th = 0
for method in result_predicted_file.columns.values[1:]:
    scorings = result_predicted_file[method].to_numpy()    # np array holds scorings for current method
    indices = get_top_scoring_indices_for_method(scorings=scorings, method=method, nbr_values=20)
    print(method)
    print(scorings[indices])
    for index in indices:
        to_displayed_point_csv = pd.read_csv(f"naht2/{welding_point_filenames[index]}", sep=';')
        Punkt_Pos = to_displayed_point_csv.head(1)["Punkt_Pos"].values[0]
        # print(Punkt_Pos)
        visualize_welding_spot(Punkt_Pos=Punkt_Pos, rgbaColor=colors[i_th], size=0.1)
    i_th = i_th + 1

# Dont let sim destroyed
while True:
    p.stepSimulation()