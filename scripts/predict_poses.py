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
import difflib
import open3d as o3d
from typing import List
from copy import copy

def create_pose_torch(
    torch_color: np.array, mesh_torch, position: List[float], 
    XVek: List[float], YVek: List[float], ZVek: List[float]
):
    tf = np.zeros((4,4))  # 4x4 homogenous transform
    tf[3,3] = 1.0
    tf[0:3,3] = position
    tf[0:3, 0] = XVek
    tf[0:3, 1] = YVek
    tf[0:3, 2] = ZVek

    mesh_torch.compute_vertex_normals()
    mesh_torch.paint_uniform_color(torch_color)
    mesh_torch.transform(tf)

    return mesh_torch


mesh_model = o3d.io.read_triangle_mesh("components/201910204483_R1.obj")
mesh_model.compute_vertex_normals()
elements = []
elements.append(mesh_model)
torch1 = o3d.io.read_triangle_mesh("torches/MRW510_10GH.obj")
torch2 = o3d.io.read_triangle_mesh("torches/TAND_GERAD_DD.obj")


# tree = ET.parse("components/201910204483/201910204483.xml")
tree = ET.parse("ml_fully_connected_base_model_1.xml")
root = tree.getroot()

for SNaht in root.findall('SNaht'):
    Name = SNaht.get('Name')
    ZRotLock = SNaht.get('ZRotLock')
    WkzWkl = SNaht.get('WkzWkl')
    WkzName = SNaht.get('WkzName')

    Punkt_s = SNaht.findall('Kontur')[0].findall('Punkt')
    Frame_s = SNaht.findall('Frames')[0].findall('Frame')

    for Frame in Frame_s:
        Pos = Frame.findall('Pos')[0]
        XVek = Frame.findall('XVek')[0]
        YVek = Frame.findall('YVek')[0]
        ZVek = Frame.findall('ZVek')[0]
        elements.append(
            create_pose_torch(
                torch_color=np.array([255, 0, 0])/255, 
                mesh_torch=copy(torch1) if WkzName=="MRW510_10GH" else copy(torch2),
                position=[float(Pos.get('X')), float(Pos.get('Y')), float(Pos.get('Z'))],
                XVek=[float(XVek.get('X')), float(XVek.get('Y')), float(XVek.get('Z'))],
                YVek=[float(YVek.get('X')), float(YVek.get('Y')), float(YVek.get('Z'))],
                ZVek=[float(ZVek.get('X')), float(ZVek.get('Y')), float(ZVek.get('Z'))],
            )
        )


o3d.visualization.draw_geometries(elements)