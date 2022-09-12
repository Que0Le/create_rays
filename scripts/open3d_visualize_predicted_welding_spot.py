#!/usr/bin/env python3
import random
from xml.dom.minidom import Element
# import pybullet as p
import xml.etree.ElementTree as ET
import numpy as np
import os, shutil
import csv, math
import pandas as pd
from pathlib import Path
from typing import List
import open3d as o3d
from copy import copy


NAHT_DIR = "naht2"
COMP_DIR = "components"
TEST_OBJ = "201910204483"
TEST_NAHT = "PgmDef_34_0" # has 2 Punkt, so we take first one.

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
    elif method == "Fl_norms_distance":
        # Smaller is bettter
        scorings = np.absolute(scorings)
        ind = np.argpartition(a=scorings, kth=nbr_values)[:nbr_values]
        return ind[np.argsort(a=-1*scorings[ind])]        
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


""" Visualize to_predict_punkt with a white, bigger sphere """
to_predict_point_csv = pd.read_csv(f"to_predict_Punkt.csv", sep=';')
mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius = 300.0)
head_1 = to_predict_point_csv.head(1)
Punkt_Pos: str = head_1["Punkt_Pos"].values[0]
Frame_XVek: str = head_1["Frame_Xvek"].values[0]
Frame_YVek: str = head_1["Frame_Yvek"].values[0]
Frame_ZVek: str = head_1["Frame_Zvek"].values[0]
tf = np.zeros((4,4))  # 4x4 homogenous transform
tf[3,3] = 1.0
tf[0:3,3] = [float(coord) for coord in Punkt_Pos.split(" ")]
tf[0:3, 0] = [float(coord) for coord in Frame_XVek.split(" ")]
tf[0:3, 1] = [float(coord) for coord in Frame_YVek.split(" ")]
tf[0:3, 2] = [float(coord) for coord in Frame_ZVek.split(" ")]
mesh_sphere.compute_vertex_normals()
mesh_sphere.paint_uniform_color(np.array([255,255,0])/255)
mesh_sphere.transform(tf)
elements.append(mesh_sphere)

result_predicted_file = pd.read_csv(f"result_predict.csv", sep=';')
welding_point_filenames = result_predicted_file["Punkt"].to_numpy()

torches_colors = {
    "torch_1_method_L2_Norm": np.array([255, 0, 255])/255,
    "torch_2_method_L2_Norm": np.array([255, 0, 255])/255,#np.array([255, 0, 255])/255,
    #
    "torch_1_method_difflib_SequenceMatcher": np.array([255, 0, 0])/255,
    "torch_2_method_difflib_SequenceMatcher": np.array([255, 0, 0])/255,#np.array([0, 255, 255])/255,    
    #
    "torch_1_method_Fl_norms_distance": np.array([0, 0, 255])/255,
    "torch_2_method_Fl_norms_distance": np.array([0, 0, 255])/255,
    #
    "torch_1_to_predict_spot": np.array([255, 255, 255])/255,
    "torch_2_to_predict_spot": np.array([255, 255, 255])/255,
}

i_th = 1
poses_mapped_by_methods = {}

for method in result_predicted_file.columns.values[1:]:
    # if method != "Fl_norms_distance":
    #     i_th = i_th + 1
    #     continue
    scorings = result_predicted_file[method].to_numpy()    # np array holds scorings for current method
    indices = get_top_scoring_indices_for_method(scorings=scorings, method=method, nbr_values=70)
    poses_mapped_by_methods[method] = indices

    # print(method)
    # print(scorings[indices])
    for index in indices:
        to_displayed_point_csv = pd.read_csv(f"naht2/{welding_point_filenames[index]}", sep=';')
        Punkt_Pos: str = to_displayed_point_csv.head(1)["Punkt_Pos"].values[0]
        WkzName: str = to_displayed_point_csv.head(1)["WkzName"].values[0]
        Frame_XVek: str = to_displayed_point_csv.head(1)["Frame_Xvek"].values[0]
        Frame_YVek: str = to_displayed_point_csv.head(1)["Frame_Yvek"].values[0]
        Frame_ZVek: str = to_displayed_point_csv.head(1)["Frame_Zvek"].values[0]

        torch_color = torches_colors.get(
            f"torch_{1 if WkzName=='MRW510_10GH' else 2}_method_{method}"
        )
        # print(torch_color)
        # print(f"torch_{1 if WkzName=='MRW510_10GH' else 2}_method_{method}")
        elements.append(
            create_pose_torch(
                torch_color=torch_color, 
                mesh_torch=copy(torch1) if WkzName=="MRW510_10GH" else copy(torch2),
                position=[float(coord) for coord in Punkt_Pos.split(" ")], 
                XVek=[float(coord) for coord in Frame_XVek.split(" ")],
                YVek=[float(coord) for coord in Frame_YVek.split(" ")],
                ZVek=[float(coord) for coord in Frame_ZVek.split(" ")],
            )
        )
    i_th = i_th + 1
    # break

# print(poses_mapped_by_methods)
SHOW_OBJECT_WITH_ALL_POSES = True
if SHOW_OBJECT_WITH_ALL_POSES:
    mapped_poses = []   # hold all poses that were mapped by one of the methods
    for key, value in poses_mapped_by_methods.items():
        mapped_poses = mapped_poses + list(value)
    mapped_poses = set(mapped_poses)
    for filename in welding_point_filenames:
        if filename not in mapped_poses:
            point_file = pd.read_csv(f"naht2/{filename}", sep=';')
            head_1 = point_file.head(1)
            Punkt_Pos: str = head_1["Punkt_Pos"].values[0]
            WkzName: str = head_1["WkzName"].values[0]
            Frame_XVek: str = head_1["Frame_Xvek"].values[0]
            Frame_YVek: str = head_1["Frame_Yvek"].values[0]
            Frame_ZVek: str = head_1["Frame_Zvek"].values[0]
            torch_color = np.array([0, 255, 0])/255
            elements.append(
                create_pose_torch(
                    torch_color=torch_color, 
                    mesh_torch=copy(torch1) if WkzName=="MRW510_10GH" else copy(torch2),
                    position=[float(coord) for coord in Punkt_Pos.split(" ")], 
                    XVek=[float(coord) for coord in Frame_XVek.split(" ")],
                    YVek=[float(coord) for coord in Frame_YVek.split(" ")],
                    ZVek=[float(coord) for coord in Frame_ZVek.split(" ")],
                )
            )
o3d.visualization.draw_geometries(elements)
