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
# from open3d_visualize_predicted_welding_spot import create_pose_torch
from copy import copy

NAHT_DIR = "naht2"
COMP_DIR = "components"
TEST_OBJ = "201910204483"
TEST_NAHT = "PgmDef_34_0" # has 2 Punkt, so we take first one.


# SHOW_OBJECT_WITH_ALL_POSES = False
# elements = []
# torch1 = o3d.io.read_triangle_mesh("torches/MRW510_10GH.obj")
# torch2 = o3d.io.read_triangle_mesh("torches/TAND_GERAD_DD.obj")
# if SHOW_OBJECT_WITH_ALL_POSES:
#     mesh_model = o3d.io.read_triangle_mesh("components/201910204483_R1.obj")
#     mesh_model.compute_vertex_normals()
#     elements.append(mesh_model)


def setup_pb():
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera( 
        cameraDistance=11.40, 
        cameraPitch=-40.40, 
        cameraYaw=-4.8, 
        cameraTargetPosition=[6.50,7.59,-3.33]
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

def generate_hit_for_punkt(
    db_file_path: str, Punkt_th: Element, Frame_th: Element, 
    ZRotLock: str, WkzWkl: str, WkzName: str,
    num_ray: int = 32, ray_len: float = 1, miss_faction: float = -1, draw_breams: bool = False
):
    punk_xyz = np.array([
        float(Punkt_th.get('X')), float(
            Punkt_th.get('Y')), float(Punkt_th.get('Z'))
    ])
    print(f"Working: punk_xyz={punk_xyz}")

    fl_norms = []
    for Fl_Norm in Punkt_th.findall('Fl_Norm'):
        fl_norms.append(([
            float(Fl_Norm.get('X')), float(
                Fl_Norm.get('Y')), float(Fl_Norm.get('Z'))
        ]))

    frame_veks = []
    Pos = Frame_th.findall('Pos')[0]
    XVek = Frame_th.findall('XVek')[0]
    YVek = Frame_th.findall('YVek')[0]
    ZVek = Frame_th.findall('ZVek')[0]
    frame_pos = [float(Pos.get('X')), float(
        Pos.get('Y')), float(Pos.get('Z'))]
    frame_veks.append(([
        float(XVek.get('X')), float(
            XVek.get('Y')), float(XVek.get('Z'))
    ]))
    frame_veks.append(([
        float(YVek.get('X')), float(
            YVek.get('Y')), float(YVek.get('Z'))
    ]))
    frame_veks.append(([
        float(ZVek.get('X')), float(
            ZVek.get('Y')), float(ZVek.get('Z'))
    ]))

    # create pos for visualization the whole object with poses
    # if SHOW_OBJECT_WITH_ALL_POSES:
    #     elements.append(
    #         create_pose_torch(
    #             torch_color=np.array([255, 0, 0])/255, 
    #             mesh_torch=copy(torch1) if WkzName=="MRW510_10GH" else copy(torch2),
    #             position=punk_xyz, 
    #             XVek=frame_veks[0],
    #             YVek=frame_veks[1],
    #             ZVek=frame_veks[2],
    #         )
    #     )

    hit_pos = []
    nor_pos = []
    hit_fraction = []


    """ Run sim """

    rayFrom = []
    rayTo = []

    rayHitColor = [x / 255 for x in [255, 0, 0]]
    rayMissColor = [x / 255 for x in [0, 255, 0]]
    
    # pybullet.MAX_RAY_INTERSECTION_BATCH_SIZE
    # 16384  2^14
    for xy in range(num_ray):
        for z in range(num_ray):
            start_pos = np.array(frame_pos)/1000
            phi = 2. * math.pi * float(xy) / num_ray
            theta = 2. * math.pi * float(z) / num_ray
            end_pos = [s+e for s, e in zip(
                [
                    ray_len * math.sin(phi) * math.cos(theta),
                    ray_len * math.sin(phi) * math.sin(theta),
                    ray_len * math.cos(phi)
                ],
                start_pos
                )
            ]
            rayFrom.append(start_pos)
            rayTo.append(end_pos)

    results = p.rayTestBatch(rayFrom, rayTo)

    for i in range(0, len(results)):
        hitObjectUid = results[i][0]
        if (hitObjectUid >= 0): # hit
            hit_fraction.append(results[i][2])
            hit_pos.append(results[i][3])
            nor_pos.append(results[i][4])
            if draw_breams:
                p.addUserDebugLine(rayFrom[i], results[i][3], rayHitColor)
        else:
            hit_fraction.append(miss_faction)
            hit_pos.append(np.array([0,0,0]))
            nor_pos.append(np.array([0,0,0]))
            if draw_breams:
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor)
    """  """
    with open(db_file_path, 'w') as naht_csvfile:
        naht_writer = csv.writer(naht_csvfile, delimiter=";")
        naht_writer.writerow([
            "ZRotLock", "WkzWkl", "WkzName",
            "Punkt_Pos", "Punkt_Fl_Norm1", "Punkt_Fl_Norm2",
            "Frame_Pos", "Frame_Xvek", "Frame_Yvek", "Frame_Zvek",
            "hit_fraction", "hit_pos", "hit_nor_pos"
        ])
        for hit_th in range(0, len(hit_pos)):
            naht_writer.writerow([
                ZRotLock, WkzWkl, WkzName,
                " ".join(str(x) for x in punk_xyz),
                " ".join(str(x) for x in fl_norms[0]),
                " ".join(str(x) for x in fl_norms[1]),
                ###
                " ".join(str(x) for x in frame_pos),
                " ".join(str(x) for x in frame_veks[0]),
                " ".join(str(x) for x in frame_veks[1]),
                " ".join(str(x) for x in frame_veks[2]),
                ###
                hit_fraction[hit_th],
                " ".join(str(x) for x in hit_pos[hit_th]),
                " ".join(str(x) for x in nor_pos[hit_th]),
            ])
    if draw_breams:
        p.removeAllUserDebugItems()


def generate_db_for_obj(
    obj_dir: str, target_dir: str, 
    num_ray: int = 32, ray_len: float = 1.0, miss_faction: float = -1.0, draw_breams: bool = False
):
    """ 
    For each SchweissPunkt, create a lidar map around and export all rays to csv file. 
    If ray didn't hit anything, its hit_fraction (0->ray_len) will be set to `miss_faction`
    """
    path_xml = os.path.join(COMP_DIR, obj_dir, obj_dir+'.xml')
    object_id = p.loadURDF(
        './components/'+obj_dir+'/model.urdf',
        basePosition=[0, 0, 0],  # r:x
        baseOrientation=[0, 0, 0, 1],
        useFixedBase=True
    )

    tree = ET.parse(path_xml)
    root = tree.getroot()

    for SNaht in root.findall('SNaht'):
        Name = SNaht.get('Name')
        ZRotLock = SNaht.get('ZRotLock')
        WkzWkl = SNaht.get('WkzWkl')
        WkzName = SNaht.get('WkzName')

        Punkt_s = SNaht.findall('Kontur')[0].findall('Punkt')
        Frame_s = SNaht.findall('Frames')[0].findall('Frame')

        for i in range(0, len(Punkt_s)):
            db_file_path = f"{target_dir}/{obj_dir}.{Name}.{WkzName}.{WkzWkl}.{i}.csv"
            generate_hit_for_punkt(
                db_file_path=db_file_path, Punkt_th=Punkt_s[i], Frame_th=Frame_s[i], 
                ZRotLock=ZRotLock, WkzWkl=WkzWkl, WkzName=WkzName,
                num_ray=num_ray, ray_len=ray_len, miss_faction=miss_faction, draw_breams=draw_breams
            )

if not os.path.isdir(f"./{NAHT_DIR}"):
    Path(NAHT_DIR).mkdir()


setup_pb()


""" Generate the DB """
generate_db_for_obj(
    obj_dir=TEST_OBJ, target_dir=NAHT_DIR, 
    num_ray=20, ray_len=1, miss_faction=1, draw_breams=False
)


""" Take a Punkt and use as test target """
to_predict_Punkt_csv_file = f"to_predict_Punkt.csv"
shutil.copy2(f"{NAHT_DIR}/201910204483.PgmDef_44_1.TAND_GERAD_DD.10.1.csv", to_predict_Punkt_csv_file)


""" Predict """
# Read target hit data
df = pd.read_csv(to_predict_Punkt_csv_file, sep=';')
hit_faction_target = df["hit_fraction"].to_numpy()
first_row = df.head(1)
Punkt_Fl_Norm1: str = first_row["Punkt_Fl_Norm1"].values[0]
Punkt_Fl_Norm2: str = first_row["Punkt_Fl_Norm2"].values[0]
to_predict_norms_l2_distance = np.linalg.norm(
    np.array([float(a) for a in Punkt_Fl_Norm1.split(" ")])-np.array([float(b) for b in Punkt_Fl_Norm2.split(" ")])
)
norm1_vector = [float(a) for a in Punkt_Fl_Norm1.split(" ")]
norm2_vector = [float(a) for a in Punkt_Fl_Norm2.split(" ")]
norm_Fl_to_predict = norm1_vector + norm2_vector
# List of only DB file
list_naht_csv_file = os.listdir(f"{NAHT_DIR}/")
# list_naht_csv_file.remove("to_predict_Punkt.csv")

pred_methods = ["L2_Norm", "difflib_SequenceMatcher", "Fl_norms_distance", "Fl_angle_degree"]
res_dist = []   # hold scores for all Punkt

for naht_file in list_naht_csv_file:
    df_candidate = pd.read_csv(f"{NAHT_DIR}/{naht_file}", sep=';')
    hit_fraction_candidate = df_candidate["hit_fraction"].to_numpy()
    pred_score_for_methods = {}
    # Teham L1-L2
    pred_score_for_methods["L1-L2"] = np.mean(hit_fraction_candidate - hit_faction_target)
    # L2 distance method
    distance = np.linalg.norm(-(hit_fraction_candidate - hit_faction_target)) # L-2 norm
    pred_score_for_methods["L2_Norm"] = distance
    # difflib
    sm=difflib.SequenceMatcher(None, hit_faction_target, hit_fraction_candidate)
    pred_score_for_methods["difflib_SequenceMatcher"] = sm.ratio()
    # Fl_norm_distance
    first_row = df_candidate.head(1)
    Punkt_Fl_Norm1: str = first_row["Punkt_Fl_Norm1"].values[0]
    Punkt_Fl_Norm2: str = first_row["Punkt_Fl_Norm2"].values[0]
    norms_l2_distance = np.linalg.norm(
        np.array([float(a) for a in Punkt_Fl_Norm1.split(" ")])-np.array([float(b) for b in Punkt_Fl_Norm2.split(" ")])
    )
    pred_score_for_methods["Fl_norms_distance"] = norms_l2_distance - to_predict_norms_l2_distance
    # Fl angle
    norm1_vector = [float(a) for a in Punkt_Fl_Norm1.split(" ")]
    norm2_vector = [float(a) for a in Punkt_Fl_Norm2.split(" ")]
    norm_Fl = norm1_vector + norm2_vector
    unit_vector_1 = norm_Fl / np.linalg.norm(norm_Fl)
    unit_vector_2 = norm_Fl_to_predict / np.linalg.norm(norm_Fl_to_predict)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle_against_to_predict = np.arccos(dot_product)  # in radian
    pred_score_for_methods["Fl_angle"] = angle_against_to_predict
    pred_score_for_methods["Fl_angle_degree"] = math.degrees(angle_against_to_predict)

    # TODO: more methods

    # Add result object
    res_dist.append(pred_score_for_methods)


""" Export predict to file """
with open(f"result_predict.csv", 'w') as result_predict_csvfile:
    result_writer = csv.writer(result_predict_csvfile, delimiter=";")
    result_writer.writerow(["Punkt", *pred_methods])
    for i in range(0, len(list_naht_csv_file)):
        result_writer.writerow([
            list_naht_csv_file[i], *[res_dist[i][method] for method in pred_methods]
        ])


# if SHOW_OBJECT_WITH_ALL_POSES:
#     o3d.visualization.draw_geometries(elements)
    # TODO: how to export to image???
