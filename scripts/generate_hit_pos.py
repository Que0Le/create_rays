#!/usr/bin/env python3
import pybullet as p
import pybullet_data as pd
import xml.etree.ElementTree as ET
import numpy as np
import os
import csv, math
import sys, getopt

numRays = 32
draw_breams = False

COMP_DIR = "components"
component_dirs = os.listdir(COMP_DIR)

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


with open(f"naht/all.csv", 'w') as all_csvfile:
    all_writer = csv.writer(all_csvfile, delimiter=",")
    all_writer.writerow([
        "filename", "Name", "ZRotLock", "WkzName", "WkzWkl",
        "Punkt_Pos", "Punkt_Fl_Norm1", "Punkt_Fl_Norm2",
        "Frame_Pos", "Frame_Xvek", "Frame_Yvek", "Frame_Zvek",
        "hit_file"
    ])
    for component_dir in component_dirs:
        path_xml = os.path.join(COMP_DIR, component_dir, component_dir+'.xml')

        object_id = p.loadURDF(
            './components/'+component_dir+'/model.urdf',
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
                hit_file_for_naht = f"{component_dir}.{Name}.{WkzName}.{WkzWkl}.{i}.csv"
                punk_xyz = np.array([
                    float(Punkt_s[i].get('X')), float(
                        Punkt_s[i].get('Y')), float(Punkt_s[i].get('Z'))
                ])
                print(f"Working: punk_xyz={punk_xyz}")

                fl_norms = []
                for Fl_Norm in Punkt_s[i].findall('Fl_Norm'):
                    fl_norms.append(([
                        float(Fl_Norm.get('X')), float(
                            Fl_Norm.get('Y')), float(Fl_Norm.get('Z'))
                    ]))

                frame_veks = []
                Pos = Frame_s[i].findall('Pos')[0]
                XVek = Frame_s[i].findall('XVek')[0]
                YVek = Frame_s[i].findall('YVek')[0]
                ZVek = Frame_s[i].findall('ZVek')[0]
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

                hit_pos = []
                nor_pos = []
                hit_fraction = []
                # Run sim
                """  """

                rayFrom = []
                rayTo = []
                rayIds = []
                rayLen = 20

                rayHitColor = [x / 255 for x in [255, 0, 0]]
                rayMissColor = [x / 255 for x in [0, 255, 0]]
                
                # pybullet.MAX_RAY_INTERSECTION_BATCH_SIZE
                # 16384  2^14
                ray_th = 0
                for xy in range(numRays):
                    for z in range(numRays):
                        rayFrom.append(punk_xyz/1000)
                        phi = 2. * math.pi * float(xy) / numRays
                        theta = 2. * math.pi * float(z) / numRays
                        rayTo.append([
                            rayLen * math.sin(phi) * math.cos(theta),
                            rayLen * math.sin(phi) * math.sin(theta),
                            rayLen * math.cos(phi)
                        ])

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
                        hit_fraction.append(-1)
                        hit_pos.append(np.array([0,0,0]))
                        nor_pos.append(np.array([0,0,0]))
                """  """
                with open(f"naht/{hit_file_for_naht}", 'w') as naht_csvfile:
                    naht_writer = csv.writer(naht_csvfile, delimiter=";")
                    naht_writer.writerow([
                        "WkzWkl", "WkzName",
                        "Punkt_Pos", "Punkt_Fl_Norm1", "Punkt_Fl_Norm2",
                        "Frame_Pos", "Frame_Xvek", "Frame_Yvek", "Frame_Zvek",
                        "hit_fraction", "pos", "nor_pos"
                    ])
                    for hit_th in range(0, len(hit_pos)):
                        naht_writer.writerow([
                            WkzWkl, WkzName,
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
        break

# start simulator
# while True:
#     p.stepSimulation()
