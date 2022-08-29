#!/usr/bin/env python3
import pybullet as p
import pybullet_data as pd
import xml.etree.ElementTree as ET
import numpy as np
import os
import csv
import math

COMP_DIR = "components"
component_dirs = os.listdir(COMP_DIR)

physicsClient = p.connect(p.GUI)

def show_welding_target(position):
    visual_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1,0,0,1])
    point_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_id,
                basePosition=position,
                )

with open(f"naht/all.csv",'w') as all_csvfile:
    all_writer = csv.writer(all_csvfile, delimiter=",")
    all_writer.writerow([
        "filename", "Name", "ZRotLock", "WkzName", "WkzWkl",
        "Punkt_Pos", "Punkt_Fl_Norm1", "Punkt_Fl_Norm2",
        "Frame_Pos", "Frame_Xvek", "Frame_Yvek", "Frame_Zvek",
        "hit_file"
    ])
    for component_dir in component_dirs:
        path_xml = os.path.join(COMP_DIR, component_dir, component_dir+'.xml')

        object_id=p.loadURDF(
            './components/'+component_dir+'/model.urdf', 
            basePosition=[0,0,0],  #r:x
            baseOrientation=[0,0,0,1], 
            useFixedBase=True
        )

        tree = ET.parse(path_xml)
        root = tree.getroot()
        # Baugruppe = root.findall('Baugruppe')
        # print(Baugruppe)
        # break
        for SNaht in root.findall('SNaht'):
            Name = SNaht.get('Name')
            ZRotLock = SNaht.get('ZRotLock')
            WkzWkl = SNaht.get('WkzWkl')
            WkzName =  SNaht.get('WkzName')

            # Kontur = SNaht.findall('Kontur')
            # Frames = SNaht.findall('Frames')
            Punkt_s = SNaht.findall('Kontur')[0].findall('Punkt')
            Frame_s = SNaht.findall('Frames')[0].findall('Frame')

            for i in range(0, len(Punkt_s)):
                hit_file_for_naht = f"{component_dir}.{Name}.{WkzName}.{WkzWkl}.{i}.csv"
                punk_xyz = np.array([
                    float(Punkt_s[i].get('X')), float(Punkt_s[i].get('Y')), float(Punkt_s[i].get('Z'))
                ])
                print(f"Working: punk_xyz={punk_xyz}")
                # show_welding_target((punk_xyz.astype(float))/1000)
                fl_norms = []
                for Fl_Norm in Punkt_s[i].findall('Fl_Norm'):
                    fl_norms.append(([
                        float(Fl_Norm.get('X')), float(Fl_Norm.get('Y')), float(Fl_Norm.get('Z'))
                    ]))

                frame_veks = []
                Pos = Frame_s[i].findall('Pos')[0]
                XVek = Frame_s[i].findall('XVek')[0]
                YVek = Frame_s[i].findall('YVek')[0]
                ZVek = Frame_s[i].findall('ZVek')[0]
                frame_pos = [float(Pos.get('X')), float(Pos.get('Y')), float(Pos.get('Z'))]
                frame_veks.append(([
                    float(XVek.get('X')), float(XVek.get('Y')), float(XVek.get('Z'))
                ]))
                frame_veks.append(([
                    float(YVek.get('X')), float(YVek.get('Y')), float(YVek.get('Z'))
                ]))
                frame_veks.append(([
                    float(ZVek.get('X')), float(ZVek.get('Y')), float(ZVek.get('Z'))
                ]))

                hit_pos = []
                nor_pos = []
                hit_fraction = []
                # Run sim
                """  """

                rayFrom = []
                rayTo = []
                rayIds = []
                numRays = 64
                rayLen = 20

                rayHitColor = [x / 255 for x in [255, 0, 0]]
                rayMissColor = [x / 255 for x in [0, 255, 0]]

                draw_breams = True
                if draw_breams:
                    replaceLines = True
                    # pybullet.MAX_RAY_INTERSECTION_BATCH_SIZE
                    # 16384  2^14
                    ray_th = 0
                    for xy in range(numRays):
                        for z in range(numRays):
                            # rayFrom.append([0, 0, 1])
                            rayFrom.append(punk_xyz/1000)
                            # rayFrom.append([5, 5, 0])
                            phi = 2. * math.pi * float(xy) / numRays
                            theta = 2. * math.pi * float(z) / numRays
                            rayTo.append([
                                rayLen * math.sin(phi) * math.cos(theta),
                                rayLen * math.sin(phi) * math.sin(theta),
                                rayLen * math.cos(phi) 
                            ])
                            # if (replaceLines):
                            #     rayIds.append(p.addUserDebugLine(rayFrom[ray_th], rayTo[ray_th], rayMissColor))
                            # else:
                            #     rayIds.append(-1)
                            # ray_th = ray_th +1

                    results = p.rayTestBatch(rayFrom, rayTo)

                    for i in range(0, len(results)):
                        hitObjectUid = results[i][0]
                        if (hitObjectUid < 0):
                            hitPosition = [0, 0, 0]
                            # p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
                            # p.removeUserDebugItem(itemUniqueId=rayIds[i])
                            pass
                        else:
                            hitPosition = results[i][3]
                            # print(f"hit object: {results[i][0]} link index {results[i][1]} hit fraction {results[i][2]} hit pos {results[i][3]} hit nor {results[i][4]}")
                            hit_fraction.append(results[i][2])
                            hit_pos.append(results[i][3])
                            nor_pos.append(results[i][4])
                            # p.addUserDebugText(text=str(results[i][3]) + str(results[i][4]), textPosition=results[i][3])
                            # p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor)
                """  """
                with open(f"naht/{hit_file_for_naht}",'w') as naht_csvfile:
                    naht_writer = csv.writer(naht_csvfile, delimiter=";")
                    # writer.writerow(["Date", "temperature 1", "Temperature 2"])
                    naht_writer.writerow([
                        "WkzWkl", "WkzName",
                        "Punkt_Pos", "Punkt_Fl_Norm1", "Punkt_Fl_Norm2",
                        "Frame_Pos", "Frame_Xvek", "Frame_Yvek", "Frame_Zvek",
                        "pos", "nor_pos"
                    ])
                    for hit_th in range(0, len(hit_pos)):
                        naht_writer.writerow([
                            WkzWkl, WkzName,
                            " ".join(str(x) for x in punk_xyz), 
                            " ".join(str(x) for x in fl_norms[0]), 
                            " ".join(str(x) for x in fl_norms[1]),
                            ###
                            " ".join(str(x) for x in frame_pos), 
                            " ".join(str(x) for x in XVek), 
                            " ".join(str(x) for x in YVek), 
                            " ".join(str(x) for x in ZVek), 
                            ###
                            " ".join(str(x) for x in hit_pos[hit_th]), 
                            " ".join(str(x) for x in nor_pos[hit_th]), 
                        ])
                # break
            # break
        break

# start simulator
# while True:
#     p.stepSimulation() 
