import kiss_icp
from kiss_icp.pipeline import OdometryPipeline
from kiss_icp.config import KISSConfig
from kiss_icp.datasets import dataset_factory
import os
import numpy as np
import open3d as o3d  

# --- 1. IMPOSTAZIONI ---
#dataset_root_dir = "/home/vlad/Scrivania/Robotica_Esame/unipr_dia_00/unipr_dia/"
#sequence_to_process = "00"
#data_dir = os.path.join(dataset_root_dir, sequence_to_process, "velodyne") 
#output_poses_file = "risultati_00_poses.txt"
#output_map_file = "risultati_00_map.pcd"

dataset_root_dir = "/home/vlad/Scrivania/Robotica_Esame/unipr_dia_01/unipr_dia/"
sequence_to_process = "01"
data_dir = os.path.join(dataset_root_dir, sequence_to_process, "velodyne") 
output_poses_file = "risultati_01_poses.txt"
output_map_file = "risultati_01_map.pcd"

print(f"Avvio Kiss-ICP sulla cartella dei file: {data_dir}")

# --- 2. CREA LA CONFIGURAZIONE DA ZERO ---
print("Creazione configurazione...")
config_obj = KISSConfig()
config_obj.data.max_range = 100.0
config_obj.data.min_range = 0.0       
config_obj.mapping.voxel_size = 1.0   
config_obj.data.deskew = True
print(f"Configurazione creata. Voxel size: {config_obj.mapping.voxel_size}, Max Range: {config_obj.data.max_range}")

# --- 3. CREA L'OGGETTO DATASET (usando la factory) ---
print("Caricamento dataset...")
dataset = dataset_factory(
    dataloader="generic",
    data_dir=data_dir,
    config=config_obj
)

# --- 4. CREA E ESEGUI LA PIPELINE ---
print(f"Creazione pipeline... Trovati {len(dataset)} frame.")
pipeline = OdometryPipeline(dataset) 
print(f"Avvio elaborazione...")
pipeline.run() 

# --- 5. SALVA I RISULTATI! (Versione finale) ---
print("Elaborazione completata. Salvataggio della mappa e delle pose...")

# 1. SALVATAGGIO POSE 
poses_calcolate = pipeline.poses 
pipeline.save_poses_kitti_format(output_poses_file, poses_calcolate)
print(f"Pose salvate con successo in: {output_poses_file}")

# 2. SALVATAGGIO MAPPA
print("Salvataggio della mappa globale...")

# Chiamiamo la funzione 'point_cloud()' trovata con dir()
punti_mappa = pipeline.odometry.local_map.point_cloud()

# Creazione oggetto PointCloud di Open3D
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(punti_mappa)

# Salvataggio file .pcd
o3d.io.write_point_cloud(output_map_file, pcd)

print(f"\n--- FATTO! ---")
print(f"Mappa salvata in:    {output_map_file}")
print(f"Pose salvate in: {output_poses_file}")
