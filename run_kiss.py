import kiss_icp # Importa il pacchetto principale KISS-ICP
from kiss_icp.pipeline import OdometryPipeline #Importa la classe principale che gestisce l’intera pipeline di odometria
from kiss_icp.config import KISSConfig #Importa la classe per creare una configurazione personalizzata dell’algoritmo
from kiss_icp.datasets import dataset_factory #Importa una funzione che crea automaticamente un dataset leggendo i file
import os #Serve per gestire percorsi di file e cartelle
import numpy as np #Importa NumPy per operazioni matematiche e matriciali
import open3d as o3d #Importa Open3D, necessario per salvare la mappa in formato .pcd

# --- 1. IMPOSTAZIONI ---
#dataset_root_dir = "/home/.../Scrivania/Robotica_Esame/unipr_dia_00/unipr_dia/"
#sequence_to_process = "00"
#data_dir = os.path.join(dataset_root_dir, sequence_to_process, "velodyne") #Combina i percorsi per ottenere la cartella esatta dei file .bin LiDAR
#output_poses_file = "risultati_00_poses.txt"
#output_map_file = "risultati_00_map.pcd"

dataset_root_dir = "/home/.../Scrivania/Robotica_Esame/unipr_dia_01/unipr_dia/"
sequence_to_process = "01"
data_dir = os.path.join(dataset_root_dir, sequence_to_process, "velodyne") 
output_poses_file = "risultati_01_poses.txt"
output_map_file = "risultati_01_map.pcd"

print(f"Avvio Kiss-ICP sulla cartella dei file: {data_dir}")

# --- 2. CREA LA CONFIGURAZIONE DA ZERO ---
print("Creazione configurazione...")
config_obj = KISSConfig()
config_obj.data.max_range = 100.0 #Imposta la distanza massima (in metri)
config_obj.data.min_range = 0.0 #Imposta la distanza minima (in metri)
config_obj.mapping.voxel_size = 1.0 #Sceglie la dimensione del voxel usato per downsample della mappa  
config_obj.data.deskew = True #Abilita la correzione dei distorsioni dovute al movimento del sensore
print(f"Configurazione creata. Voxel size: {config_obj.mapping.voxel_size}, Max Range: {config_obj.data.max_range}")

# --- 3. CREA L'OGGETTO DATASET (usando la factory) ---
print("Caricamento dataset...")
dataset = dataset_factory( #Crea un dataset utilizzando i file .bin nella cartella velodyne
    dataloader="generic", #Significa che vengono letti file binari standard come in KITTI
    data_dir=data_dir,
    config=config_obj
)

# --- 4. CREA E ESEGUI LA PIPELINE ---
print(f"Creazione pipeline... Trovati {len(dataset)} frame.") #Stampa il numero di scansioni LiDAR disponibili nel dataset
pipeline = OdometryPipeline(dataset) #Crea la pipeline di odometria passando il dataset
print(f"Avvio elaborazione...")
pipeline.run()  #Avvia l’algoritmo KISS-ICP. 
                #Esegue frame-by-frame:
                #deskew
                #estrazione features
                #ICP
                #aggiornamento della mappa
                #salvataggio pose

# --- 5. SALVA I RISULTATI! (Versione finale) ---
print("Elaborazione completata. Salvataggio della mappa e delle pose...")

# 1. SALVATAGGIO POSE 
poses_calcolate = pipeline.poses #Estrae le pose stimate (matrici 4×4) dalla pipeline
pipeline.save_poses_kitti_format(output_poses_file, poses_calcolate) #Salva le pose nel formato KITTI (una matrice per riga)
print(f"Pose salvate con successo in: {output_poses_file}")

# 2. SALVATAGGIO MAPPA
print("Salvataggio della mappa globale...")

# Chiamiamo la funzione 'point_cloud()' trovata con dir()
punti_mappa = pipeline.odometry.local_map.point_cloud() #Estrae tutti i punti presenti nella mappa interna della pipeline

# Creazione oggetto PointCloud di Open3D
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(punti_mappa)

# Salvataggio file .pcd
o3d.io.write_point_cloud(output_map_file, pcd) #Scrive la mappa su disco.

print(f"\n--- FATTO! ---")
print(f"Mappa salvata in:    {output_map_file}")
print(f"Pose salvate in: {output_poses_file}")
