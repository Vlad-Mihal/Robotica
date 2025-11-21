#include <iostream>
#include <pcl/io/pcd_io.h>         // Funzioni PCL per leggere/scrivere file .pcd
#include <pcl/point_types.h>       // Definizione dei tipi di punti PCL
#include <pcl/common/transforms.h> // Per applicare trasformazioni alle point cloud
#include <pcl/range_image/range_image.h> // Include necessario per generare RangeImage
#include <Eigen/Geometry>          // Libreria Eigen per trasformazioni 3D
#include <cmath>                   // Per std::isfinite (controllo numeri finiti)
#include <rofl/common/param_map.h> // <-- NUOVO: Include la classe per i parametri

typedef pcl::PointXYZ PointT;      // Alias: PointT = punto con X, Y, Z
typedef pcl::PointCloud<PointT> PointCloudT; // Alias: PointCloudT = nube di PointT

int main(int argc, char** argv)    // <-- MODIFICA: Prendiamo gli argomenti da riga di comando
{
    // --- 0. LETTURA PARAMETRI (Classe rofl::ParamMap) ---
    rofl::ParamMap params;
    params.read(argc, argv);

    // Variabili per i parametri
    float res_x_deg, res_y_deg, fov_h_deg, fov_v_deg;
    float pos_x, pos_y, pos_z;
    std::string input_file, output_file;

    // Leggiamo i parametri (con i TUOI valori come default)
    
    // Parametri Sensore
    params.getParam<float>("res_x", res_x_deg, 0.1f);   // Risoluzione Orizzontale (gradi)
    params.getParam<float>("res_y", res_y_deg, 0.1f);   // Risoluzione Verticale (gradi)
    params.getParam<float>("fov_h", fov_h_deg, 360.0f); // FoV Orizzontale (gradi)
    params.getParam<float>("fov_v", fov_v_deg, 45.0f);  // FoV Verticale (gradi)
    
    // Parametri Posizione (Pose)
    params.getParam<float>("x", pos_x, 10.0f);  // Posizione X
    params.getParam<float>("y", pos_y, 0.0f);   // Posizione Y
    params.getParam<float>("z", pos_z, 0.35f);  // Posizione Z (il tuo valore modificato)

    // Parametri File
    params.getParam<std::string>("in", input_file, "../risultati_01_map.pcd");
    params.getParam<std::string>("out", output_file, "vista_virtuale_mondo.pcd");

    // Stampiamo un riepilogo per conferma
    std::cout << "\n==========================================\n";
    std::cout << "   GENERATORE SCANSIONE VIRTUALE (PCL)    \n";
    std::cout << "==========================================\n";
    std::cout << "Input:  " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Posizione Sensore: (" << pos_x << ", " << pos_y << ", " << pos_z << ")\n";
    std::cout << "Risoluzione Angolare: " << res_x_deg << " x " << res_y_deg << " deg\n";
    std::cout << "Field of View (FoV):  " << fov_h_deg << " x " << fov_v_deg << " deg\n";
    std::cout << "==========================================\n\n";


    // --- 1. Caricare la Mappa Globale ---
    PointCloudT::Ptr map_cloud(new PointCloudT);   // Alloco una PointCloud vuota su puntatore intelligente

    if (pcl::io::loadPCDFile<PointT>("../risultati_01_map.pcd", *map_cloud) == -1)
    {                                               // Carico la mappa globale da file .pcd
        PCL_ERROR("Impossibile caricare il file map.pcd\n");  // Errore se fallisce il caricamento
        return (-1);                                // Esco dal programma con errore
    }

    std::cout << "Caricata mappa globale con " 
              << map_cloud->size() << " punti." 
              << std::endl;                         // Stampo quanti punti sono stati caricati

    // --- 2. Definire la Pose Virtuale (R, t) ---
    // Usiamo le variabili lette dai parametri (pos_x, pos_y, pos_z)
    Eigen::Vector3f t(pos_x, pos_y, pos_z);            // Vettore traslazione
    Eigen::Quaternionf q(1.0f, 0.0f, 0.0f, 0.0f);      // Quaternione identità: nessuna rotazione
    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity(); 
    sensor_pose.translation() = t;                     // Applico la traslazione
    sensor_pose.rotate(q);                             // Applico la rotazione

    // --- 3. Impostare i Parametri del Sensore Virtuale ---
    // Convertiamo i gradi (letti dai parametri) in radianti per PCL
    float angular_resolution_x = (float)(res_x_deg * (M_PI/180.0)); // Risoluzione orizzontale
    float angular_resolution_y = (float)(res_y_deg * (M_PI/180.0)); // Risoluzione verticale
    float max_angle_width      = (float)(fov_h_deg * (M_PI/180.0)); // Campo visivo orizzontale
    float max_angle_height     = (float)(fov_v_deg * (M_PI/180.0)); // Campo visivo verticale

    // --- 4. Calcolo Vista Virtuale (Ray Tracing con RangeImage) ---
    std::cout << "Avvio calcolo vista virtuale (Ray Tracing)..." << std::endl;

    pcl::RangeImage range_image;                      // Creo un oggetto RangeImage
    range_image.createFromPointCloud(
        *map_cloud,                                   // Point cloud globale (la mappa)
        angular_resolution_x,                         // Risoluzione angolare orizzontale
        angular_resolution_y,                         // Risoluzione angolare verticale
        max_angle_width,                              // Ampiezza FOV orizzontale
        max_angle_height,                             // Ampiezza FOV verticale
        sensor_pose,                                  // Pose da cui "simulo" il sensore
        pcl::RangeImage::CAMERA_FRAME,                // Modalità frame (camera-like)
        0.0, 0.0, 0                                   // Parametri opzionali 
    );

    // --- ESTARZIONE PUNTI VALIDI ---
    // Convertiamo la RangeImage in PointCloudXYZ scartando i punti infiniti.
    PointCloudT::Ptr virtual_view_cloud(new PointCloudT);  // Alloco la nuova vista virtuale
    
    for (const auto& point_with_range : range_image.points) // Ciclo su tutti i punti
    {
        if (std::isfinite(point_with_range.range))          // Se il punto è valido
        {
            virtual_view_cloud->points.push_back(
                PointT(point_with_range.x, point_with_range.y, point_with_range.z)
            );                                              // Copio solo X,Y,Z
        }
    }
    virtual_view_cloud->width  = virtual_view_cloud->size(); // Imposto dimensione
    virtual_view_cloud->height = 1;                          // Cloud non strutturata
    virtual_view_cloud->is_dense = true;                     // Tutti i punti sono validi
    
    std::cout << "Vista virtuale generata con " 
              << virtual_view_cloud->size() 
              << " punti." << std::endl;

    // --- 5. Salvare la Vista Virtuale ---
    // Uso il nome file specificato nei parametri (default: vista_virtuale_mondo.pcd)
    pcl::io::savePCDFileBinary(output_file, *virtual_view_cloud);  
    std::cout << "Salvata " << output_file << std::endl;
    
    return 0;
}
