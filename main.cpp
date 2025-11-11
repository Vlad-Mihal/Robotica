#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h> // Include chiave
#include <Eigen/Geometry>
#include <cmath> // Per std::isfinite

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main()
{
    // --- 1. Caricare la Mappa Globale ---
    PointCloudT::Ptr map_cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>("../risultati_01_map.pcd", *map_cloud) == -1) //../risultati_00_map.pcd" e ../risultati_01_map.pcd"
    {
        PCL_ERROR("Impossibile caricare il file map.pcd\n");
        return (-1);
    }
    std::cout << "Caricata mappa globale con " << map_cloud->size() << " punti." << std::endl;


    // --- 2. Definire la Pose Virtuale (R, t) ---
    Eigen::Vector3f t(10.0f, 0.0f, 1.0f); 
    Eigen::Quaternionf q(1.0f, 0.0f, 0.0f, 0.0f);
    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    sensor_pose.translation() = t;
    sensor_pose.rotate(q);


    // --- 3. Impostare i Parametri del Sensore Virtuale (Simuliamo un VLP-16) ---
    float angular_resolution_x = (float) (0.2 * (M_PI/180.0));
    float angular_resolution_y = (float) (1.0 * (M_PI/180.0));
    float max_angle_width = (float) (360.0 * (M_PI/180.0));
    float max_angle_height = (float) (30.0 * (M_PI/180.0));


    // --- 4. Calcolo Vista Virtuale (Ray Tracing con RangeImage) ---
    std::cout << "Avvio calcolo vista virtuale (Ray Tracing)..." << std::endl;

    pcl::RangeImage range_image;
    range_image.createFromPointCloud(
        *map_cloud,             // La mappa globale
        angular_resolution_x,
        angular_resolution_y,
        max_angle_width,
        max_angle_height,
        sensor_pose,            // La nostra posa virtuale
        pcl::RangeImage::CAMERA_FRAME, 
        0.0, 0.0, 0
    );

    // --- MODIFICA CHIAVE ---
    // Convertiamo la RangeImage (che è PointCloud<PointWithRange>) 
    // in una PointCloud<PointXYZ> (scartando i punti invalidi).

    PointCloudT::Ptr virtual_view_cloud(new PointCloudT);

    for (const auto& point_with_range : range_image.points)
    {
        // std::isfinite controlla che il raggio non sia infinito (cioè non abbia colpito nulla)
        if (std::isfinite(point_with_range.range))
        {
            // Copiamo solo i valori X, Y, Z
            virtual_view_cloud->points.push_back(
                PointT(point_with_range.x, point_with_range.y, point_with_range.z)
            );
        }
    }
    virtual_view_cloud->width = virtual_view_cloud->size();
    virtual_view_cloud->height = 1;
    virtual_view_cloud->is_dense = true;


    // Ora il numero di punti dovrebbe essere MOLTO minore e realistico!
    std::cout << "Vista virtuale generata con " << virtual_view_cloud->size() << " punti." << std::endl;


    // --- 5. Salvare la Vista Virtuale ---
    pcl::io::savePCDFileBinary("vista_virtuale_mondo.pcd", *virtual_view_cloud);
    std::cout << "Salvata vista_virtuale_mondo.pcd" << std::endl;

    return 0;
}
