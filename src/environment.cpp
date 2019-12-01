/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_rays = false;
    bool render_cloud = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_cluster = true;
    bool render_box = true;
    bool render_pca_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud = lidar->scan();
    if (render_rays){
        renderRays(viewer, lidar->position, in_cloud);
    }
    if (render_cloud){
        renderPointCloud(viewer, in_cloud, "Point Cloud");
    }

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(in_cloud, 100, 0.2);
    if (render_obst){
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    }
    if (render_plane){
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        if (render_cluster){
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box){
            if (render_pca_box){
                BoxQ box = pointProcessor->BoundingBoxQ(cluster);
                renderBox(viewer, box, clusterId);
            } else {
                Box box = pointProcessor->BoundingBox(cluster);
                renderBox(viewer, box, clusterId);
            }
        }
        clusterId++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderInputCloud = false;
    bool render_obst = false;
    bool render_plane = true;


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    if (renderInputCloud){
        renderPointCloud(viewer,inputCloud,"inputCloud");
    }

    //// Step 0: Filter cloud
    std::cout << "Number of points before filtering -> inputCloud: " << inputCloud->points.size() << std::endl;
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);

    // filterRes = 0.25, minPoint.coordinates/{6,6,10}, maxPoint.coordinates/{2,6,10} seem to work well
    Eigen::Vector4f minPt(minPoint.x/6, minPoint.y/6, minPoint.z/10, 1.0);
    Eigen::Vector4f maxPt(maxPoint.x/2, maxPoint.y/6, maxPoint.z/10, 1.0);
    std::cout << "maxPoint:\n" << maxPoint << std::endl;
    std::cout << "minPoint:\n" << minPoint << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, minPt, maxPt);
    renderPointCloud(viewer, filteredCloud, "filteredCloud");
    std::cout << "Number of points after filtering -> filteredCloud: " << filteredCloud->points.size() << std::endl;

    //// Step 1: Segment filtered cloud into 2 parts: Road and Obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    if (render_obst){
        renderPointCloud(viewer, segmentedCloud.first, "obstCloud", Color(1,0,0));
    }
    if (render_plane){
        renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(0,1,0));
    }

    //// Step 2: Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedCloud.first, 0.75, 25, 1000);

    // Colors: R, B, Y
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(1, 1, 0)};
    int colorId = 0;
    int clusterId = 0;
    while (clusterId < cloudClusters.size()){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster = cloudClusters.at(clusterId);
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        if (colorId == cloudClusters.size()){
            colorId = 0;
        }
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[colorId]);

        //// Step 3: Find bounding boxes for the clusters
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        colorId++;
        clusterId++;
    }

}

// City Block w/ streaming (Good explanation for "Reference to a Pointer": https://www.youtube.com/watch?v=7HmCb343xR8)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    // ------------------------------------------------------------
    // -----Open 3D viewer and display City Block w/ Streaming-----
    // ------------------------------------------------------------

    // RENDER OPTIONS
    bool render_obst = false;
    bool render_plane = true;

    //// Step 0: Filter cloud
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);

    // filterRes = 0.25, minPoint.coordinates/{6,6,10}, maxPoint.coordinates/{2,6,10} seem to work well
    Eigen::Vector4f minPt(minPoint.x/6, minPoint.y/6, minPoint.z/10, 1.0);
    Eigen::Vector4f maxPt(maxPoint.x/2, maxPoint.y/6, maxPoint.z/10, 1.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, minPt, maxPt);

    //// Step 1: Segment filtered cloud into 2 parts: Road and Obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    if (render_obst){
        renderPointCloud(viewer, segmentedCloud.first, "obstCloud", Color(1,0,0));
    }
    if (render_plane){
        renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(0,1,0));
    }

    //// Step 2: Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedCloud.first, 0.75, 25, 1000);

    // Colors: R, B, Y
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0)};
    int colorId = 0;
    int clusterId = 0;
    while (clusterId < cloudClusters.size()){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster = cloudClusters.at(clusterId);
        if (colorId == cloudClusters.size()){
            colorId = 0;
        }
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[colorId]);

        //// Step 3: Find bounding boxes for the clusters
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        colorId++;
        clusterId++;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


//// Main function w/o streaming pcd
//int main (int argc, char** argv)
//{
//    bool renderCityBlock = true;
//    std::cout << "starting enviroment" << std::endl;
//
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    CameraAngle setAngle = XY;
//    initCamera(setAngle, viewer);
//    if (renderCityBlock){
//        cityBlock(viewer);
//    } else {
//        simpleHighway(viewer);
//    }
//
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce ();
//    }
//}

//// Main function w/ streaming pcd
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()){
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    }
}