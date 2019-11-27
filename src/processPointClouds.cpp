// PCL lib Functions for processing point clouds 
#include <unordered_set>
#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Downsample point cloud using voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr reducedCloud(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> pointReduction;
    pointReduction.setInputCloud(cloud);
    pointReduction.setLeafSize(filterRes, filterRes, filterRes);
    pointReduction.filter(*reducedCloud);

    // Perform RoI based filtering on downsampled point cloud
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> filterRoI(true);
    filterRoI.setMin(minPoint);
    filterRoI.setMax(maxPoint);
    filterRoI.setInputCloud(reducedCloud);
    filterRoI.filter(*filteredCloud);

    // Perform RoI based filtering on filtered point cloud to remove roof points
    // http://docs.pointclouds.org/trunk/classpcl_1_1_crop_box.html#af16f9210caa470a230f9a6e0fda41235
    std::vector<int> roofIndices;
    pcl::CropBox<PointT> roofRoI(true);
    Eigen::Vector4f minPt(-1.5, -1.5, -1.0, 1.0);
    Eigen::Vector4f maxPt(3.0, 1.5, 1.0, 1.0);
    filterRoI.setMin(minPt);
    filterRoI.setMax(maxPt);
    filterRoI.setInputCloud(filteredCloud);
    filterRoI.filter(roofIndices);

    // Filter roof points from filtered point cloud
    typename pcl::PointCloud<PointT>::Ptr roofFilteredCloud (new pcl::PointCloud<PointT>());
    pcl::PointIndices::Ptr roofInliers {new pcl::PointIndices ()};
    for (int i {0}; i<roofIndices.size(); i++){
        roofInliers->indices.emplace_back(roofIndices[i]);
    }

    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers: // http://pointclouds.org/documentation/tutorials/planar_segmentation.php
    extract.setInputCloud(filteredCloud);
    extract.setIndices(roofInliers);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    /*
    std::cout << "cloud size: " << cloud->points.size() << std::endl;
    std::cout << "reducedCloud size: " << reducedCloud->points.size() << std::endl;
    std::cout << "filteredCloud (w/ roof) size: " << filteredCloud->points.size() << std::endl;
    std::cout << "roofIndices size: " << roofIndices.size() << std::endl;
    std::cout << "filteredCloud (w/o roof) size: " << filteredCloud->points.size() << std::endl;
    */

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (std::size_t i {0}; i<inliers->indices.size(); i++){
        planeCloud->points.emplace_back(cloud->points.at(inliers->indices.at(i)));
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers: // http://pointclouds.org/documentation/tutorials/planar_segmentation.php
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients  {new pcl::ModelCoefficients ()};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices ()};

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Create KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    // L85: http://pointclouds.org/documentation/tutorials/cluster_extraction.php
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
            cloudCluster->points.emplace_back(cloud->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.emplace_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    std::unordered_set<int> inliersResult;

    // TODO: Fill in this function
    // For max iterations
    for (int i = 0; i < maxIterations; i++) {

        std::unordered_set<int> indices;

        // Randomly sample subset and fit line
        while (indices.size() < 3) {
            indices.emplace(rand() % cloud->points.size());
        }

        // Initialize iterator to unordered_set
        auto idxPtr = indices.begin();
        int pt1 = *idxPtr;
        idxPtr++;
        int pt2 = *idxPtr;
        idxPtr++;
        int pt3 = *idxPtr;

        // Define 2 vectors w/ pt1 as reference
        double v1x = cloud->points[pt2].x - cloud->points[pt1].x;
        double v1y = cloud->points[pt2].y - cloud->points[pt1].y;
        double v1z = cloud->points[pt2].z - cloud->points[pt1].z;

        double v2x = cloud->points[pt3].x - cloud->points[pt1].x;
        double v2y = cloud->points[pt3].y - cloud->points[pt1].y;
        double v2z = cloud->points[pt3].z - cloud->points[pt1].z;

        // Fit line
        double A = v1y * v2z - v1z * v2y;
        double B = v1z * v2x - v1x * v2z;
        double C = v1x * v2y - v1y * v2x;
        double D = -(A * cloud->points[pt1].x + B * cloud->points[pt1].y + C * cloud->points[pt1].z);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for (int j{0}; j < cloud->points.size(); j++) {

            // Continue if j is same as sampled points used for fittint line
            if (indices.count(j) > 0) {
                continue;
            }

            double distance = fabs(A * cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D) /
                              sqrt(A * A + B * B + C * C); // fabs for floats
            if (distance <= distanceThreshold) {
                indices.emplace(j);
            }
        }

        // Return indicies of inliers from fitted line with most inliers
        if (inliersResult.size() < indices.size()) {
            inliersResult = indices;
        }
    }

    for(auto it = inliersResult.begin(); it != inliersResult.end(); it++) {
        inliers->indices.emplace_back(*it);
    }

    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::proximity(const std::vector<std::vector<float>> &points, int id, std::vector<bool> &processed, std::vector<int> *cluster, KdTree *tree, float distanceTol) {
    // Mark point as processed
    processed[id] = true;

    // Add point id to cluster
    cluster->emplace_back(id);

    // Find nearby points to point at index id
    std::vector<int> nearbyPoints = tree->search(points.at(id), distanceTol);

    for (auto idx : nearbyPoints){

        // If point has not been processed then recursively call proximity
        if (!processed[idx]){
            proximity(points, idx, processed, cluster, tree, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol) {
    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);  // For checking if a point has been processed

    for (int id {0}; id<points.size(); id++){

        // If point has not been processed then create cluster and call proximity
        if (!processed[id]){
            std::vector<int> cluster;
            proximity(points, id, processed, &cluster, tree, distanceTol);
            clusters.emplace_back(cluster);
        }
    }

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Create KdTree object for the search method of the extraction
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;

    for (int i {0}; i<cloud->points.size(); i++){
        std::vector<float> point {cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z};
        tree->insert(point, i);
        points.emplace_back(point);
    }

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance);

    for (std::vector<std::vector<int>>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->begin(); pit != it->end(); pit++){
            cloudCluster->points.emplace_back(cloud->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        // Cluster size should be within minSize and maxSize
        if (cloudCluster->width >= minSize && cloudCluster->width <= maxSize){
            clusters.emplace_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    //// From user Nicola Fioraio on the PCL forums. http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
    /*
    1) compute the centroid (c0, c1, c2) and the normalized covariance
    2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
    3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
    4) compute the max, the min and the center of the diagonal
    5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)
    */
    // Compute principal directions
    Eigen::Vector4f centroid;  // Shape: 4 x 1
    pcl::compute3DCentroid(*cluster, centroid);  // Output: centroid
    Eigen::Matrix3f covariance;  // Shape: 3 x 3
    pcl::computeCovarianceMatrixNormalized(*cluster, centroid, covariance);  // Output: covariance
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors();  // Shape: 3 x 3
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));

    std::cout << "eigenVectors:\n" << eigenVectors << std::endl;
    std::cout << "eigenValues:\n" << eigenSolver.eigenvalues() << std::endl;

    // Transform the original clusters to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity()); // Shape: 4 x 4
    projectionTransform.block<3,3>(0,0) = eigenVectors.transpose();  // Assign block of 3 x 3 starting at (0, 0)
    projectionTransform.block<3,1>(0,3) = -1.0f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    std::cout << "projectionTransform\n" << projectionTransform << std::endl;

    // Get the min and max points of the transformed cloud
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectors);
    const Eigen::Vector3f bboxTransform = eigenVectors * meanDiagonal + centroid.head<3>();

    std::cout << "bboxTransform:\n" << bboxTransform << std::endl;
    std::cout << "maxPoint:\n" << maxPoint << std::endl;
    std::cout << "minPoint:\n" << minPoint << std::endl;

    BoxQ box;
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;
    box.cube_length = fabs(maxPoint.x-minPoint.x);
    box.cube_width = fabs(maxPoint.y-minPoint.y);
    box.cube_height = fabs(maxPoint.z-minPoint.z);

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
