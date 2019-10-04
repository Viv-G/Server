#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/common/time.h>

using namespace std::chrono_literals;

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_zxy (new pcl::PointCloud<pcl::PointXYZ> ());
  // Load bun0.pcd -- should be available with the PCL archive in test 
  pcl::io::loadPCDFile ("points.pcd", *cloud_filtered_zxy);

  float res = 0;
  double rad = 0;
  double radDown = 0;
  int track = 0;
  double upRad = 0;
  double upStep = 0;
  int depth = 0;
  int downSample = 0;
  int nPick = 0;
  double neRad = 0; // 0.05
  int minNei = 0;
  int upsample = 0;
  int iter = 0;
  float vSize = 0;


  // TIMING INITS

  //double crop_time, vox_time, start_time, mlsNormal_time, mlsNoNormal_time, estNormal_time, mesh_time, end_time = 0;
  double start_time, end_time = 0;
  double elapsed_crop, elapsed_vox, elapsed_mls, elapsed_mlsNoNormal, elapsed_Normal, elapsed_Mesh, elapsed_rad = 0;


  while (downSample == 0 && track < 2){
  std::cout << "How would you like to Downsample? \n\t0 = None\n\t 1 = VoxGrid \n\t 2 = Radius Outlier Removal" << std::endl;
  std::cin >> downSample;
  track++;
  }
  track = 0;

    if (downSample == 2){
    while (radDown == 0 && track < 2){
    std::cout << "ENTER Dowsnsample Search Radius \n";
    std::cin >> radDown;
    track++;
    }
    track = 0;
    while (minNei == 0 && track < 2){
    std::cout << "ENTER Min Neighbors (800 Default) \n";
    std::cin >> minNei;
    track++;
    }
    track = 0;
}

  if (downSample == 1){
    while (res == 0 && track < 2){
    std::cout << "ENTER vox grid Value (0.1 std) \n";
    std::cin >> res;
    track++;
    }
    track = 0;
}

  
  while (nPick== 0 && track < 2){
  std::cout << "How would you like to estimate Normals? \n\t 1 = Moving Least Squares \n\t 2 = Normal Estimation" << std::endl;
  std::cin >> nPick;
  track++;
  }
  track = 0;


  while (rad == 0 && track < 2){
  std::cout << "ENTER MLS radius search value (0.01 std)\n";
  std::cin >> rad;
  track++;
  }
  track = 0;

/*  track = 0;
  while (upRad == 0 && track < 2){
  std::cout << "ENTER Upsampling Radius (0.005 Std) \n";
  std::cin >> upRad;
  track++;
  }
  track = 0;

  while (upStep == 0 && track < 2){
  std::cout << "ENTER Upsampling Step Size (0.003 Std) \n";
  std::cin >> upStep;
  track++;
  }
  track = 0; */

  if (nPick == 2){
    while (neRad == 0 && track < 2){
    std::cout << "ENTER Normal Estimation radius search value (0.05 std)\n";
    std::cin >> neRad;
    track++;
    }
    track = 0;
    }

    while (upsample == 0 && track < 2){
      std::cout << "How would you like to Upsample Cloud? \n\t 1 = None \n\t 2 = Sample Local Plane \n\t 3 = VOX Grid" << std::endl;
      std::cin >> upsample;
      track++;
    }
    track = 0;

    // 1 = No upsample
    // 2 = Sample Local Plane
    // 3 = VOX GRID
    // 4 = Bilateral filtering upsampling
    if (upsample == 2) {
        while (upRad == 0 && track < 2){
          std::cout << "ENTER Upsampling Radius (0.005 Std) \n";
          std::cin >> upRad;
          track++;
        }
        track = 0;

        while (upStep == 0 && track < 2){
        std::cout << "ENTER Upsampling Step Size (0.003 Std) \n";
        std::cin >> upStep;
        track++;
        }
        track = 0;
      }


    else if (upsample == 3){
        while (vSize == 0 && track < 2){
          std::cout << "ENTER Voxel Size (0.001 Std) \n";
          std::cin >> vSize;
          track++;
        }
        track = 0;

        while (iter == 0 && track < 2){
          std::cout << "ENTER Upsampling iterations (1 Std) \n";
          std::cin >> iter;
          track++;
        }
        track = 0;
      }

  while (depth == 0 && track < 2){
  std::cout << "ENTER DEPTH (9 STANDARD): \n";
  std::cin >> depth;
  track++;
  }
  track = 0;

  start_time = pcl::getTime ();

 /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_zx (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_zxy (new pcl::PointCloud<pcl::PointXYZ>);
  // Filter Z Distance (Depth from original frame)
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 3);
  pass.filter (*cloud_filtered_z);
  // FIlter X distance (Probably not needed)
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud (cloud_filtered_z);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-1, 1);
  pass_x.filter (*cloud_filtered_zx);
  // Filter Y Distances (... Probably not needed)
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud (cloud_filtered_zx);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-1.7, -0.5);
  pass_y.filter (*cloud_filtered_zxy); */

  std::cout << "Filtered Points Was:" << cloud->points.size() << "Now: " << cloud_filtered_zxy->points.size() << " - Starting Smoothing...\n" << std::endl;
  end_time = pcl::getTime ();
  elapsed_crop = end_time - start_time;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

if (downSample == 1)
{ 
  start_time = pcl::getTime ();
  std::cout << "Downsampling Points...\n";
  // create passthrough filter instance
 pcl::VoxelGrid<pcl::PointXYZ> vox;
 // set input cloud
 vox.setInputCloud (cloud_filtered_zxy);
 // set cell/voxel size to 0.1 meters in each dimension
 vox.setLeafSize (res, res, res);
  // do filtering
 vox.filter (*cloud_downsampled);
 std::cout << "Success... Reduced from " << cloud_filtered_zxy->points.size() << " to " << cloud_downsampled->points.size() << std::endl;
 end_time = pcl::getTime ();
 elapsed_vox = end_time - start_time;
}

if (downSample == 2)
{ 
  start_time = pcl::getTime ();
  std::cout << "Filtering... Points Radius Outlier...\n";
  // create passthrough filter instance
 pcl::RadiusOutlierRemoval<pcl::PointXYZ> r_rem;
 // set input cloud
 r_rem.setInputCloud (cloud_filtered_zxy);
 // set cell/voxel size to 0.1 meters in each dimension
 r_rem.setRadiusSearch(radDown);
  // do filtering
 r_rem.setMinNeighborsInRadius(minNei);
 r_rem.filter (*cloud_downsampled);
 std::cout << "Success... Reduced from " << cloud_filtered_zxy->points.size() << " to " << cloud_downsampled->points.size() << std::endl;
 end_time = pcl::getTime ();
 elapsed_rad = end_time - start_time;
}

  pcl::io::savePCDFile ("DownSampleCloud.pcd", *cloud_downsampled);
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals (new pcl::PointCloud<pcl::PointNormal>);


if (nPick == 1) {
    // Create a KD-Tree
  start_time = pcl::getTime ();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  // Set parameters
  if (downSample > 0) { 
    mls.setInputCloud (cloud_downsampled);
  }
  else {
    mls.setInputCloud (cloud_filtered_zxy);
  }
  mls.setSearchRadius(rad);
//  mls.setPolynomialFit (true); DEPRECATED
  mls.setPolynomialOrder (0);
  mls.setSearchMethod(tree);
  if (upsample == 2) {
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal>::SAMPLE_LOCAL_PLANE); 
  mls.setUpsamplingRadius(upRad);
  mls.setUpsamplingStepSize(upStep);
}
else if (upsample == 3){
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal>::VOXEL_GRID_DILATION); 
  mls.setDilationVoxelSize(vSize);
  mls.setDilationIterations(iter);
}
  // Reconstruct
  mls.process(*cloud_point_normals);

      for (size_t i = 0; i < cloud_point_normals->size(); ++i)
  {
    cloud_point_normals->points[i].normal_x *= -1;
    cloud_point_normals->points[i].normal_y *= -1;
    cloud_point_normals->points[i].normal_z *= -1;
  }
  
  std::cout << "Smothed Points And Calculated Normals Now " << cloud_point_normals->points.size() <<"Points ... Starting Poisson Meshing\n" << std::endl;
  end_time = pcl::getTime ();
  elapsed_mls = end_time - start_time;
}

if (nPick == 2) {
  start_time = pcl::getTime();
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZ>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
 // mls.setComputeNormals (true);
  // Set parameters
  if (downSample > 0) { 
    mls.setInputCloud (cloud_downsampled);
  }
  else {
    mls.setInputCloud (cloud_filtered_zxy);
  }
  mls.setSearchRadius(rad);
  mls.setPolynomialOrder (0);
  if (upsample == 2) {
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ>::SAMPLE_LOCAL_PLANE); 
  mls.setUpsamplingRadius(upRad);
  mls.setUpsamplingStepSize(upStep);
}
else if (upsample == 3){
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ>::VOXEL_GRID_DILATION); 
  mls.setDilationVoxelSize(vSize);
  mls.setDilationIterations(iter);
}
  // Reconstruct
  mls.process(*mls_points);
  end_time = pcl::getTime ();
  elapsed_mlsNoNormal = end_time - start_time;

  std::cout << "Smothed Points Now " << mls_points->points.size() <<" - Starting Normal Estimation...\n" << std::endl;
  
  start_time = pcl::getTime();
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads (2);
  ne.setInputCloud(mls_points);
  ne.setRadiusSearch(neRad);
  Eigen::Vector4f centroid;
  compute3DCentroid(*mls_points, centroid);
  ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_normals);

  for (size_t i = 0; i < cloud_normals->size(); ++i)
  {
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }


pcl::concatenateFields(*mls_points,*cloud_normals, *cloud_point_normals);
 end_time = pcl::getTime ();
elapsed_Normal = end_time - start_time;
std::cout << "Calculated Normals - Starting Poisson Meshing...\n";
}

///// MESHING/////
//////////////////
start_time = pcl::getTime();
pcl::Poisson<pcl::PointNormal> poisson;
poisson.setDepth(depth);
poisson.setInputCloud(cloud_point_normals);
pcl::PolygonMesh mesh;
poisson.reconstruct(mesh);


 end_time = pcl::getTime ();
 elapsed_Mesh = end_time - start_time;

pcl::io::saveVTKFile ("poisson.vtk", mesh);
pcl::io::savePCDFile ("SmoothedCloud.pcd", *cloud_point_normals);
pcl::io::savePCDFile ("Filtered.pcd", *cloud_downsampled);

std::cout << "Success... - Displaying Result...\n";


/*  if (nPick == 1) { 
    if (downSample == 1){
      elapsed_mls = mlsNormal_time - vox_time;
    }

  else {
    elapsed_mls = mlsNormal_time - crop_time;
  }

   elapsed_Mesh = mesh_time - mlsNormal_time;
}
if (nPick == 2) {
  if (downSample == 1){
    elapsed_mlsNoNormal = mlsNoNormal_time - vox_time;
  }
  else {
    elapsed_mlsNoNormal = mlsNoNormal_time - crop_time;
  }
  elapsed_Normal = estNormal_time - mlsNoNormal_time;
  elapsed_Mesh = mesh_time - estNormal_time;
} */

ofstream Logfile;
Logfile.open("Log.txt", ios::out | ios::trunc );

Logfile << "Settings: \n" << std::endl;
if (downSample == 1){
  Logfile << "Downsampled: YES - VOX" << std::endl;
  Logfile << "Vox Resolution: " << res << std::endl;
}
else if  (downSample == 2){
  Logfile << "Downsampled: YES - Radius Outlier Removal" << std::endl;
  Logfile << "Radius: " << radDown << std::endl;
  Logfile << "Min Nearest Neighbors: " << minNei << std::endl;
}

else {
  Logfile << "Downsampled: NO" << std::endl;
}

  if (nPick == 1){
    Logfile << "Normal Estimation: MLS" << std::endl;
  }
  else {
    Logfile << "Normal Estimation: Threaded Normal" << std::endl;
    Logfile << "Normal Estimation Search Radius: " << neRad << std::endl;
  }

  Logfile << "MLS Search Radius: " << rad << std::endl;
  Logfile << "UPSCALE: Local Plane Search Radius: " << upRad << std::endl;
  Logfile << "UPSCALE: Local Plane Step Size: " << upStep << std::endl;
  Logfile << "Poisson Depth: " << depth << std::endl;

  Logfile << "TIMING: " <<std::endl;

  Logfile << "\tCrop Points (PassThrough)... " << elapsed_crop << std::endl;
  if (downSample == 1){
    Logfile << "\tDownsample (VoxelGrid)... " << elapsed_vox << std::endl;
  }
  if (downSample == 2) {
    Logfile << "\tDownsample (Radius Outlier Removal)... " << elapsed_rad << std::endl;
  }
  if (nPick == 1) {
    Logfile << "\tMLS Smoothing, UpScale and Normal Estimation (MLS Smooth + Upscale)... " << elapsed_mls << std::endl;
  }
  
  if (nPick == 2) {
    Logfile << "\tMLS Smoothing and Upscale (MLS Smooth + Upscale)... " << elapsed_mlsNoNormal << std::endl;
    Logfile << "\tNormal Estimation... " << elapsed_Normal << std::endl;

  }

  Logfile << "\tPoission Meshing... " << elapsed_Mesh << std::endl;
  Logfile.close();

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointNormal> (cloud_point_normals, "Output");
  viewer->addPolygonMesh(mesh,"Poisson");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Output");
//  viewer->addCoordinateSystem (0.1);
//  viewer->setCameraPosition(0,0,0, centroid[0], centroid[1], centroid[2], 0,1,0);
  viewer->initCameraParameters ();
  viewer->spin();

  // Finish
  return (0);
}
 