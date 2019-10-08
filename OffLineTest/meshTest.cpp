//System
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <iostream>
#include <sstream>
#include <boost/asio.hpp>
#include <mutex>
#include <thread>
#include <vector> 
#include <atomic>
#include <boost/filesystem.hpp>
//Points
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//Filters
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

//Surfaces
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>
//IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
//Visualize
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h> 


using boost::asio::ip::tcp;
using namespace std::chrono_literals;
using std::string;
using std::cout;
using std::endl;

std::atomic<bool> update(false);
boost::mutex updateModelMutex;

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " <options>\n"
    	<< "where options are:\n"
    	<< "  -port p :: \tset the server port (default: 11111)\n"
    	<< " -ft ftStart :: \tSet the number of points to start filtering at\n"
		<< " -rD radDown :: \tSet the radius for Radius Outlier Removal (Default 0.05)\n"
		<< " -mNei minNei :: \tSet the minimum neighbours for Radius Outlier Removal (Default 40) \n"
		<<" -rad rad :: \tSet the search radius for MLS Smoothing (Default 0.04)\n"
		<<" -nr, neRad :: \tSet the search radius for Normal Estimation (Default 0.04)\n"
		<<" -std, stdDev :: \tSet the standard deviation multiplier for the statistical outlier removal\n"
		<<" -mK, mK :: \tSet the number of nearest neighbours for statistical outlier removal\n" 
		<<" -d, depth :: \tSet the depth for poisson meshing\n"<< endl;
}

void FilterPoints(double radDown, int minNei, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled){
  // create passthrough filter instance
 pcl::RadiusOutlierRemoval<pcl::PointXYZ> r_rem;
 // set input cloud
 r_rem.setInputCloud (cloud);
 r_rem.setRadiusSearch(radDown);
  // do filtering
 r_rem.setMinNeighborsInRadius(minNei);
 r_rem.filter (*cloud_downsampled);
}

void MLS(double rad, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled, pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points){
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	mls.setInputCloud (cloud_downsampled);
	mls.setSearchRadius(rad);
	mls.setPolynomialOrder(0);
	mls.process(*mls_points);
}

void estNormals(double neRad, pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads (2);
	ne.setInputCloud(mls_points);
	ne.setRadiusSearch(neRad);
	Eigen::Vector4f centroid;
	compute3DCentroid(*mls_points, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
	ne.compute(*cloud_normals);

	for (size_t i = 0; i < cloud_normals->size(); ++i)
	  {
	    cloud_normals->points[i].normal_x *= -1;
	    cloud_normals->points[i].normal_y *= -1;
	    cloud_normals->points[i].normal_z *= -1;
	  }

}

void statOutlier (int mK, double stdDev, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr preMesh){
	// S t a t i s t i c a l O u t l i e r Removal
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_point_normals);
	sor.setMeanK(mK);
	sor.setStddevMulThresh(stdDev);
	sor.filter(*preMesh);
}

//void meshReconstruct(int depth, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals, pcl::PolygonMesh::Ptr mesh){
void meshReconstruct(int depth, pcl::PointCloud<pcl::PointNormal>::Ptr preMesh, pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr updateMesh, string LogName){
		double start_time = pcl::getTime();
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(depth);
		poisson.setInputCloud(preMesh);
		poisson.reconstruct(*updateMesh);
		boost::mutex::scoped_lock updateLock(updateModelMutex);
		*mesh = *updateMesh;
		update = true;
		double end_time = pcl::getTime ();
		double meshTime = end_time - start_time;
//		cout << "MESH-\tMesh Time: " << meshTime << endl;
//		cout << "MESH-\tPointCloud Size: " << preMesh->points.size() << endl;
		ofstream Logfile;
		Logfile.open(LogName, ios::out | ios::app );
		Logfile << "\n/////////MESHING/////////" << endl;
		Logfile << "\tMESH-\tMesh Time: " << meshTime << endl;
		Logfile << "\tMESH-\tPointCloud Size: " << preMesh->points.size() << endl;
		Logfile.close();
		}


int
main (int argc, char** argv)
{

	if (pcl::console::find_argument (argc, argv, "-h") != -1)
  	{
    	usage (argv);
    	return (0);
  	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled2 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr preMesh (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh());
	pcl::PolygonMesh::Ptr updateMesh (new pcl::PolygonMesh());

	pcl::io::loadPCDFile ("points.pcd", *cloud);

	boost::thread meshThread;

	// Declare Tracking Variables
	///////TIMING VARIABLES////////

	double readNumTrack;
	double readPointTrack;

	double start_time, end_time = 0;
	double new_time_Read;
	double elapsed_time;
	double HandlePoints_Time;
	double handleTrack;
	double elapsed_crop, elapsed_vox, elapsed_mls, elapsed_mlsNoNormal, elapsed_Normal, elapsed_Mesh, elapsed_rad = 0;
	int readINT, meshINT = 0;
	int counter = 0;
	////////POINT READING VARIABLES/////////
	int numPoints = 0;
	float xyz[2] = {};
	float camPos[3] = {0.308285, 0.409804, 2.70774};
	std::vector<int> idVec;
	int idTemp = 0;
	int cloudSize_old = 0;
	int cloudSize_new = 0;
	int track = 0;
	int ptStart = 150;
	int ftStart = 250;
	double totalTime = 0;
	double end_time_total = 0;

	pcl::console::parse_argument (argc, argv, "-ft", ftStart);


	double start_time_total = pcl::getTime ();
	double start_time_master = pcl::getTime ();
	double start_time_Read = pcl::getTime();

	//////CRUDE FILTER///////
	double mU = 1.1;
	pcl::console::parse_argument (argc, argv, "-mU", mU);


	///////FILTER///////
	int minNei = 40;	// 10 old
	double radDown = 0.04; // 0.3 old
		pcl::console::parse_argument (argc, argv, "-rD", radDown);
		pcl::console::parse_argument (argc, argv, "-mNei", minNei);

	///////MLS///////
	double rad = 0.05;
		pcl::console::parse_argument (argc, argv, "-rad", rad);

	///////Normal Estimation///////
	double neRad = 0.05; // 0.05
	pcl::console::parse_argument (argc, argv, "-nr", neRad);

	///////STATISTICAL OUTLIER REMOVAL//////
	int mK = 500; //Set the number of nearest neighbors to use for mean distance estimation.
	double stdDev = 1.0; //Set the standard deviation multiplier for the distance threshold calculation.
	pcl::console::parse_argument (argc, argv, "-std", stdDev);
	pcl::console::parse_argument (argc, argv, "-mK", mK);


	///////MESHING////////////
	int depth = 6;
	pcl::console::parse_argument (argc, argv, "-d", depth);

	time_t rawtime;
  	struct tm * timeinfo;
  	char timeChar [15];

  	time (&rawtime);
  	timeinfo = localtime(&rawtime);

  	strftime(timeChar,80,"%I_%M_%S",timeinfo);
  	std::string timeString(timeChar);

  	std::string folName = "./"+timeString;
  	std::string LogName = folName+"//Log_"+timeString+".txt";
  	std::string pointString = folName + "//PointsFilt.pcd";
  	std::string meshString = folName + "//mesh.vtk";
  	std::string meshString2 = folName + "//model.stl";
  	std::string ptString = folName + "//points.pcd";



  	boost::filesystem::create_directories(folName);

  	/*
  	puts (buffer);

	// declaring argument of time() 
    time_t my_time = time(NULL); 
  
    // ctime() used to give the present time 

	std::stringstream sName;
	sName << "Log_" << ctime(&my_time) << ".txt";
	string LogName = sName.str(); */

	ofstream Logfile;
	Logfile.open(LogName, ios::out | ios::trunc);
	Logfile << "/////////RADIUS OUTLIER REMOVAL/////////" << endl;
	Logfile << "\tRadius: " << radDown << endl;
  	Logfile << "\tMin Nearest Neighbors: " << minNei << endl;
 	Logfile << "\n/////////STATISTICAL OUTLIER REMOVAL/////////" << endl;
	Logfile << "\tStandard Deviation Multiplier: " << stdDev << endl;
  	Logfile << "\tNearest Neighbors: " << mK << endl; 	
  	Logfile << "\n/////////SMOOTHING & NORMAL ESTIMATION/////////" << endl;
  	Logfile << "\tMLS Search Radius: " << rad << endl;
  	Logfile << "\tNormal Estimation Search Radius: " << neRad << endl;
  	Logfile << "\n/////////POISSON MESHING/////////" << endl;
    Logfile << "\tPoisson Depth: " << depth << endl;
    Logfile << "\n\n/////////START RUN/////////" << endl;
    Logfile.close();

    cloudSize_old = cloud->points.size();

/// FIND CENTROID ////
    Eigen::Vector4f cR;
	compute3DCentroid(*cloud, cR);

	cout << "Centroid = " << cR[0] << " , "<< cR[1] << " , " << cR[2] << endl;

	std::vector<float> dis;
	float disTemp = 0;
	float avgProg = 0;
	float disSqrt = 0;

	for (size_t i = 0; i < cloudSize_old; ++i){
		disTemp = abs((cloud->points[i].x -cR[0])+(cloud->points[i].y -cR[1])+(cloud->points[i].z -cR[2]));
		disSqrt = sqrt(disTemp);
		dis.push_back(disSqrt);
		avgProg += disSqrt;
	}

	float avgDistance = avgProg / dis.size();
	cout << avgDistance << endl;
	for (size_t i = 0; i < cloudSize_old; ++i){
		disTemp = abs((cloud->points[i].x -cR[0])+(cloud->points[i].y -cR[1])+(cloud->points[i].z -cR[2]));
		disSqrt = sqrt(disTemp);

		if (disSqrt < mU*avgDistance){
			cloudTemp->points.push_back(cloud->points[i]);
		}
	}

	cout << "Cloud Temp Size = " << cloudTemp->points.size() << endl;


	/*double dis = 0;
	for (size_t i = 0; i < cloudSize_old; ++i){
		dis = sqrt((cloud->points[i].x -cr[0])+(cloud->points[i].y -cr[1])+(cloud->points[i].z -cr[2]));
		*/


/*			///////////////DOWNSAMPLE//////////////
	 		start_time = pcl::getTime ();
 			/// CLEAR POINT CLOUDS /////
			FilterPoints(radDown, minNei, cloud, cloud_downsampled);
 			end_time = pcl::getTime ();
 			elapsed_rad = end_time - start_time;	 		
			///////////////////////////////////////  */

			//////////// REMOVE FINAL OUTLIERS ////////////
			start_time = pcl::getTime();
			statOutlier(mK,stdDev, cloud, cloud_downsampled);
			end_time = pcl::getTime ();
			double elapsed_Stat = end_time - start_time;

			///////////////FILTER//////////////
	 		start_time = pcl::getTime ();
 			/// CLEAR POINT CLOUDS /////
			FilterPoints(radDown, minNei, cloud_downsampled, cloud_downsampled2);
 			end_time = pcl::getTime ();
 			elapsed_rad = end_time - start_time;	 		
			/////////////////////////////////////// 


			/////////////MLS SMOOTHING////////////
	 		start_time = pcl::getTime();
	 		//// CLEAR POINT CLOUDS /////
		 	
		 	MLS(rad, cloud_downsampled2, mls_points);
		 	
		
		  	end_time = pcl::getTime ();
		  	elapsed_mlsNoNormal = end_time - start_time;
			//////////////////////////////////////


			////////////EST NORMALS//////////////
	  		start_time = pcl::getTime();
	  		//// CLEAR POINT CLOUDS /////

			estNormals(neRad, mls_points, cloud_normals);
//			pcl::concatenateFields(*mls_points,*cloud_normals, *cloud_point_normals);
			pcl::concatenateFields(*mls_points,*cloud_normals, *preMesh);
			end_time = pcl::getTime ();
			elapsed_Normal = end_time - start_time;
			////////////////////////////////////


	/*		//////////// REMOVE FINAL OUTLIERS ////////////
			start_time = pcl::getTime();
			statOutlier(mK,stdDev, cloud_point_normals, preMesh);
			end_time = pcl::getTime ();
			double elapsed_Stat = end_time - start_time; */
		


				start_time = pcl::getTime();
				/// CLEAR POINT CLOUDS /////
			//	meshReconstruct(depth, cloud_point_normals, mesh);
			//	boost::thread meshThread(meshReconstruct, depth, preMesh, mesh, updateMesh);
			//	meshReconstruct(depth, preMesh, mesh, updateMesh);
				meshThread = boost::thread(meshReconstruct, depth, preMesh, mesh, updateMesh, LogName);
				end_time = pcl::getTime ();
				elapsed_Mesh = end_time - start_time;
				meshINT++;

			cloudSize_new = preMesh->points.size();

	if(meshThread.joinable())
		{
		meshThread.join();
		}
	int m(0);
	int pN(0);
	int p(0);
	int pT(0);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(preMesh, 0, 255, 0);
//	viewer->setCameraPosition(0.156407, 0.590266, -1.05477, cR[0], cR[1], cR[2], 0,1,0);
	viewer->createViewPort (0.0, 0.0, 0.25, 1.0, m);
	viewer->createViewPort (0.26, 0.0, 0.5, 1.0, pN);
	viewer->createViewPort (0.51, 0.0, 0.75, 1.0, pT);
	viewer->createViewPort (0.76, 0.0, 1.0, 1.0, p);
	viewer->setCameraPosition(1.80495, 0.684705, -1.34932, cR[0], cR[1], cR[2], 0,1,0);
//	viewer->setCameraPosition(1.80495, 0.684705, -1.34932, cR[0], cR[1], cR[2], 0,1,0,pN);
//	viewer->setCameraPosition(1.80495, 0.684705, -1.34932, cR[0], cR[1], cR[2], 0,1,0,pN);
	viewer->addPolygonMesh(*mesh,"Poisson",m);
	viewer->addPointCloud<pcl::PointNormal> (preMesh, single_color, "Output",pN);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "pts",p);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_downsampled, "ptsTemp",pT);
//	viewer->addPointCloudNormals<pcl::PointNormal, pcl::Normal> (preMesh, normals, 10, 0.05, "normals");
  	viewer->setBackgroundColor (0, 0, 0);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,"Output");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,"pts");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,"ptsTemp");
//  viewer->setCameraFieldOfView(0.523599);
//  viewer->setCameraClipDistances(0.00522511, 50);
//  	viewer->setCameraPosition(0.156407, 0.590266, -1.05477, cR[0], cR[1], cR[2], 0,1,0);
  	viewer->initCameraParameters ();
  	double new_time = pcl::getTime ();
    elapsed_time = new_time - start_time_master;

  	viewer->spin();

          	Logfile.open(LogName, ios::out | ios::app);
          	Logfile << "\n/////////TIMINGS/////////" << endl;
          	Logfile << "\tRadius Outlier Removal: " << elapsed_rad << " Seconds" << endl;
          	Logfile << "\tMLS: " << elapsed_mlsNoNormal << " Seconds" << endl;
          	Logfile << "\tNormals: " << elapsed_Normal << " Seconds" << endl;
          	Logfile << "\tStatistical Outlier: " << elapsed_Stat << " Seconds" << endl;
          	int remPts = cloudSize_old - cloudSize_new;
          	int remPtsStat = cloudSize_old - cloud_downsampled->points.size();
          	int remPtsRad = cloud_downsampled->points.size() - cloud_downsampled2->points.size();


//          	Logfile << "\tMesh Average Time: " << handleAv << endl;
          	Logfile.close();
        
        ////////////////////////////////////////////////////
    //    meshThread.join();

//    pcl::io::savePCDFile ("Points.pcd", *cloud);
    pcl::io::savePCDFile (pointString, *preMesh);
    pcl::io::savePCDFile (ptString, *cloud);
    pcl::io::saveVTKFile (meshString, *mesh);
    pcl::io::savePolygonFileSTL(meshString2, *mesh, true);
    //WRITE TO FILE
    Logfile.open(LogName, ios::out | ios::app );
    Logfile << "\n\n/////////TOTALS (END OF RUN)/////////" << endl;
    Logfile << "\tInput Cloud Size: " << cloudSize_old << endl;
    Logfile << "\tFinal Cloud Size: " << cloudSize_new << endl;
	Logfile << "\tTotal Run Time: " << elapsed_time << endl;
	Logfile << "\tRemoved Overall: " << remPts << endl;
	Logfile << "\tRemoved in Statistical Outlier Removal: " << remPtsStat << endl;
	Logfile << "\tRemoved in Radius Outlier Removal: " << remPtsRad << endl;

  	Logfile.close();

    return (0);
}