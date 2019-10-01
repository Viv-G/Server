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
//Points
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

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
//#include <pcl/visualization/cloud_viewer.h>


using boost::asio::ip::tcp;
using namespace std::chrono_literals;
using std::string;
using std::cout;
using std::endl;

void FilterPoints(double radDown, int minNei, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled){
  // create passthrough filter instance
 pcl::RadiusOutlierRemoval<pcl::PointXYZ> r_rem;
 // set input cloud
 r_rem.setInputCloud (cloud);
 // set cell/voxel size to 0.1 meters in each dimension
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
	mls.process(*mls_points);
}

void estNormals(double neRad = 0.05, pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
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

void meshReconstruct(int depth, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals, pcl::PolygonMesh::Ptr mesh){
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(depth);
		poisson.setInputCloud(cloud_point_normals);
		poisson.reconstruct(*mesh);
		}

void send_(tcp::socket & socket, const string& message){
	const string msg = message + "\n";
	boost::asio::write( socket, boost::asio::buffer(message));
}



void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " <options>\n"
       << "where options are:\n"
       << "  -port p :: set the server port (default: 11111)\n";
}

int
main (int argc, char** argv)
{
// argc cmd line options	
	if (pcl::console::find_argument (argc, argv, "-h") != -1)
  	{
    	usage (argv);
    	return (0);
  	}
	int port = 11111;
	string device_id;

	pcl::console::parse_argument (argc, argv, "-port", port);

// Create Empty Point Cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//	pcl::PointCloud<pcl::PointXYZ> cloud, cloudTemp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp;
	cloudTemp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled;
	cloud_downsampled = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points;
	mls_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	cloud_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals;
	cloud_point_normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();


	pcl::PolygonMesh::Ptr mesh;
	mesh = boost::make_shared<pcl::PolygonMesh>();


// Create new io service
	boost::asio::io_service io_service;
// Listen for new connection
	tcp::endpoint endpoint (tcp::v4 (), static_cast<unsigned short> (port));
	tcp::acceptor acceptor (io_service, endpoint);
// Create Socket
	tcp::socket socket (io_service);
// Wait for Connection and Announce
	std::cout << "Listening on port " << port << "..." << std::endl;
	acceptor.accept (socket);
	std::cout << "Client connected." << std::endl;

// Declare Tracking Variables
	///////TIMING VARIABLES////////      
	double start_time_master = pcl::getTime ();
	double readNumTrack;
	double readPointTrack;
	double start_time_Read = pcl::getTime();
	double start_time, end_time = 0;
	double new_time_Read;
	double elapsed_time;
	double HandlePoints_Time;
	double handleTrack;
	double elapsed_crop, elapsed_vox, elapsed_mls, elapsed_mlsNoNormal, elapsed_Normal, elapsed_Mesh, elapsed_rad = 0;
	int readINT;
	int counter = 0;
	////////POINT READING VARIABLES/////////
	int numPoints = 0;
	float xyz[2] = {};
	float camPos[3] = {};
	std::vector<int> idVec;
	int idTemp = 0;
	int cloudSize_old = 0;
	int cloudSize_new = 0;
	int track = 0;
	int ptStart = 150;
	int ftStart = 1000;

	///////FILTER///////
	int minNei = 40;	// 10 old
	double radDown = 0.04; // 0.3 old

	///////MLS///////
	double rad = 0.05;

	///////Normal Estimation///////
	double neRad = 0.05; // 0.05

	///////MESHING////////////
	int depth = 6;

	// Initial cloudTemp setup
	cloudTemp->height = 1;
	cloudTemp->is_dense = false;

	boost::asio::streambuf buf; // New Streambuf	
	boost::system::error_code err1; // READ NUMPOINTS
	boost::system::error_code err; // READ POINTS
	string data = "";
	string PointString = "";

	boost::asio::read_until(socket, buf, "ENDN\n", err1);
	if (err1) {
  		cout <<"Error in read_NumPoints" << err1.message() << endl;
  		return 0;
	}
	std::istream is(&buf);
	std::getline(is, data);

//	cout << data << endl;
	std::stringstream ssNum(data);
	ssNum >> numPoints;
//	cout << "Number of Points: " << numPoints << endl;

// Scale Point Cloud 
	cloud->width = numPoints;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(numPoints);


// Read Points From Socket
    boost::asio::read_until(socket, buf, "ENDP\n", err);
	if (err){
  		cout << "Connection Closed On StartUp(" << err.message() <<") - Exiting" << endl;
  		return 0;
	}
	std::getline(is, PointString);
	std::stringstream ssPoints(PointString);
	ssPoints >> camPos[0];
	ssPoints >> camPos[1];
	ssPoints >> camPos[2];
	for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			ssPoints >> idTemp;
			idVec.push_back(idTemp);
			ssPoints >> cloud->points[i].x;
			ssPoints >> cloud->points[i].y;
			ssPoints >> cloud->points[i].z;
		}
	Eigen::Vector4f centroid;
  	compute3DCentroid(*cloud, centroid);

/*	if (cloud->points.size() > ptStart)
	{
	///////////////FILTER//////////////
	  	if (cloud->points.size() > ftStart){
	 	start_time = pcl::getTime ();
		FilterPoints(radDown, minNei, cloud, cloud_downsampled);
	 	end_time = pcl::getTime ();
	 	elapsed_rad = end_time - start_time;
	 	std::cout << "Success... Reduced from " << cloud->points.size() << " to " << cloud_downsampled->points.size() << std::endl;
	}
	///////////////////////////////////////

	/////////////MLS SMOOTHING////////////
	 	start_time = pcl::getTime();
	 	MLS(rad, cloud_downsampled, mls_points);
	  	end_time = pcl::getTime ();
	  	elapsed_mlsNoNormal = end_time - start_time;
	  	std::cout << "Smothed Points Now " << mls_points->points.size() <<" - Starting Normal Estimation...\n" << std::endl;
	  
	//////////////////////////////////////


	////////////EST NORMALS//////////////
	  	start_time = pcl::getTime();
		estNormals(neRad, mls_points, cloud_normals);
		pcl::concatenateFields(*mls_points,*cloud_normals, *cloud_point_normals);
		end_time = pcl::getTime ();
		elapsed_Normal = end_time - start_time;
		std::cout << "Calculated Normals - Starting Poisson Meshing...\n";
	////////////////////////////////////

	///////////////MESHING//////////////
		start_time = pcl::getTime();
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(depth);
		poisson.setInputCloud(cloud_point_normals);
		poisson.reconstruct(*mesh);
		end_time = pcl::getTime ();
		elapsed_Mesh = end_time - start_time;
	/////////////////////////////////////
	} */

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer");
	viewer->setSize(3260, 2100);
	//viewer->setFullScreen(true);
	//viewer->createInteractor();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
 // boost::shared_ptr::pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
 /* 	if (cloud->points.size() > ptStart )
	{ 
		viewer->addPointCloud<pcl::PointNormal> (cloud_point_normals, "Output");
		viewer->addPolygonMesh(*mesh,"Poisson"); 

	}
	else 
	{
		viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "Output");
	}
*/
  	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "Output");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"Output");
  	viewer->setCameraPosition(camPos[0],camPos[1],camPos[2], centroid[0], centroid[1], centroid[2], 0,1,0);
//  viewer->setCameraFieldOfView(0.523599);
//  viewer->setCameraClipDistances(0.00522511, 50);
  	viewer->initCameraParameters ();
  	viewer->spinOnce(1);


	// ENTER MAIN LOOP
    while (!viewer->wasStopped ())
    {
      	numPoints = 0;
      	data = "";
      	PointString = "";
      	track++;
    	/////////////READ NUMBER OF POINTS TO BE READ////////////////
      	start_time_Read = pcl::getTime();
		boost::asio::read_until( socket, buf, "ENDN\n", err1);
		if (err1) {
	  		cout <<"Error Reading Number of Points: " << err1.message() << endl;
	  		cout <<"Saving Mesh + Point Cloud And Exiting... " << endl;
	  		pcl::io::savePCDFile ("Points.pcd", *cloud);
	  		pcl::io::saveVTKFile ("mesh.vtk", *mesh);
	  		return 0;
		}
		std::getline(is, data);
		std::stringstream ssNum(data);
		ssNum >> numPoints;
		new_time_Read = pcl::getTime ();
        elapsed_time = new_time_Read - start_time_Read;
        readNumTrack += elapsed_time;
        elapsed_time = 0;

		/////////////// READ NEW POINTS FROM SOCKET //////////////////
        start_time_Read = pcl::getTime();
 		boost::asio::read_until(socket, buf, "ENDP\n", err);
		if (err){
	  		cout <<"Error Reading New Points: " << err1.message() << endl;
	  		cout <<"Saving Mesh + Point Cloud And Exiting... " << endl;
	  		pcl::io::savePCDFile ("Points.pcd", *cloud);
	  		pcl::io::saveVTKFile ("mesh.vtk", *mesh);
  		return 0;
		}

		std::getline(is, PointString);
		std::stringstream ssPoints(PointString);
		new_time_Read = pcl::getTime ();
        elapsed_time = new_time_Read - start_time_Read;
        readPointTrack += elapsed_time;
        readINT++;
        elapsed_time = 0;
		cloudSize_old = cloud->points.size();

		////////////////////HANDLE POINTS///////////////////////////
		int rep = 0;
		int tFlag = 0;
		double start_time_Handle = pcl::getTime();
		ssPoints >> camPos[0];
		ssPoints >> camPos[1];
		ssPoints >> camPos[2];
		for (size_t i = 0; i < numPoints; ++i)
			{
				ssPoints >> idTemp;
				for(size_t j=0; j < cloudSize_old; ++j)
				{
					if (idTemp == idVec.at(j))
					{
						cloud->points[j].x = 0;
						cloud->points[j].y= 0;
						cloud->points[j].z = 0;
						ssPoints >> cloud->points[j].x;
						ssPoints >> cloud->points[j].y;
						ssPoints >> cloud->points[j].z;
						rep++;
						tFlag = 1;
					}
				}
				if (tFlag != 1)
				{
					idVec.push_back(idTemp);
					pcl::PointXYZ point;
					ssPoints >> point.x;
					ssPoints >> point.y;
					ssPoints >> point.z;
					cloud->points.push_back(point);
					tFlag = 0;
				}
			}
		cloud->width = (uint32_t) cloud->points.size();
		cloudSize_new = cloud->points.size();
		rep = 0;
		Eigen::Vector4f centroid;
		compute3DCentroid(*cloud, centroid);

		double new_time_Handle = pcl::getTime();
		elapsed_time = new_time_Handle -start_time_Handle;
		handleTrack += elapsed_time; 
		elapsed_time = 0;



		if (cloud->points.size() > ptStart)
		{
			///////////////DOWNSAMPLE//////////////
		  	if (cloud->points.size() > ftStart)
		  	{
	 			start_time = pcl::getTime ();
	 			/// CLEAR POINT CLOUDS /////
	 			cloud_downsampled->width = cloud_downsampled->height = 0;
	 			cloud_downsampled->clear();

				downSample(radDown, minNei, cloud, cloud_downsampled);
	 			end_time = pcl::getTime ();
	 			elapsed_rad = end_time - start_time;
	 		}
			///////////////////////////////////////

			/////////////MLS SMOOTHING////////////
	 		start_time = pcl::getTime();
	 		//// CLEAR POINT CLOUDS /////
	 	 	mls_points->width = mls_points->height = 0;
	 		mls_points->clear();
		 	if (cloud->points.size() > ftStart)
		 	{
		 		MLS(rad, cloud_downsampled, mls_points);
		 	}
		 	else
		 	{
		 		MLS(rad, cloud, mls_points);
		 	}
		  	end_time = pcl::getTime ();
		  	elapsed_mlsNoNormal = end_time - start_time;
			//////////////////////////////////////


			////////////EST NORMALS//////////////
	  		start_time = pcl::getTime();
	  		//// CLEAR POINT CLOUDS /////
	  	 	cloud_normals->width = cloud_normals->height = 0;
	 		cloud_normals->clear();
	 		cloud_point_normals->width = cloud_point_normals->height = 0;
	 		cloud_point_normals->clear();
			estNormals(neRad, mls_points, cloud_normals);
			pcl::concatenateFields(*mls_points,*cloud_normals, *cloud_point_normals);
			end_time = pcl::getTime ();
			elapsed_Normal = end_time - start_time;
			////////////////////////////////////
		

			///////////////MESHING//////////////
			if (cloudSize_new != cloudSize_old) // ONLY RE-MESH IF CLOUD HAS BEEN UPDATED
			{
				start_time = pcl::getTime();
				/// CLEAR POINT CLOUDS /////
				meshReconstruct(depth, cloud_point_normals, mesh);
				end_time = pcl::getTime ();
				elapsed_Mesh = end_time - start_time;
			}
			/////////////////////////////////////
		}

		/////////////VIEWER////////////////////////////
    	viewer->removeAllPointClouds();
    	if (cloud->points.size() > ptStart )
		{ 
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud_point_normals, 0, 255, 0);
    		viewer->addPolygonMesh(*mesh,"Poisson");
    		viewer->addPointCloud<pcl::PointNormal> (cloud_point_normals, single_color, "Output");
  		}
  		else 
  		{	
  			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 0, 255, 0);
  			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "Output");
  		}
     	viewer->setCameraPosition(camPos[0],camPos[1],camPos[2], centroid[0], centroid[1], centroid[2], 0,1,0);
     	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"Output");
	 	viewer->spinOnce();

	 	counter++;
        double new_time = pcl::getTime ();
        elapsed_time = new_time - start_time_master;
        if (elapsed_time > 1.0)
        {       
        // Average Read Times
          	double readNumAv = readNumTrack / readINT;
          	double readPointAv = readPointTrack / readINT;
          	double handleAv = handleTrack / readINT;
          	cout << "\tRead Num Points Average Time: " << readNumAv << endl;
          	cout << "\tRead Points In Average Time: " << readPointAv << endl;
          	cout << "\tHandle PointCloud Average Time: " << handleAv << endl;

          	double frames_per_second = counter / elapsed_time;
          	start_time_master = new_time;
          	counter = 0;
          	cout << "\tfps: " << frames_per_second << endl;
          	cout << "\tCloud Size: " << cloudSize_new << endl;
        }
        ////////////////////////////////////////////////////

	}
    pcl::io::savePCDFile ("Points.pcd", *cloud);
    pcl::io::saveVTKFile ("mesh.vtk", *mesh);
    return (0);
}