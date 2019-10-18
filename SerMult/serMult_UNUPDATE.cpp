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
#include <pcl/filters/voxel_grid.h>

//Surfaces
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

// Plane fitting
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

//IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

//Visualize
#include <pcl/visualization/pcl_visualizer.h>


using boost::asio::ip::tcp;
using namespace std::chrono_literals;
using std::string;
using std::cout;
using std::endl;

std::atomic<bool> update(false);
std::atomic<bool> waitFlag(false);
std::atomic<bool> exitFlag(false);
std::atomic<bool> newPoints(false);

boost::mutex updateModelMutex;
boost::mutex readPointsMutex;

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
	mls.setPolynomialOrder(1);
	mls.process(*mls_points);
}

void projPts(pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected){
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_downsampled);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
}

void voxFilter(float vLeaf, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_filtered){
  // create filter instance
 	pcl::VoxelGrid<pcl::PointXYZ> vox;
 // set input cloud
 	vox.setInputCloud (cloud_projected);
 // set cell/voxel size to vLeaf meters in each dimension
 	vox.setLeafSize (vLeaf, vLeaf, vLeaf);
  // do filtering
 	vox.filter (*cloud_base_filtered);
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

void vtkSmooth(int iter, pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr smoothedMesh){
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(mesh);
    vtk.setNumIter(20);
    vtk.setRelaxationFactor(0.1);      
	vtk.setFeatureEdgeSmoothing(false);
	vtk.setBoundarySmoothing(true);
    vtk.process(*smoothedMesh);
}

void vtkDec(float rFac, pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr decMesh){
    pcl::MeshQuadricDecimationVTK dec;
    dec.setInputMesh(mesh);
    dec.setTargetReductionFactor(rFac);
    dec.process(*decMesh);
}

//void meshReconstruct(int depth, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals, pcl::PolygonMesh::Ptr mesh){
void meshReconstruct(float rFac, int iter, string LogName, int depth, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals, pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr updateMesh, pcl::PolygonMesh::Ptr smoothedMesh){
	double start_time = pcl::getTime();
	pcl::PolygonMesh::Ptr decMesh (new pcl::PolygonMesh());
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(depth);
	boost::mutex::scoped_lock updateLock(updateModelMutex);
	poisson.setInputCloud(cloud_point_normals);
	poisson.reconstruct(*updateMesh);
	//*mesh = *updateMesh;
	cout << "FINISHED MESH" << endl;

	cout << "- STARTED SMOOTHING" << endl;
	vtkDec(rFac, updateMesh, decMesh);
	vtkSmooth(iter, decMesh, smoothedMesh);
	cout << "- FINISHED SMOOTHING" << endl;
	*mesh = *updateMesh;
	update = true;
	double end_time = pcl::getTime ();
	double meshTime = end_time - start_time;
	ofstream Logfile;
	Logfile.open(LogName, ios::out | ios::app );
	Logfile << "\n/////////MESHING/////////" << endl;
	Logfile << "\tMESH-\tMesh Time: " << meshTime << endl;
	Logfile << "\tMESH-\tPointCloud Size: " << cloud_point_normals->points.size() << endl;
	Logfile.close();
	updateLock.unlock();
}


bool saveFiles(string timeString, pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr decMesh, pcl::PolygonMesh::Ptr smoothedMesh, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	string folName = "./"+timeString;
	std::string ptString = folName + "//points.pcd";
	std::string pointString = folName + "//PointsFilt.pcd";
  	std::string meshString = folName + "//mesh.vtk";
  	std::string meshString2 = folName + "//model.stl";
  	std::string meshString3 = folName + "//decMesh.vtk";
  	std::string meshString4 = folName + "//smoothedMesh.vtk";
  	std::string meshString5 = folName + "//smoothedMesh.stl";

	pcl::io::savePCDFile (pointString, *cloud_point_normals);
	pcl::io::savePCDFile (ptString, *cloud);
	pcl::io::saveVTKFile (meshString, *mesh);
//	pcl::io::saveVTKFile (meshString3, *decMesh);
	pcl::io::saveVTKFile (meshString4, *smoothedMesh);
	pcl::io::savePolygonFileSTL(meshString2, *mesh, true);
	pcl::io::savePolygonFileSTL(meshString5, *smoothedMesh, true);

	return (true);
}


void send_(tcp::socket & socket, const string& message){
	const string msg = message + "\n";
	boost::asio::write( socket, boost::asio::buffer(message));
}


void readPoints(int port, string LogName, boost::shared_ptr<string> pointString, boost::shared_ptr<int> numPoints){
	int track = 0;
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

	boost::asio::streambuf buf; // New Streambuf	
	boost::system::error_code err1; // READ NUMPOINTS
	boost::system::error_code err; // READ POINTS
	string data = "";
	string pointStringTemp = "";
	*pointString = "";	
	*numPoints = 0;

	boost::asio::read_until(socket, buf, "ENDN\n", err1);
	if (err1) {
  		cout <<"Error in read_NumPoints" << err1.message() << endl;
  		return;
	}
	std::istream is(&buf);
	std::getline(is, data);

	boost::mutex::scoped_lock readLock(readPointsMutex);
	std::stringstream ssNum(data);
	ssNum >> *numPoints;

//	cout << "NumPoints after assignment = " << *numPoints << endl;


// Read Points From Socket
    boost::asio::read_until(socket, buf, "ENDP\n", err);
	if (err){
  		cout << "Connection Closed On StartUp(" << err.message() <<") - Exiting" << endl;
  		return;
	}
	std::getline(is, *pointString);

//	cout << *pointString << endl;

	readLock.unlock();

	waitFlag = false;
	double start_time = pcl::getTime();
  	while(!exitFlag){
    	//numPoints = 0;
  		data = "";
  		pointStringTemp = "";
  		track++;

		/////////////READ NUMBER OF POINTS TO BE READ////////////////
	//  	start_time_Read = pcl::getTime();
		boost::asio::read_until( socket, buf, "ENDN\n", err1);
		if (err1) {
	  		cout <<"Error Reading Number of Points: " << err1.message() << endl;
	  		cout <<"Saving Mesh + Point Cloud And Exiting... " << endl;
	  		exitFlag = true;

	  	/*	if(saveFiles(timeString, mesh, decMesh, smoothedMesh, cloud_point_normals, cloud)){
	  			cout << "Succesfully Saved 7 Files to " << folName << "." << endl;
	  		} */
	/*  		end_time_total = pcl::getTime();
			totalTime = end_time_total - start_time_total;

	  		Logfile.open(LogName, ios::out | ios::app);
		    Logfile << "/////////TOTALS (END OF RUN)/////////" << endl;
			Logfile << "\tTotal Run Time: " << totalTime << endl;
		  	Logfile << "\tTotal Frames Recieved: " << track << endl;
		  	Logfile << "\tCloud Size: " << cloudSize_new << endl;
		  	Logfile.close();

		  	//////////WAIT FOR THREADS TO FINISH/////////
	  		if(meshThread.joinable())
			{
				meshThread.join();
			} */
	  		return;
		} 
		readLock.lock();
		std::getline(is, data);
		std::stringstream ssNum(data);
		ssNum >> *numPoints;
//		new_time_Read = pcl::getTime ();
//	    elapsed_time = new_time_Read - start_time_Read;
//	    readNumTrack += elapsed_time;
//	    elapsed_time = 0;

		/////////////// READ NEW POINTS FROM SOCKET //////////////////
//	    start_time_Read = pcl::getTime();
		boost::asio::read_until(socket, buf, "ENDP\n", err);
		if (err){
	  		cout <<"Error Reading New Points: " << err1.message() << endl;
	  		cout <<"Saving Mesh + Point Cloud And Exiting... " << endl;
	  		exitFlag = true;
	  	/*	if(saveFiles(timeString, mesh, decMesh, smoothedMesh, cloud_point_normals, cloud)){
	  			cout << "Succesfully Saved 7 Files to " << folName << "." << endl;
	  		}
	  		end_time_total = pcl::getTime();
			totalTime = end_time_total - start_time_total;

	  		Logfile.open(LogName, ios::out | ios::app );
		    Logfile << "/////////TOTALS (END OF RUN)/////////" << endl;
			Logfile << "\tTotal Run Time: " << totalTime << endl;
		  	Logfile << "\tTotal Frames Recieved: " << track << endl;
		  	Logfile << "\tCloud Size: " << cloudSize_new << endl;
		  	Logfile.close();
	  		if(meshThread.joinable())
			{
				meshThread.join();
			} */
			return;
		}

		std::getline(is, pointStringTemp); ///////RETURN POINTSTRING
		*pointString = pointStringTemp;
		newPoints = true;
		readLock.unlock();

		double new_time = pcl::getTime ();
        double elapsed_time = new_time - start_time;
        if (elapsed_time > 5.0)
        {       
        	double fps = track / elapsed_time;
        	start_time = new_time;
        	track = 0;
          	cout << "\tREAD POINTS FPS: " << fps << endl;
		}
	}
}



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

	/////// Create Point Clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled2 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr preMesh (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh());
	pcl::PolygonMesh::Ptr updateMesh (new pcl::PolygonMesh());
	pcl::PolygonMesh::Ptr smoothedMesh (new pcl::PolygonMesh());
	pcl::PolygonMesh::Ptr decMesh (new pcl::PolygonMesh());

	// Initial cloud setup
	cloud->height = 1;
	cloud->is_dense = false;


	boost::shared_ptr<int> numPoints (new int);
	*numPoints = 0;

	boost::shared_ptr<string> pointString (new string);

	/// Declare Thread for Meshing
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
	double elapsed_crop, elapsed_vox, elapsed_mls, elapsed_mlsNoNormal, elapsed_Normal, elapsed_Mesh, elapsed_rad, elapsed_Stat = 0;
	int readINT, meshINT = 0;
	int counter = 0;
	////////POINT READING VARIABLES/////////
	float xyz[2] = {};
	float camPos[3] = {};
	std::vector<int> idVec;
	int idTemp = 0;
	int cloudSize_old = 0;
	int cloudSize_new = 0;
	int track = 0;
	
	double totalTime = 0;
	double end_time_total = 0;

	int ptStart = 150;
	int ftStart = 1000;
	pcl::console::parse_argument (argc, argv, "-ft", ftStart);


	///////FILTER///////
	int minNei = 10;	// 10 old
	double radDown = 0.05; // 0.3 old
	pcl::console::parse_argument (argc, argv, "-rD", radDown);
	pcl::console::parse_argument (argc, argv, "-mNei", minNei);

	/////////////VOX FILTER///////////
	float vLeaf = 0.01;
	pcl::console::parse_argument (argc, argv, "-vLeaf", vLeaf);

	///////MLS///////
	double rad = 0.04;
	pcl::console::parse_argument (argc, argv, "-rad", rad);

	///////Normal Estimation///////
	double neRad = 0.08; // 0.05
	pcl::console::parse_argument (argc, argv, "-nr", neRad);

	///////STATISTICAL OUTLIER REMOVAL//////
	int mK = 200; //Set the number of nearest neighbors to use for mean distance estimation.
	double stdDev = 1.0; //Set the standard deviation multiplier for the distance threshold calculation.
	pcl::console::parse_argument (argc, argv, "-std", stdDev);
	pcl::console::parse_argument (argc, argv, "-mK", mK);

	///////MESHING////////////
	int depth = 6;
	pcl::console::parse_argument (argc, argv, "-d", depth);

	//////////MESH SMOOTHING/////////
	int iter = 20;
	float rFac = 0.6;
	pcl::console::parse_argument (argc, argv, "-it", iter);
	pcl::console::parse_argument (argc, argv, "-rFac", rFac);


	////////PLANE EQUATION/////////
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	coefficients->values.resize (4);
  	coefficients->values[0] = coefficients->values[2] = 0;
  	coefficients->values[1] = 1.0;
  	coefficients->values[3] = 0;

  	/////////// THREADS AVAILABLE /////////////
  	unsigned int nthreads = boost::thread::hardware_concurrency();
  	cout << "Number of Threads Available: " << nthreads << endl;

  	string pointStringRun = "";


	/////////////FILENAME SETUP/////////////////
	time_t rawtime;
  	struct tm * timeinfo;
  	char timeChar [15];

  	time (&rawtime);
  	timeinfo = localtime(&rawtime);

  	strftime(timeChar,80,"%I_%M_%S",timeinfo);
  	std::string timeString(timeChar);

  	std::string folName = "./"+timeString;
  	std::string LogName = folName+"//Log_"+timeString+".txt";
  	boost::filesystem::create_directories(folName);


	ofstream Logfile;
	Logfile.open(LogName, ios::out | ios::trunc);
	Logfile << "/////////RADIUS OUTLIER REMOVAL/////////" << endl;
	Logfile << "\tRadius: " << radDown << endl;
  	Logfile << "\tMin Nearest Neighbors: " << minNei << endl;
 	Logfile << "\n/////////STATISTICAL OUTLIER REMOVAL/////////" << endl;
	Logfile << "\tStandard Deviation Multiplier: " << stdDev << endl;
  	Logfile << "\tNearest Neighbors: " << mK << endl;
  	Logfile << "/////////VOX FILTER/////////" << endl;
	Logfile << "\tLeaf Size: " << vLeaf << endl;
  	Logfile << "\n/////////SMOOTHING & NORMAL ESTIMATION/////////" << endl;
  	Logfile << "\tMLS Search Radius: " << rad << endl;
  	Logfile << "\tNormal Estimation Search Radius: " << neRad << endl;
  	Logfile << "\n/////////POISSON MESHING/////////" << endl;
    Logfile << "\tPoisson Depth: " << depth << endl;
    Logfile << "\n\n/////////START RUN/////////" << endl;
    Logfile.close();

    boost::thread readThread(readPoints, port, LogName, pointString, numPoints );

    waitFlag = true;

    while(waitFlag){
		std::this_thread::sleep_for(1ms);
	}
	int numPointsRun = *numPoints;
//	cout << "Num Points in MAIN: " << numPointsRun << endl;;

	pointStringRun = *pointString;
//	cout << "Points in MAIN: " << pointStringRun << endl;

	std::stringstream ssPoints(pointStringRun);

	ssPoints >> coefficients->values[3];
	ssPoints >> camPos[0];
	ssPoints >> camPos[1];
	ssPoints >> camPos[2];
	for (size_t i = 0; i < numPointsRun; ++i)
		{
			ssPoints >> idTemp;
			idVec.push_back(idTemp);
			pcl::PointXYZ point;
			ssPoints >> point.x;
			ssPoints >> point.y;
			ssPoints >> point.z;
			cloud->points.push_back(point);		
		}
	cloud->points.resize(numPointsRun);
	Eigen::Vector4f centroid;
  	compute3DCentroid(*cloud, centroid);


  	int m(0);
	int pN(0);
	int p(0);
	int pT(0);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer");
	viewer->createViewPort (0.0, 0.0, 0.33, 1.0, pN);
	viewer->createViewPort (0.34, 0.0, 0.66, 1.0, m);
	viewer->createViewPort (0.67, 0.0, 1.0, 1.0, p);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  	viewer->setBackgroundColor (0, 0, 0);
  	viewer->addPointCloud<pcl::PointXYZ> (cloud, "pts",p);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"pts");
  	viewer->setCameraPosition(camPos[0],camPos[1],camPos[2], centroid[0], centroid[1], centroid[2], 0,1,0);
//  viewer->setCameraFieldOfView(0.523599);
//  viewer->setCameraClipDistances(0.00522511, 50);
  	viewer->initCameraParameters ();
  	viewer->spinOnce(1);
  	double start_time_total = pcl::getTime ();
	double start_time_master = pcl::getTime ();
	double start_time_Read = pcl::getTime();


	// ENTER MAIN LOOP
    while (!viewer->wasStopped ())
    {
      	track++;

    if(newPoints){
    	boost::mutex::scoped_lock readLock(readPointsMutex); 	
    	pointStringRun = *pointString;
		std::stringstream ssPoints(pointStringRun);
		int numPointsRun = *numPoints;
	/*	cout << "Num Points in MAIN: " << numPointsRun << endl;
		cout << "Points in MAIN: " << pointStringRun << endl;
		cout << "Cloud Size: " << cloud->points.size() << endl;*/
		newPoints = false;
		readLock.unlock();
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
		ssPoints >> coefficients->values[3];
		ssPoints >> camPos[0];
		ssPoints >> camPos[1];
		ssPoints >> camPos[2];
		for (size_t i = 0; i < numPointsRun; ++i)
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


		if ((cloudSize_new != cloudSize_old) && cloud->points.size() > ptStart) // ONLY RE-MESH IF CLOUD HAS BEEN UPDATED
		{
			//////////// REMOVE  OUTLIERS ////////////
			start_time = pcl::getTime();
			if (cloud->points.size() < 1000){
				mK = cloud->points.size();
				stdDev = 1.5;
			}

			if (cloud->points.size() > 1000){
				mK = 400;
				stdDev = 1.4;
			}

			if (cloud->points.size() > 2000){
				mK = 300;
				stdDev = 1.2;
			}

			if (cloud->points.size() > 3000){
				mK = 200;
				stdDev = 1;
			}

			statOutlier(mK,stdDev, cloud, cloud_downsampled);
			end_time = pcl::getTime ();
			double elapsed_Stat = end_time - start_time;


			///////////////FILTER//////////////
		  	if (cloud->points.size() > ftStart)
		  	{
	 			start_time = pcl::getTime ();
	 			/// CLEAR POINT CLOUDS /////
	 			cloud_downsampled2->width = cloud_downsampled2->height = 0;
	 			cloud_downsampled2->clear();

	 			if (cloud->points.size() > 1000){
				radDown = 0.1;
				minNei = 5;
				}
				if (cloud->points.size() > 2000){
				radDown = 0.075;
				minNei = 7;
				}
				if (cloud->points.size() > 3000){
				radDown = 0.05;
				minNei = 10;
				}

				FilterPoints(radDown, minNei, cloud_downsampled, cloud_downsampled2);
	 			end_time = pcl::getTime ();
	 			elapsed_rad = end_time - start_time;
	 		}
			///////////////////////////////////////

			////////////PROJECT INLIERS TO PLANE///////////
	  		if (cloud->points.size() > ftStart)
		 	{
		 		projPts(coefficients, cloud_downsampled2, cloud_projected);
		 	}
		 	else
		 	{
		 		projPts(coefficients, cloud_downsampled, cloud_projected);
		 	}
		 	//////////////////////////////////////////////


		 	////////////DOWNSAMPLE BASE POINTS///////////

		 	voxFilter(vLeaf,cloud_projected, cloud_base_filtered);

		 	/////////////////////////////////////////////

			if (cloud->points.size() > ftStart)
		 	{
		 		*cloud_base_filtered += *cloud_downsampled2;
		 	}
		 	else
		 	{
		 		*cloud_base_filtered += *cloud_downsampled;
		 	}


			/////////////MLS SMOOTHING////////////
	 		start_time = pcl::getTime();
	 		//// CLEAR POINT CLOUDS /////
	 	 	mls_points->width = mls_points->height = 0;
	 		mls_points->clear();
	 		MLS(rad, cloud_base_filtered, mls_points);
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
//			pcl::concatenateFields(*mls_points,*cloud_normals, *cloud_point_normals);
			pcl::concatenateFields(*mls_points,*cloud_normals, *cloud_point_normals);
			end_time = pcl::getTime ();
			elapsed_Normal = end_time - start_time;
			////////////////////////////////////

			if(update || meshINT == 0){	

				viewer->removeAllPointClouds();
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud_point_normals, 0, 255, 0);
			//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 255, 255, 255);
	    		viewer->addPolygonMesh(*mesh,"Poisson",m);
	    		viewer->addPolygonMesh(*smoothedMesh,"Smoothed Mesh",p);
	    		viewer->addPointCloud<pcl::PointNormal> (cloud_point_normals, single_color, "Output", pN);
	  		//	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "pts", p);
	  			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"Output");
	     	//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"pts");
	  			viewer->setCameraPosition(camPos[0],camPos[1],camPos[2], centroid[0], centroid[1], centroid[2], 0,1,0);
	  			viewer->spinOnce();
	  			update = false;

				if(meshThread.joinable())
				{	start_time = pcl::getTime();
					meshThread.join();
					end_time = pcl::getTime ();
					elapsed_Mesh = end_time - start_time;
					cout << "PAUSE-\tWaited for: " << elapsed_Mesh << " seconds" << endl;
					cout << "PAUSE-\tPointCloud Size: " << cloudSize_new << endl;
				}	
				meshThread = boost::thread(meshReconstruct,rFac, iter, LogName, depth, cloud_point_normals, mesh, updateMesh, smoothedMesh);
				cout << "STARTED MESH" << endl;
				meshINT++;
			}
		}
	}
			/////////////////////////////////////
	//	}

		/////////////VIEWER////////////////////////////
/*		if (update){
			viewer->removeAllPointClouds();
			if (cloud->points.size() > ptStart )
			{ 
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud_point_normals, 0, 255, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 255, 255, 255);
    		viewer->addPolygonMesh(*mesh,"Poisson",m);
    		viewer->addPointCloud<pcl::PointNormal> (cloud_point_normals, single_color, "Output", pN);
  			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "pts", p);
  			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"Output");
     		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"pts");
  			}
  			else 
  			{	
  			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 0, 255, 0);
  			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "pts", p);
  			}
  			update = false;
		} */
		if(update == false)
		{
			if (cloud->points.size() > ptStart )
			{ 
			viewer->removePointCloud("Output");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud_point_normals, 0, 255, 0);
    		viewer->addPointCloud<pcl::PointNormal> (cloud_point_normals, single_color, "Output", pN);

		//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 255, 255, 255);
  		//	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "pts", p);

  			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"Output");
     	//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,"pts");
  			}
  			else 
  			{
  			viewer->removePointCloud("pts");	
  			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 0, 255, 0);
  			viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "pts", p);
  			}
		}

     	viewer->setCameraPosition(camPos[0],camPos[1],camPos[2], centroid[0], centroid[1], centroid[2], 0,1,0);
	 	viewer->spinOnce();

	 	counter++;
        double new_time = pcl::getTime ();
        elapsed_time = new_time - start_time_master;
        if (elapsed_time > 5.0)
        {       
        // Average Read Times
        /*  	double readNumAv = readNumTrack / readINT;
          	double readPointAv = readPointTrack / readINT;
          	double handleAv = handleTrack / readINT;
          	double meshAV = elapsed_Mesh / meshINT; */
    /*      	cout << "\tRead Num Points Average Time: " << readNumAv << endl;
          	cout << "\tRead Points In Average Time: " << readPointAv << endl;
          	cout << "\tHandle PointCloud Average Time: " << handleAv << endl;
//          	cout << "\tMesh Average Time: " << handleAv << endl; */

          	Logfile.open(LogName, ios::out | ios::app);
          	Logfile << "\n\tFrame Number: " << track << endl;
          	Logfile << "\tCloud Size: " << cloudSize_new << endl;
       //   	Logfile << "\tRead Num Points Average Time: " << readNumAv << endl;
        //  	Logfile << "\tRead Points In Average Time: " << readPointAv << endl;
         // 	Logfile << "\tHandle PointCloud Average Time: " << handleAv << endl;

//          	Logfile << "\tMesh Average Time: " << handleAv << endl;
          	Logfile.close();
          	double frames_per_second = counter / elapsed_time;
          	start_time_master = new_time;
          	counter = 0;
          	cout << "\tfps: " << frames_per_second << endl;
          	cout << "\tCloud Size: " << cloudSize_new << endl;
        }
        ////////////////////////////////////////////////////
    //    meshThread.join();
	}



	if(saveFiles(timeString, mesh, decMesh, smoothedMesh, cloud_point_normals, cloud)){
	  	cout << "Succesfully Saved 7 Files to " << folName << "." << endl;
	  		}

    end_time_total = pcl::getTime();
    totalTime = end_time_total - start_time_total;
    //WRITE TO FILE
    Logfile.open(LogName, ios::out | ios::app );
    Logfile << "/////////TOTALS (END OF RUN)/////////" << endl;
	Logfile << "\tTotal Run Time: " << totalTime << endl;
  	Logfile << "\tTotal Frames Recieved: " << track << endl;
  	Logfile << "\tCloud Size: " << cloudSize_new << endl;
  	Logfile.close();

	if(meshThread.joinable())
		{
		meshThread.join();
		}
    return (0);
}