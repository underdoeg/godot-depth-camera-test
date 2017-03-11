#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>

#include "depthClusters.h"

void DepthClusters::_bind_methods(){
	ObjectTypeDB::bind_method(_MD("setMinAmount", "int"), &DepthClusters::setMinAmount);
	ObjectTypeDB::bind_method(_MD("getMinAmount"), &DepthClusters::getMinAmount);
	ADD_PROPERTY( PropertyInfo(Variant::INT,"minAmount"), _SCS("setMinAmount"), _SCS("getMinAmount"));

	ObjectTypeDB::bind_method(_MD("setMaxAmount", "int"), &DepthClusters::setMaxAmount);
	ObjectTypeDB::bind_method(_MD("getMaxAmount"), &DepthClusters::getMaxAmount);
	ADD_PROPERTY( PropertyInfo(Variant::INT,"maxAmount"), _SCS("setMaxAmount"), _SCS("getMaxAmount"));

	ObjectTypeDB::bind_method(_MD("setTolerance", "float"), &DepthClusters::setTolerance);
	ObjectTypeDB::bind_method(_MD("getTolerance"), &DepthClusters::getTolerance);
	ADD_PROPERTY( PropertyInfo(Variant::REAL,"tolerance"), _SCS("setTolerance"), _SCS("getTolerance"));

	ObjectTypeDB::bind_method(_MD("setDebugRender", "bool"), &DepthClusters::setDebugRender);
	ObjectTypeDB::bind_method(_MD("getDebugRender"), &DepthClusters::getDebugRender);
	ADD_PROPERTY( PropertyInfo(Variant::BOOL,"debugRender"), _SCS("setDebugRender"), _SCS("getDebugRender"));


	ObjectTypeDB::bind_method(_MD("setPaused", "bool"), &DepthClusters::setPaused);
	ObjectTypeDB::bind_method(_MD("getPaused"), &DepthClusters::getPaused);
	ADD_PROPERTY( PropertyInfo(Variant::BOOL,"paused"), _SCS("setPaused"), _SCS("getPaused"));

	ADD_SIGNAL(MethodInfo("newPoints"));
}

DepthClusters::DepthClusters():
	minAmount(10),
	maxAmount(9999),
	tolerance(.05f),
	bDebugRender(true),
	bPaused(false)
{}

void DepthClusters::_notification(int p_what){
	switch(p_what){
	case NOTIFICATION_ENTER_TREE: {
		//set_fixed_process(true);

		for(auto& t: testCubes){
			memdelete_notnull(t);
		}

		for(auto i=0; i<15; i++){
			testCubes.push_back(memnew(TestCube));
			testCubes.back()->hide();
			add_child(testCubes.back(), false);
		}
	}
	case NOTIFICATION_FIXED_PROCESS:
		break;
	}
}

int DepthClusters::getMaxAmount() const{
	return maxAmount;
}

void DepthClusters::setMaxAmount(int value){
	maxAmount = value;
}

int DepthClusters::getMinAmount() const{
	return minAmount;
}

void DepthClusters::setMinAmount(int value){
	minAmount = value;
}

float DepthClusters::getTolerance() const{
	return tolerance;
}

void DepthClusters::setTolerance(float value){
	tolerance = value;
}

void DepthClusters::update(){

}

bool DepthClusters::getPaused() const{
	return bPaused;
}

void DepthClusters::setPaused(bool value){
	bPaused = value;
}

bool DepthClusters::getDebugRender() const{
	return bDebugRender;
}

void DepthClusters::setDebugRender(bool value){
	bDebugRender = value;
}

std::vector<Vector3> DepthClusters::getPoints(){
	return points;
}

void DepthClusters::onNewFrame(Cloud::ConstPtr cloud){
	if(bPaused)
		return;

	points.clear();
	clusters.clear();

	if(!cloud->empty()){


		//extract clusters
		pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
		tree->setInputCloud(cloud);

		std::vector<pcl::PointIndices> clusterIndices;
		pcl::EuclideanClusterExtraction<Point> ec;
		ec.setClusterTolerance(tolerance); // 5cm
		ec.setMinClusterSize(minAmount);
		ec.setMaxClusterSize(maxAmount);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(clusterIndices);

		//LogInfo() << clusterIndices.size();



		for(auto indices: clusterIndices){
			//create a new cloud of the clusters and calculate the center
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for(auto i: indices.indices){
				cluster->points.push_back(cloud->points[i]);
			}
			clusters.push_back(cluster);
			Eigen::Vector4f v;
			pcl::compute3DCentroid(*cluster, v);
			points.push_back({v[0], v[1], v[2]});
		}

	}

	// update debug render
	size_t i = 0;
	if(bDebugRender){
		for(; i<points.size(); i++){
			if(i<testCubes.size()){
				auto cube = testCubes[i];
				cube->show();
				cube->set_translation(points[i]);
				cube->set_scale({.1, .1, .1});
			}
		}
	}

	for(; i<testCubes.size(); i++){
		testCubes[i]->hide();
	}

	emit_signal("newPoints");
}
