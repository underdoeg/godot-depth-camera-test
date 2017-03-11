#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

#include "depthCamera.h"

void DepthCamera::_bind_methods(){

	ObjectTypeDB::bind_method(_MD("setDownsampling", "vector"), &DepthCamera::setDownsampling);
	ObjectTypeDB::bind_method(_MD("getDownsampling"), &DepthCamera::getDownsampling);
	ADD_PROPERTY( PropertyInfo(Variant::VECTOR3,"downsampling"), _SCS("setDownsampling"), _SCS("getDownsampling"));

	ObjectTypeDB::bind_method(_MD("setMaxDepth", "float"), &DepthCamera::setMaxDepth);
	ObjectTypeDB::bind_method(_MD("getMaxDepth"), &DepthCamera::getMaxDepth);
	ADD_PROPERTY( PropertyInfo(Variant::REAL,"maxDepth"), _SCS("setMaxDepth"), _SCS("getMaxDepth"));

	ObjectTypeDB::bind_method(_MD("setEnableBounds", "bool"), &DepthCamera::setEnableBounds);
	ObjectTypeDB::bind_method(_MD("getEnableBounds"), &DepthCamera::getEnableBounds);
	ADD_PROPERTY( PropertyInfo(Variant::BOOL,"enableBounds"), _SCS("setEnableBounds"), _SCS("getEnableBounds"));

	ObjectTypeDB::bind_method(_MD("setBoundsMin", "vector"), &DepthCamera::setBoundsMin);
	ObjectTypeDB::bind_method(_MD("getBoundsMin"), &DepthCamera::getBoundsMin);
	ADD_PROPERTY( PropertyInfo(Variant::VECTOR3,"boundsMin"), _SCS("setBoundsMin"), _SCS("getBoundsMin"));

	ObjectTypeDB::bind_method(_MD("setBoundsMax", "vector"), &DepthCamera::setBoundsMax);
	ObjectTypeDB::bind_method(_MD("getBoundsMax"), &DepthCamera::getBoundsMax);
	ADD_PROPERTY( PropertyInfo(Variant::VECTOR3,"boundsMax"), _SCS("setBoundsMax"), _SCS("getBoundsMax"));

	ADD_SIGNAL(MethodInfo("newFrame"));
}

DepthCamera::DepthCamera():
	bRunning(false),
	maxDepth(9999),
	bEnableBounds(false)
{

	downsampling = {0, 0, 0};

	cloud = CloudPtr(new Cloud());

	//	boost::function<void (const Cloud::ConstPtr&)> f = boost::bind(&DepthCamera::cloudIn, this, _1);
#include <pcl/common/transforms.h>
	//	grabber.registerCallback(f);

	//grabber.registerCallback ([&](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud){
	//});
}

DepthCamera::~DepthCamera(){
	stopGrabbing();
}

void DepthCamera::startGrabbing(){
	if(bRunning)
		return;

	bRunning = startGrabbingImpl();;
	print_line("start depth cam grabber");
}

void DepthCamera::stopGrabbing(){
	if(!bRunning)
		return;

	bRunning = !stopGrabbingImpl();
	print_line("stop depth cam grabber");
}

void DepthCamera::_notification(int what){
	switch(what){
	case NOTIFICATION_ENTER_WORLD:
		startGrabbing();
		break;
	case NOTIFICATION_EXIT_WORLD:
		stopGrabbing();
		break;
	}
}

Cloud::ConstPtr DepthCamera::getCloud(){
	return cloud;
}

void DepthCamera::setDownsampling(const Vector3 v){
	downsampling = v;
}

Vector3 DepthCamera::getDownsampling(){
	return downsampling;
}

void DepthCamera::setMaxDepth(const float &d){
	maxDepth = d;
}

float DepthCamera::getMaxDepth(){
	return maxDepth;
}

void DepthCamera::cloudIn(const Cloud::ConstPtr &newCloud){
	*cloud = *newCloud;

	//auto cloudFiltered = CloudPtr(new Cloud());

	// limit z
	pcl::PassThrough<Point> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, maxDepth);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud);

	// downsample
	if(downsampling.length_squared() > 0.0001f){
		// Create the filtering object
		pcl::VoxelGrid<Point> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(downsampling.x, downsampling.y, downsampling.z);
		sor.filter(*cloud);
		//*cloud = *cloudFiltered;
	}


	// apply transform
	Transform t = get_global_transform();

	auto eulers = t.get_basis().get_euler();
	Eigen::Matrix3f n;
	n = Eigen::AngleAxisf(eulers.x, Eigen::Vector3f::UnitX())
		*Eigen::AngleAxisf(eulers.y, Eigen::Vector3f::UnitY())
		*Eigen::AngleAxisf(eulers.z, Eigen::Vector3f::UnitZ());

	auto scale = t.get_basis().get_scale();

	Eigen::Matrix4f mat;
	mat << n, Eigen::Vector3f(t.origin.x, t.origin.y, t.origin.z),
			0, 0, 0, 1;

	//n.col(3).head<3>() << 1, 2, 3;

	pcl::transformPointCloud(*cloud, *cloud, Eigen::Affine3f(mat) * Eigen::Scaling(Eigen::Vector3f(scale.x, -scale.y, scale.z))); //Eigen::Affine3f(eigenMat));

	//crop region
	if(bEnableBounds){
		pcl::CropBox<Point> crop;
		crop.setInputCloud(cloud);
		crop.setMin({boundsMin.x, boundsMin.y, boundsMin.z, 1});
		crop.setMax({boundsMax.x, boundsMax.y, boundsMax.z, 1});
		crop.filter(*cloud);
	}

	emit_signal("newFrame");
}


bool DepthCamera::getEnableBounds() const{
	return bEnableBounds;
}

void DepthCamera::setEnableBounds(bool value){
	bEnableBounds = value;
}

Vector3 DepthCamera::getBoundsMin() const{
	return boundsMin;
}

void DepthCamera::setBoundsMin(const Vector3 &value){
	boundsMin = value;
}

Vector3 DepthCamera::getBoundsMax() const{
	return boundsMax;
}

void DepthCamera::setBoundsMax(const Vector3 &value){
	boundsMax = value;
}
