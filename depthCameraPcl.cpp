#include "depthCameraPcl.h"

DepthCameraPcl::DepthCameraPcl(){

}

bool DepthCameraPcl::startGrabbingImpl(){
	grabber = std::make_shared<pcl::io::OpenNI2Grabber>();

	boost::function<void (const Cloud::ConstPtr&)> f([&](const Cloud::ConstPtr in){
		cloudIn(in);
	});

	grabber->registerCallback(f);

	grabber->start();

	return true;
}

bool DepthCameraPcl::stopGrabbingImpl(){
	grabber->stop();
	grabber.reset();

	return true;
}
