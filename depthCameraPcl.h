#ifndef DEPTHCAMERAPCL_H
#define DEPTHCAMERAPCL_H

#include "depthCamera.h"

class DepthCameraPcl: public DepthCamera{
	OBJ_TYPE(DepthCameraPcl, DepthCamera)

public:
	DepthCameraPcl();

	bool startGrabbingImpl();
	bool stopGrabbingImpl();

private:
	std::shared_ptr<pcl::io::OpenNI2Grabber> grabber;
};

#endif // DEPTHCAMERAPCL_H
