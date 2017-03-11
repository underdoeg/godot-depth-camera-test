#ifndef DEPTHCAMERAOPENNI2_H
#define DEPTHCAMERAOPENNI2_H

#include <OpenNI.h>

#include "depthCamera.h"

class DepthCameraOpenNI2: public DepthCamera, public openni::VideoStream::NewFrameListener{
	OBJ_TYPE(DepthCameraOpenNI2, DepthCamera);

public:
	DepthCameraOpenNI2();

private:
	bool startGrabbingImpl();
	bool stopGrabbingImpl();
	void onNewFrame(openni::VideoStream&);

	openni::Device device;
	openni::VideoStream stream;
};

#endif // DEPTHCAMERAOPENNI2_H
