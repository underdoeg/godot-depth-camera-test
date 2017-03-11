#include "depthCameraConsumer.h"

void DepthCameraConsumer::_bind_methods(){
	//ObjectTypeDB::bind_method( _MD("getDepthCam"), &DepthCameraConsumer::getDepthCam);
	ObjectTypeDB::bind_method( _MD("hasDepthCam"), &DepthCameraConsumer::hasDepthCam);
	ObjectTypeDB::bind_method( _MD("newFrame"), &DepthCameraConsumer::newFrame);

	//ADD_PROPERTY( PropertyInfo(Variant::NODE_PATH,"depthCamera"), _SCS("setDepthCamera"), _SCS("getDepthCamera"));

	ADD_SIGNAL(MethodInfo("newFrame"));
}

DepthCameraConsumer::DepthCameraConsumer():
	depthCam(nullptr)
{

}

void DepthCameraConsumer::_notification(int what){
	switch(what){
	case NOTIFICATION_PARENTED:
		depthCam = get_parent()->cast_to<DepthCamera>();
		if(hasDepthCam())
			depthCam->connect("newFrame", this, "newFrame");
		break;
	}
}


bool DepthCameraConsumer::hasDepthCam(){
	return depthCam;
}

DepthCamera* DepthCameraConsumer::getDepthCam(){
	return depthCam;
}

void DepthCameraConsumer::newFrame(){
	onNewFrame(getDepthCam()->getCloud());
}
