#include "register_types.h"
#include "object_type_db.h"
#include "depthTracker.h"
#include "depthCameraRender.h"
#include "depthCamera.h"
#include "depthCameraConsumer.h"
#include "depthCameraOpenNI2.h"
#include "depthCameraPcl.h"
#include "depthCameraFreenect2.h"
#include "depthClusters.h"

void register_depthCamera_types(){
	ObjectTypeDB::register_type<DepthCamera>();
	ObjectTypeDB::register_type<DepthCameraOpenNI2>();
	ObjectTypeDB::register_type<DepthCameraPcl>();
	ObjectTypeDB::register_type<DepthCameraFreenect2>();
	ObjectTypeDB::register_type<DepthCameraConsumer>();
	ObjectTypeDB::register_type<DepthCameraRender>();
	ObjectTypeDB::register_type<DepthClusters>();
	ObjectTypeDB::register_type<DepthTracker>();
}

void unregister_depthCamera_types(){
	//
}
