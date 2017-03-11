#include "depthCameraRender.h"
#include <random>
#include "depthCamera.h"

void DepthCameraRender::_bind_methods(){
	//ObjectTypeDB::bind_method(_MD(setTracker))

	//ADD_PROPERTY( PropertyInfo(Variant::NODE_PATH,"tracker"), _SCS("setTracker"), _SCS("getTracker"));
}

DepthCameraRender::DepthCameraRender():
	geom(nullptr)
{
}

void DepthCameraRender::_notification(int p_what) {

	switch(p_what) {

	case NOTIFICATION_ENTER_TREE: {
		set_fixed_process(true);
		memdelete_notnull(geom);
		geom = memnew(ImmediateGeometry);
		add_child(geom, false);
	}

	case NOTIFICATION_FIXED_PROCESS:{
		update();
	}

	}
}

void DepthCameraRender::update(){
	if(!hasDepthCam())
		return;

	geom->clear();
	geom->begin(Mesh::PRIMITIVE_POINTS);
	for(auto p: *getDepthCam()->getCloud()){
		if(std::isnan(p.x))
			continue;

		geom->add_vertex({p.x, p.y, p.z});
	}
	geom->set_color({1, 1, 1, 1});
	geom->end();

}

void DepthCameraRender::onNewFrame(Cloud::ConstPtr cloud){

}
