#pragma once

#include <scene/3d/immediate_geometry.h>
#include "depthCameraConsumer.h"

class DepthCameraRender: public DepthCameraConsumer{

	OBJ_TYPE(DepthCameraRender, DepthCameraConsumer);

public:

	static void _bind_methods();

	DepthCameraRender();

	void _notification(int p_what);

	void update();
private:
	ImmediateGeometry* geom;
	Mesh mesh;

	// DepthCameraConsumer interface
protected:
	void onNewFrame(Cloud::ConstPtr cloud) override;
};

