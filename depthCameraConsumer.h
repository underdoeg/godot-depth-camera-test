#ifndef DEPTHCAMERACONSUMER_H
#define DEPTHCAMERACONSUMER_H

#include "scene/main/node.h"

#include "depthCamera.h"

class DepthCameraConsumer: public Node{
	OBJ_TYPE(DepthCameraConsumer, Node)
	OBJ_CATEGORY("Depth Camera");

public:
	static void _bind_methods();

	DepthCameraConsumer();

	void _notification(int what);

	bool hasDepthCam();
	DepthCamera* getDepthCam();

protected:
	virtual void onNewFrame(Cloud::ConstPtr cloud){};

private:
	void newFrame();

	NodePath pathToDepthCam;
	DepthCamera* depthCam;
};

#endif // DEPTHCAMERACONSUMER_H
