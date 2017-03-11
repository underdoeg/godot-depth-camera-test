#ifndef DEPTHCLUSTERS_H
#define DEPTHCLUSTERS_H

#include "depthCameraConsumer.h"
#include "scene/3d/immediate_geometry.h"
#include "scene/3d/test_cube.h"

class DepthClusters: public DepthCameraConsumer{
	OBJ_TYPE(DepthClusters, DepthCameraConsumer);

public:
	static void _bind_methods();

	DepthClusters();

	void _notification(int p_what);

	int getMaxAmount() const;
	void setMaxAmount(int value);

	int getMinAmount() const;
	void setMinAmount(int value);

	float getTolerance() const;
	void setTolerance(float value);

	bool getDebugRender() const;
	void setDebugRender(bool value);

	std::vector<Vector3> getPoints();

	bool getPaused() const;
	void setPaused(bool value);

protected:
	void onNewFrame(Cloud::ConstPtr cloud) override;

private:
	void update();

	int minAmount, maxAmount;
	float tolerance;
	bool bDebugRender, bPaused;

	std::vector<Vector3> points;
	std::vector<CloudPtr> clusters;

	std::vector<TestCube*> testCubes;
};

#endif // DEPTHCLUSTERS_H
