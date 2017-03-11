#ifndef TRACKER_H
#define TRACKER_H

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/openni2_grabber.h>

#include "core/print_string.h"
#include "scene/3d/spatial.h"

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;


class DepthCamera: public Spatial{
	OBJ_TYPE(DepthCamera, Spatial)
	OBJ_CATEGORY("Depth Camera");

public:
	static void _bind_methods();

	DepthCamera();
	virtual ~DepthCamera();

	void startGrabbing();
	void stopGrabbing();

	void _notification(int p_what);

	Cloud::ConstPtr getCloud();

	void setDownsampling(const Vector3 v);
	Vector3 getDownsampling();

	void setMaxDepth(const float& d);
	float getMaxDepth();

	void setEnableBounds(bool value);
	bool getEnableBounds() const;

	Vector3 getBoundsMin() const;
	void setBoundsMin(const Vector3 &value);

	Vector3 getBoundsMax() const;
	void setBoundsMax(const Vector3 &value);

protected:
	void cloudIn(const Cloud::ConstPtr &cloud);

	virtual bool startGrabbingImpl(){return true;};
	virtual bool stopGrabbingImpl(){return true;};

protected:
	bool bRunning;
	CloudPtr cloud;
	Vector3 downsampling;
	float maxDepth;
	bool bEnableBounds;
	Vector3 boundsMin, boundsMax;
};

//std::shared_ptr<Tracker> getTracker();

#endif // TRACKER_H
