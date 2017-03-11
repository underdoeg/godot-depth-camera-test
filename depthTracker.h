#ifndef INTERACTIVEWINDOW_H
#define INTERACTIVEWINDOW_H

#include <chrono>

#include "scene/3d/camera.h"
#include <pcl/common/common.h>
#include "depthCameraConsumer.h"
#include "depthClusters.h"

class DepthTracker: public Node{
	OBJ_TYPE(DepthTracker,Node);

	class Point{
	public:
		/*
			Point(const Point& p){
				position = p.position;
				created = p.created;
				id = p.id;
			}
			*/

		Point(){
			position.x = position.y = 0;
			created = std::chrono::system_clock::now();
			id = 0;
		}

		Point(Vector2 p):Point(){
			position = p;
		}

		float getAge() const{
			return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - created).count() / 1000.f;
		}

		Vector2 position;
		unsigned id;
		std::chrono::system_clock::time_point created;
	};

	using PointList = std::vector<Point>;

public:
	static void _bind_methods();

	DepthTracker();

	void _notification(int what);

	float getMaxDistance() const;
	void setMaxDistance(float value);

	float getMinAge() const;
	void setMinAge(float value);

	float getInterpolation() const;
	void setInterpolation(float value);

	bool getDebugRender() const;
	void setDebugRender(bool value);

	bool hasDepthClusters();
	DepthClusters* getDepthClusters();

protected:
	void newPoints();

private:
	void setInput(std::vector<Vector3> ps);
	unsigned newId();

	float maxDistance, minAge, interpolation;
	bool bDebugRender;

	PointList points;
	PointList pointsActive;

	size_t maxPointsIn;
	unsigned curId;

	DepthClusters* clusters;

	std::vector<TestCube*> testCubes;
	std::vector<Material> testMaterials;
};

#endif // INTERACTIVEWINDOW_H
