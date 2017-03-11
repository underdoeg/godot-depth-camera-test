#ifndef DEPTHCAMERAFREENECT2_H
#define DEPTHCAMERAFREENECT2_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

#include "depthCamera.h"

#include <thread>
#include <mutex>
#include <atomic>

class DepthCameraFreenect2: public DepthCamera{
	OBJ_TYPE(DepthCameraFreenect2, DepthCamera)

public:
	DepthCameraFreenect2();

	void _notification(int p_what);

protected:
	bool startGrabbingImpl() override;
	bool stopGrabbingImpl() override;

private:

	void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);

	libfreenect2::Freenect2Device::IrCameraParams irParams;
	libfreenect2::Freenect2 freenect2;

	std::shared_ptr<libfreenect2::Freenect2Device> device;
	std::shared_ptr<libfreenect2::Registration> registration;
	std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener;

	libfreenect2::FrameMap frames;
	libfreenect2::Frame undistorted;

	CloudPtr cloudThread;
	CloudPtr cloudCopy;
	std::atomic_bool bNewFrame;
	std::atomic_bool bStopThread;

	Eigen::Matrix<float,512,1> colmap_;
	Eigen::Matrix<float,424,1> rowmap_;

	std::thread thread;
	std::mutex mutex;
};

#endif // DEPTHCAMERAFREENECT2_H
