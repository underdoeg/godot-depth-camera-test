#include <libfreenect2/logger.h>
#include "depthCameraFreenect2.h"

DepthCameraFreenect2::DepthCameraFreenect2():
	undistorted(512, 424, 4),
	cloudThread(new Cloud()),
	cloudCopy(new Cloud()){
	libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Error));
	bNewFrame = false;
}

void DepthCameraFreenect2::_notification(int p_what){
	switch(p_what){
	case NOTIFICATION_READY:
		set_fixed_process(true);
		break;
	case NOTIFICATION_FIXED_PROCESS:
		if(bNewFrame){
			mutex.lock();
			*cloudCopy = *cloudThread;
			mutex.unlock();
			cloudIn(cloudCopy);
		}
		bNewFrame = false;
		break;
	}
}

bool DepthCameraFreenect2::startGrabbingImpl(){
	stopGrabbingImpl();

	if(freenect2.enumerateDevices() == 0){
		print_line("no kinect connected");
		return false;
	}

	libfreenect2::PacketPipeline* pipeline;

#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
	pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
	pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
	pipeline = new libfreenect2::CpuPacketPipeline();
#endif
#endif

	//check serial number
	std::string serialNumber;
	if(serialNumber.empty()){
		serialNumber = freenect2.getDefaultDeviceSerialNumber();
	}

	//open device
	device = std::shared_ptr<libfreenect2::Freenect2Device>(freenect2.openDevice(serialNumber, pipeline));
	if(!device){
		return false;
	}

	// calibration
	irParams.fx = 365.456f;
	irParams.fy = 365.456f;
	irParams.cx = 254.878f;
	irParams.cy = 205.395f;
	irParams.k1 = 0.0905474f;
	irParams.k2 = -0.26819f;
	irParams.k3 = 0.0950862f;
	irParams.p1 = 0.0f;
	irParams.p2 = 0.0f;
	device->setIrCameraParams(irParams);

	//set listener
	int frameTypes = libfreenect2::Frame::Type::Depth;
	listener = std::make_shared<libfreenect2::SyncMultiFrameListener>(frameTypes);
	//device->setColorFrameListener(this);
	device->setIrAndDepthFrameListener(listener.get());
	libfreenect2::Freenect2Device::Config config;
	config.EnableBilateralFilter = true;
	config.EnableEdgeAwareFilter = true;
	config.MinDepth = .3;
	config.MaxDepth = 6;
	device->setConfiguration(config);

	//registration
	registration = std::shared_ptr<libfreenect2::Registration>(new libfreenect2::Registration(device->getIrCameraParams(), device->getColorCameraParams()));

	//start the device
	if(!device->startStreams(false, true))
		return false;

	bStopThread = false;
	thread = std::thread([&]{

		libfreenect2::Frame undistorted(512, 424, 4);

		const auto qnan_ = std::numeric_limits<float>::quiet_NaN();

		auto cloudTmp = CloudPtr(new Cloud());

		if(!device)
			return;

		prepareMake3D(device->getIrCameraParams());

		while(!bStopThread){

			if(!listener)
				return;

			if(!listener->waitForNewFrame(frames, 10))
				continue;

			auto depth = frames[libfreenect2::Frame::Depth];

			registration->undistortDepth(depth, &undistorted);

			listener->release(frames);

			const auto w = undistorted.width;
			const auto h = undistorted.height;

			const unsigned skip = 1;

			cloudTmp.reset(new Cloud(w/skip, h/skip));

			Point* itP = &cloudTmp->points[0];
			const float * itD0 = (float *)undistorted.data;


			for(unsigned y = 0; y < h; y+=skip){
				const unsigned int offset = y * w;
				const float *itD = itD0 + offset;
				const float dy = rowmap_(y);

				for(size_t x = 0; x < w; x+=skip, ++itP, itD+=skip){
					const float depth_value = *itD / 1000.0f;

					if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){

						const float rx = colmap_(x) * depth_value;
						const float ry = dy * depth_value;

						itP->z = depth_value;
						itP->x = rx;
						itP->y = ry;

					} else {
						itP->z = qnan_;
						itP->x = qnan_;
						itP->y = qnan_;
					}
				}
			}

			cloudTmp->is_dense = false;

			mutex.lock();
			*cloudThread = *cloudTmp;
			mutex.unlock();
			bNewFrame = true;
		}
	});

	return true;
}

bool DepthCameraFreenect2::stopGrabbingImpl(){
	bStopThread = true;

	if(thread.joinable())
		thread.join();

	if(!device)
		return false;
	device->stop();
	device->close();

	return true;
}

void DepthCameraFreenect2::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams &depth_p){
	const int w = 512;
	const int h = 424;
	float * pm1 = colmap_.data();
	float * pm2 = rowmap_.data();
	for(int i = 0; i < w; ++i)
	{
		*pm1++ = (i - depth_p.cx + 0.5) / depth_p.fx;
	}
	for (int i = 0; i < h; i++)
	{
		*pm2++ = (i - depth_p.cy + 0.5) / depth_p.fy;
	}
}
