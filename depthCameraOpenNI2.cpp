#include "depthCameraOpenNI2.h"

DepthCameraOpenNI2::DepthCameraOpenNI2(){

}

bool DepthCameraOpenNI2::startGrabbingImpl(){
	static bool bInit = false;
	if(!bInit){
		openni::OpenNI::initialize();
		bInit = true;
	}

	if(device.open(openni::ANY_DEVICE) != openni::STATUS_OK){
		print_line("Could not open device");
		return false;
	}
	device.setDepthColorSyncEnabled(false);
	stream.create(device, openni::SensorType::SENSOR_DEPTH);
	stream.addNewFrameListener(this);
	stream.start();
	return true;
}

bool DepthCameraOpenNI2::stopGrabbingImpl(){
	device.close();
	stream.destroy();
	return true;
}

void DepthCameraOpenNI2::onNewFrame(openni::VideoStream& in){
	openni::VideoFrameRef ref;
	in.readFrame(&ref);

	const int width = ref.getWidth();
	const int height = ref.getHeight();

	auto cloud = CloudPtr(new Cloud());
	size_t size = width * height;
	if(cloud->points.size() != size)
		cloud->resize(size);

	const float bad_point = std::numeric_limits<float>::quiet_NaN();
	const float factor = .001f;

	float worldX, worldY, worldZ;
	worldX = worldY = worldZ = 0;

	for(int h=0; h<height; h++){
		for(int w=0; w<width; w++){
			int i = h*width+w;
			openni::CoordinateConverter::convertDepthToWorld(in, w, h, ((openni::DepthPixel*)ref.getData())[i], &worldX, &worldY, &worldZ);
			Point& pOut = cloud->at(i);

			pOut.x = worldX * factor;
			pOut.y = worldY * factor;
			pOut.z = worldZ * factor;

			if(pOut.z == 0){
				pOut.x = bad_point;
				pOut.y = bad_point;
				pOut.z = bad_point;
			}
		}
	}

	cloudIn(cloud);
}
