#include "skeletonjelly.hpp"

int main(int argc, char *argv[]) 
{
	Kinect k;

	XnMapOutputMode output = {640, 480, 30};

	if (k.init(640, 480, 30, false) != XN_STATUS_OK)
	{
		printf("init failed: %s\n", k.errorMessage());
		return -1;
	}

	printf("Ready!\n");

	k.setRenderMode(Kinect::RENDER_SILHOUETTE);
	k.setTicksPerSecond(5);

	k.runThreaded();

	printf("Thread started.");

	k.waitForThread(5 * 1000);
	k.stopThread();

	return (0);
}
