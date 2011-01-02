#include "skeletonjelly.hpp"

static const char FORMAT_BPP[2] = {4, 3};

#define CHECK_RC(x) {\
	if (x != XN_STATUS_OK)\
		return x;\
}

void XN_CALLBACK_TYPE cb_newUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	((Kinect *)pCookie)->onNewUser(nId);
}

void XN_CALLBACK_TYPE cb_lostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	((Kinect *)pCookie)->onLostUser(nId);
}

void XN_CALLBACK_TYPE cb_poseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	((Kinect *)pCookie)->onPoseDetected(strPose, nId);
}

void XN_CALLBACK_TYPE cb_calibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	((Kinect *)pCookie)->onCalibrationStart(nId);
}

void XN_CALLBACK_TYPE cb_calibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	((Kinect *)pCookie)->onCalibrationEnd(nId, bSuccess);
}

Kinect::Kinect()
{
	_needPose = false;
	_error = XN_STATUS_OK;
	_init = false;
	_paused = false;
	_autoTrack = true;

	_gotImage = false;
	_renderFormat = RENDER_RGBA;

	_tickTime = 1000 / 30;

	_eventCallback = 0;
	_callbackData = 0;

#ifdef _WIN32
	_thread = 0;
#endif

	for (int i = 0; i < MAX_USERS; ++i)
	{
		_userStatus[i] = USER_INACTIVE;
		_userData[i] = 0;
	}
}

Kinect::~Kinect()
{
	stopThread();

	for (int i = 0; i < MAX_USERS; ++i)
		delete _userData[i];

	_context.Shutdown();
}

XnStatus Kinect::init(SensorMode depthMode, SensorMode imageMode)
{
	XnMapOutputMode sensorModes[] =
	{
		{ 0, 0, 0 }, /* SENSOR_DISABLED */
		{ 320, 240, 60 }, /* SENSOR_QVGA */
		{ 640, 480, 30 }, /* SENSOR_VGA */
		{ 1280, 1024, 10 } /* SENSOR_SXGA */
	};

	xn::Query qDepth, qImage;
	XnCallbackHandle cb_user, cb_calibration, cb_pose;

	if (depthMode == SENSOR_DISABLED)
		return XN_STATUS_NOT_IMPLEMENTED;

	qDepth.AddSupportedCapability(XN_CAPABILITY_MIRROR);
	qDepth.AddSupportedMapOutputMode(sensorModes[depthMode]);

	qImage.AddSupportedCapability(XN_CAPABILITY_MIRROR);
	qImage.AddSupportedMapOutputMode(sensorModes[imageMode]);

	_error = _context.Init();
	CHECK_RC(_error);

	/* init depth generator */
	{
    	_error = _depth.Create(_context, &qDepth);
    	CHECK_RC(_error);
    
    	_error = _depth.SetMapOutputMode(sensorModes[depthMode]);
    	CHECK_RC(_error);

		_depth.GetMirrorCap().SetMirror(true);
	}

	if (imageMode != SENSOR_DISABLED) /* init image generator */
	{
    	_error = _image.Create(_context, &qImage);
    	CHECK_RC(_error);
    
    	_error = _image.SetMapOutputMode(sensorModes[imageMode]);
    	CHECK_RC(_error);
    
    	_error = _image.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
    	CHECK_RC(_error);

		_image.GetMirrorCap().SetMirror(true);
		_gotImage = true;
	}

	/* init user generator */
	{
    	_error = _userGen.Create(_context);
    	CHECK_RC(_error);
    
    	if (!_userGen.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    		return XN_STATUS_ERROR;
    
    	_userGen.RegisterUserCallbacks(cb_newUser, cb_lostUser, this, cb_user);
    	_userGen.GetSkeletonCap().RegisterCalibrationCallbacks(cb_calibrationStart, cb_calibrationEnd, this, cb_calibration);
    
    	if (_userGen.GetSkeletonCap().NeedPoseForCalibration())
    	{
    		_needPose = true;
    
    		if (!_userGen.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
    			return XN_STATUS_ERROR;
    
    		_userGen.GetPoseDetectionCap().RegisterToPoseCallbacks(cb_poseDetected, NULL, this, cb_pose);
    		_userGen.GetSkeletonCap().GetCalibrationPose(_calibrationPose);
    	}
    
    	_userGen.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	}

	_error = _context.StartGeneratingAll();
	CHECK_RC(_error);

	_init = true;
	return XN_STATUS_OK;
}

void Kinect::tick()
{
	int nextTick = GetTickCount() + _tickTime;

	// putchar('t');
	_context.WaitAndUpdateAll();

	for (int i = 0; i < MAX_USERS; ++i)
		if (_userData[i] != 0)
			updateUserData(i, _userData[i]);

	int sleep_time = nextTick - GetTickCount();
	if (sleep_time > 0)
		Sleep(sleep_time);
}

XnStatus Kinect::resetUser(XnUserID id /*= DEFAULT_USER*/)
{
	if (id < MAX_USERS && _userStatus[id] != USER_INACTIVE)
	{
    	_userGen.GetPoseDetectionCap().StopPoseDetection(id);
    	_userGen.GetSkeletonCap().Reset(id);
		_userStatus[id] = USER_ACTIVE;
		return XN_STATUS_OK;
	}

	return XN_STATUS_NO_MATCH;
}

XnStatus Kinect::trackUser(XnUserID id /*= DEFAULT_USER*/)
{
	if (id < MAX_USERS && _userStatus[id] != USER_INACTIVE)
	{
    	if (_userStatus[id] != USER_ACTIVE)
    		resetUser(id);
    
    	if (_needPose)
    	{
    		_userGen.GetPoseDetectionCap().StartPoseDetection(_calibrationPose, id);
    		_userStatus[id] |= USER_LOOKING_FOR_POSE;
    	}
    	else
    	{
    		_userGen.GetSkeletonCap().RequestCalibration(id, true);
    	}

		return XN_STATUS_OK;
	}

	return XN_STATUS_NO_MATCH;
}

void Kinect::onNewUser(XnUserID id)
{
	if (id < MAX_USERS)
	{
		_userStatus[id] = USER_ACTIVE;
		_userData[id] = new Kinect_UserData;

		if (_autoTrack)
			trackUser(id);

		if (_eventCallback != 0)
			this->_eventCallback(this, CB_NEW_USER, id, _callbackData);
	}
}

void Kinect::onLostUser(XnUserID id)
{
	if (id < MAX_USERS)
	{
		_userStatus[id] = USER_INACTIVE;
		delete _userData[id];
		_userData[id] = 0;

		if (_eventCallback != 0)
			this->_eventCallback(this, CB_LOST_USER, id, _callbackData);
	}
}


void Kinect::onPoseDetected(const XnChar* strPose, XnUserID id)
{
	if (id < MAX_USERS)
	{
		_userStatus[id] &= ~USER_LOOKING_FOR_POSE;
		_userStatus[id] |= USER_GOT_POSE;

    	_userGen.GetPoseDetectionCap().StopPoseDetection(id);
    	_userGen.GetSkeletonCap().RequestCalibration(id, true);

		if (_eventCallback != 0)
			this->_eventCallback(this, CB_POSE_DETECTED, id, _callbackData);
	}
}

void Kinect::onCalibrationStart(XnUserID id)
{
	if (id < MAX_USERS)
	{
		_userStatus[id] |= USER_CALIBRATING;

		if (_eventCallback != 0)
			this->_eventCallback(this, CB_CALIBRATION_START, id, _callbackData);
	}
}

void Kinect::onCalibrationEnd(XnUserID id, XnBool success)
{
	if (id < MAX_USERS)
	{
    	if (success)
    	{
    		_userGen.GetSkeletonCap().StartTracking(id);
    		_userStatus[id] &= ~USER_CALIBRATING;
    		_userStatus[id] |= USER_TRACKING;

    		if (_eventCallback != 0)
    			this->_eventCallback(this, CB_CALIBRATION_SUCCESS, id, _callbackData);
    	}
    	else
    	{
    		_userStatus[id] = USER_ACTIVE;

    		if (_eventCallback != 0)
    			this->_eventCallback(this, CB_CALIBRATION_FAIL, id, _callbackData);

			if (_autoTrack)
				trackUser(id);
    	}
	}
}

void Kinect::calculateHistogram(int resolutionX, int resolutionY, const XnDepthPixel *depth_pixels)
{
	int numPoints = 0;

	unsigned int *histogram = _histogram;

	memset((void *)histogram, 0x0, MAX_DEPTH * sizeof(unsigned int));

	for (int y = 0; y < resolutionY; y++)
	{
		for (int x = 0; x < resolutionX; x++)
		{
			unsigned int val = *depth_pixels++ & DEPTH_MASK;

			if (val != 0)
			{
				histogram[val]++;
				numPoints++;
			}
		}
	}

	for (int idx = 1; idx < MAX_DEPTH; idx++)
	{
		histogram[idx] += histogram[idx - 1];
	}

	if (numPoints > 0)
	{
		for (int idx = 1; idx < MAX_DEPTH; idx++)
		{
			int color = (unsigned char)(255 * (1.0f - ((float)histogram[idx] / (float)numPoints)));
			histogram[idx] = color;
		}
	}
}

void Kinect::renderImage(unsigned char *buffer, int pitch)
{
	xn::ImageMetaData imageMD;
	const XnUInt8 *image  = NULL;

	if (buffer == 0 || !_gotImage)
		return;

	_image.GetMetaData(imageMD);
	image = imageMD.Data();

	const int resX = imageMD.XRes();
	const int resY = imageMD.YRes();

	const int bpp = FORMAT_BPP[_renderFormat];

	pitch = pitch ? (pitch - resX) * bpp : 0;

	if (pitch == 0 && _renderFormat == RENDER_RGB)
	{
		memcpy(buffer, image, resX * resY * bpp);
		return;
	}

	unsigned char *dst = (unsigned char *)buffer;

	for (int y = 0; y < resX; ++y, dst += pitch)
		for (int x = 0; x < resY; ++x, dst += bpp, image += 3)
		{
			dst[0] = image[0];
			dst[1] = image[1];
			dst[2] = image[2];
			dst[3] = 0xFF;
		}
}

void Kinect::renderDepth(unsigned char *buffer, bool background, int pitch)
{
	static const int COLOR_COUNT = 7;
	static const unsigned char COLORS[COLOR_COUNT][3] = 
	{
		{0xFF, 0xFF, 0xFF},
		{0xFF, 0x00, 0x00},
		{0x00, 0xFF, 0x00},
		{0x00, 0x00, 0xFF},
		{0xFF, 0xFF, 0x00},
		{0x00, 0xFF, 0xFF},
		{0xFF, 0x00, 0xFF},
	};

	assert(_init);

	if (buffer == 0)
		return;

	unsigned int *histogram = _histogram;
	unsigned char *dst = (unsigned char *)buffer;

	xn::SceneMetaData sceneMD;
	_userGen.GetUserPixels(0, sceneMD);

	xn::DepthMetaData depthMD;
	_depth.GetMetaData(depthMD);

    const XnDepthPixel *depthPixels = depthMD.Data();
	const XnLabel *labelPixels = sceneMD.Data();

	const int resolutionX = depthMD.XRes();
	const int resolutionY = depthMD.YRes();

	const int bpp = FORMAT_BPP[_renderFormat];

	const unsigned char bg_masks[2] = {0x0, 0xFF};
	const unsigned char force_mask = background ? 0xFF : 0x0;

	pitch = pitch ? (pitch - resolutionX) * bpp : 0;

	calculateHistogram(resolutionX, resolutionY, depthPixels);

	for (int y = 0; y < resolutionY; y++)
	{
		for (int x = 0; x < resolutionX; x++)
		{
            unsigned int val = *depthPixels++ & DEPTH_MASK;
            int c = *labelPixels++ % COLOR_COUNT;

			unsigned char mask = bg_masks[(int)(c > 0)] | force_mask;
			unsigned char pixel = histogram[val] & mask;
            
            dst[0] = pixel & COLORS[c][0];
            dst[1] = pixel & COLORS[c][1];
            dst[2] = pixel & COLORS[c][2];
            dst[3] = mask;

			dst += bpp;
		}

		dst += pitch;
	}
}

void Kinect::updateUserData(XnUserID id, Kinect_UserData *data)
{
	for (int i = 0; i < JOINT_COUNT; ++i) 
	{
		_userGen.GetSkeletonCap().GetSkeletonJoint(id, (XnSkeletonJoint)i, data->world_joints[i]);
		memcpy(&data->screen_joints[i], &data->world_joints[i].position, sizeof(XnPoint3D));
	}

	_depth.ConvertRealWorldToProjective(JOINT_COUNT, data->screen_joints, data->screen_joints);
	_userGen.GetCoM(id, data->world_com);
}

int Kinect::userStatus(XnUserID id)
{
	return (id < MAX_USERS) ? _userStatus[id] : USER_INACTIVE;
}

char const* Kinect::errorMessage()
{
	return (_error != XN_STATUS_OK) ? xnGetStatusString(_error) : 0;
}

const XnPoint3D *Kinect::getJoint(int joint, bool screen_position, XnUserID id)
{
	if (id < MAX_USERS && _userStatus[id] != USER_INACTIVE)
	{
		return screen_position ? 
			&(_userData[id]->screen_joints[joint]) : 
			&(_userData[id]->world_joints[joint].position.position);
	}

	return 0;
}

const XnPoint3D *Kinect::getCoM(XnUserID id)
{
	if (id < MAX_USERS && _userStatus[id] != USER_INACTIVE)
	{
		return &(_userData[id]->world_com);
	}

	return 0;
}

void Kinect::setEventCallback(Callback callback, void *userData)
{
	_eventCallback = callback;
	_callbackData = userData;
}

void Kinect::setTicksPerSecond(int ticksPerSecond)
{
	_tickTime = ticksPerSecond ? 1000 / ticksPerSecond : 0;
}

#ifdef _WIN32
DWORD WINAPI _Kinect_Thread(LPVOID ptr)
{
	Kinect *k = (Kinect *)ptr;
	for (;;) k->tick();
	return 0;
}

XnStatus Kinect::runThreaded()
{
	_thread = CreateThread(NULL, 0, _Kinect_Thread, (void *)this, 0, 0);
	return (_thread != 0) ? XN_STATUS_OK : XN_STATUS_OS_INVALID_THREAD;
}

void Kinect::waitForThread(int timeout)
{
	if (_thread != 0)
	{
		if (WaitForSingleObject(_thread, timeout) == 0)
			_thread = 0;
	}
}

void Kinect::stopThread()
{
	if (_thread != 0)
	{
    	TerminateThread(_thread, 0);
		_thread = 0;
	}
}

bool Kinect::isThreaded()
{
	return (_thread != 0);
}
#else
XnStatus Kinect::runThreaded()
{
	return XN_STATUS_ERROR;
}

void Kinect::waitForThread(int timeout)
{
}

void Kinect::stopThread()
{
}

bool Kinect::isThreaded()
{
	return false;
}
#endif

unsigned int Kinect::getImageTexSize(int pitch /*= 0*/)
{
	XnUInt32XYPair res = getImageResolution();
	int bpp = FORMAT_BPP[_renderFormat];
	return pitch ? (pitch * res.Y * bpp) : (res.X * res.Y * bpp);
}

unsigned int Kinect::getDepthTexSize(int pitch /*= 0*/)
{
	XnUInt32XYPair res = getDepthResolution();
	int bpp = FORMAT_BPP[_renderFormat];
	return pitch ? (pitch * res.Y * bpp) : (res.X * res.Y * bpp);
}

XnUInt32XYPair Kinect::getDepthResolution()
{
	XnMapOutputMode output;
	XnUInt32XYPair resolution;

	_depth.GetMapOutputMode(output);
	resolution.X = output.nXRes;
	resolution.Y = output.nYRes;

	return resolution;
}

XnUInt32XYPair Kinect::getImageResolution()
{
	XnMapOutputMode output;
	XnUInt32XYPair resolution;

	_image.GetMapOutputMode(output);
	resolution.X = output.nXRes;
	resolution.Y = output.nYRes;

	return resolution;
}

void Kinect::setRenderFormat(RenderFormat format)
{
	_renderFormat = format;
}



