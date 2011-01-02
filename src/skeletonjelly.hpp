#ifndef __IL_SKELETON_JELLY_H__
#define __IL_SKELETON_JELLY_H__

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#if defined(JELLY_BUILD_DLL)
#	if defined(__GNUC__)
#		define DLL_PUBLIC __attribute__((dllexport))
#	elif defined(_MSC_VER)
#		define DLL_PUBLIC __declspec(dllexport)
#	else
#		define DLL_PUBLIC
#	endif
#else
#	if defined(__GNUC__)
#		define DLL_PUBLIC __attribute__((dllimport))
#	elif defined(_MSC_VER)
#		define DLL_PUBLIC __declspec(dllimport)
#	else
#		define DLL_PUBLIC
#	endif
#endif

#define KINECT_DEFAULT_USER 1
#define KINECT_DEFAULT_WIDTH 640
#define KINECT_DEFAULT_HEIGHT 480
#define KINECT_DEFAULT_FPS 30

#ifndef SWIG
static const int JOINT_COUNT = 25;
struct Kinect_UserData
{
    XnSkeletonJointTransformation world_joints[JOINT_COUNT];
    XnPoint3D screen_joints[JOINT_COUNT];
    XnPoint3D world_com;
};
#endif

class Kinect
{

public:
	enum RenderFormat
	{
		RENDER_RGBA,
		RENDER_RGB
	};

	enum SensorMode
	{
		SENSOR_DISABLED = 0, /* no video */
		SENSOR_QVGA_60FPS, /* 320x240 */
		SENSOR_VGA_30FPS, /* 640x480 */
		SENSOR_SXGA_15FPS /* 1280x1024 */
	};

    enum UserStatus 
    {
		USER_INACTIVE = 0,
    	USER_ACTIVE = (1 << 0),
    	USER_LOOKING_FOR_POSE = (1 << 1),
		USER_GOT_POSE = (1 << 2),
    	USER_CALIBRATING = (1 << 3),
		USER_GOT_CALIBRATION = (1 << 4),
    	USER_TRACKING = (1 << 5),
    };

	enum CallbackType
	{
		CB_NEW_USER,
		CB_LOST_USER,
		CB_POSE_DETECTED,
		CB_CALIBRATION_START,
		CB_CALIBRATION_SUCCESS,
		CB_CALIBRATION_FAIL
	};

	typedef void (*Callback)(Kinect*, CallbackType, XnUserID, void*);


#ifndef SWIG
private:
	static const int MAX_DEPTH = 4096;
	static const int DEPTH_MASK = MAX_DEPTH - 1;
	static const int MAX_USERS = 3;

    friend void XN_CALLBACK_TYPE cb_newUser(xn::UserGenerator& generator, XnUserID nId, void *pCookie);
    friend void XN_CALLBACK_TYPE cb_lostUser(xn::UserGenerator& generator, XnUserID nId, void *pCookie);
    friend void XN_CALLBACK_TYPE cb_poseDetected(xn::PoseDetectionCapability& capability, const XnChar *strPose, XnUserID nId, void *pCookie);
    friend void XN_CALLBACK_TYPE cb_calibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void *pCookie);
    friend void XN_CALLBACK_TYPE cb_calibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void *pCookie);

	xn::Context _context;
	xn::DepthGenerator _depth;
	xn::UserGenerator _userGen;
	xn::ImageGenerator _image;

	XnStatus _error;

	bool _needPose;
	char _calibrationPose[20];

	bool _paused;
	bool _init;
	bool _autoTrack;
	bool _gotImage;

	RenderFormat _renderFormat;
    unsigned int _histogram[MAX_DEPTH]; 
    void calculateHistogram(int resolutionX, int resolutionY, const XnDepthPixel *depth_pixels);

    void onNewUser(XnUserID nId);
    void onLostUser(XnUserID nId);
    void onPoseDetected(const XnChar *strPose, XnUserID nId);
    void onCalibrationStart(XnUserID nId);
    void onCalibrationEnd(XnUserID nId, XnBool bSuccess);

	int _userStatus[MAX_USERS];
	Kinect_UserData *_userData[MAX_USERS];

    void updateUserData(XnUserID id, Kinect_UserData *data);

	Callback _eventCallback;
	void *_callbackData;

	int _tickTime;

#	ifdef _WIN32
	HANDLE _thread;
#	endif

#endif

public:
	Kinect();
	~Kinect();

	void setTicksPerSecond(int ticksPerSecond);
	void tick();

	XnStatus runThreaded();
	void waitForThread(int timeout = INFINITE);
	void stopThread();
	bool isThreaded();

	XnStatus init(SensorMode depthSensor = SENSOR_VGA_30FPS, SensorMode imageSensor = SENSOR_DISABLED);

	XnStatus resetUser(XnUserID id = KINECT_DEFAULT_USER);
	XnStatus trackUser(XnUserID id = KINECT_DEFAULT_USER);
	int userStatus(XnUserID id = KINECT_DEFAULT_USER);

	const XnPoint3D *getJoint(int articulation, bool screen_position, XnUserID id = KINECT_DEFAULT_USER);
	const XnPoint3D *getCoM(XnUserID id = KINECT_DEFAULT_USER);

	void setEventCallback(Callback callback, void *userData);
	char const* errorMessage();

	void renderImage(unsigned char *buffer, int pitch);
	void renderDepth(unsigned char *buffer, bool background, int pitch);

	unsigned int getImageTexSize(int pitch = 0);
	unsigned int getDepthTexSize(int pitch = 0);

	XnUInt32XYPair getDepthResolution();
	XnUInt32XYPair getImageResolution();

	void setRenderFormat(RenderFormat format);
};

#endif
