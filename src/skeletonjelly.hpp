#ifndef __IL_SKELETON_JELLY_H__
#define __IL_SKELETON_JELLY_H__

#include <list>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define KINECT_DEFAULT_USER 1
#define KINECT_DEFAULT_WIDTH 640
#define KINECT_DEFAULT_HEIGHT 480
#define KINECT_DEFAULT_FPS 30
#define KINECT_JOINT_MAX 7

struct KinectUser
{
	int status;

	struct Hand
	{
		std::list<XnPoint3D> history;
		XnPoint3D pos;
		bool tracked;
		float variance;
	} left, right;

	XnSkeletonJointPosition joints[KINECT_JOINT_MAX];
	XnPoint3D centerOfMass;
};

class Kinect
{

public:
	enum Joints
	{
		JOINT_HEAD,
		JOINT_HAND_LEFT,
		JOINT_HAND_RIGHT,
		JOINT_ELBOW_LEFT,
		JOINT_ELBOW_RIGHT,
		JOINT_SHOULDER_LEFT,
		JOINT_SHOULDER_RIGHT
	};

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

private:
	static const int MAX_DEPTH = 4096;
	static const int DEPTH_MASK = MAX_DEPTH - 1;
	static const unsigned int MAX_USERS = 3;

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

	KinectUser *_userData[MAX_USERS];

    void updateUserData(XnUserID id, KinectUser *data);
	void processHand(KinectUser::Hand *hand, XnSkeletonJointPosition *jointWorld, float backPlane, float planeDepth, float xRes, float yRes);

	Callback _eventCallback;
	void *_callbackData;

	int _elapsed;
	int _tickTime;

public:
	Kinect();
	~Kinect();

	void setTicksPerSecond(int ticksPerSecond);
	void tick(int elapsed);

	XnStatus init(SensorMode depthSensor = SENSOR_VGA_30FPS, SensorMode imageSensor = SENSOR_DISABLED);

	XnStatus resetUser(XnUserID id = KINECT_DEFAULT_USER);
	XnStatus trackUser(XnUserID id = KINECT_DEFAULT_USER);

	inline int userStatus(XnUserID id = KINECT_DEFAULT_USER)
	{
		return userActive(id) ? _userData[id]->status : USER_INACTIVE;
	}

	inline bool userActive(XnUserID id = KINECT_DEFAULT_USER)
	{
		return (id < MAX_USERS) && (_userData[id] != 0) && (_userData[id]->status != USER_INACTIVE);
    }

	inline const KinectUser *getUserData(XnUserID id)
	{
		return userActive(id) ? _userData[id] : 0;
	}

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
