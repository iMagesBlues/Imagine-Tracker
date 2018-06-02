// Example low level rendering Unity plugin

#include "PlatformBase.h"
#include "RenderAPI.h"

#include <assert.h>
#include <math.h>
#include <vector>

#include <glew.h>
#include <opencv2/opencv.hpp>
#include <DebugCPP.hpp>
#include <ImageTarget.hpp>
#include <Tracker.hpp>


// --------------------------------------------------------------------------
// SetTextureFromUnity, an example function we export which is called by one of the scripts.

static void* g_TextureHandle = NULL;
static int   g_TextureWidth  = 0;
static int   g_TextureHeight = 0;

static cv::VideoCapture cap;
static cv::Mat webcamImage, gray, debugMatches, trainImg;

static CameraCalibration cameraCalibration;

//static vector<ImageTarget> imageTargets;
//static vector<Tracker> trackers;

static ImageTarget imageTarget;
static Tracker tracker;

struct Color32
{
    uchar r;
    uchar g;
    uchar b;
    uchar a;
};

// ------------ WEBCAM FUNCTIONS ----------------
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API OpenWebcam(int* w, int* h)
{
    cap.open(0);
    if(!cap.isOpened()){
        std::stringstream ss;
        ss << "Failed to open webcam (" << 0 << + ")";
        Debug::Log(ss);
        //Debug::Log("Failed to open webcam");
        return;
    }
    
    cap >> webcamImage;
    *w = webcamImage.cols;
    *h = webcamImage.rows;
    
     cameraCalibration = CameraCalibration(webcamImage.cols, webcamImage.cols, webcamImage.cols / 2, webcamImage.rows / 2);
    
    trainImg = cv::imread("trainImg.jpg");
    int minW = 300;
    int minH = (float)(trainImg.rows * minW) / trainImg.cols;
    cv::resize(trainImg, trainImg, Size(minW, minH));
    
    return;
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API CloseWebcam()
{
    cap.release();
    
    return;
}
//--------------------------------------------------
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetWebcamTexture(void* textureHandle, int w, int h)
{
	g_TextureHandle = textureHandle;
	g_TextureWidth = w;
	g_TextureHeight = h;
    
    Debug::Log("init texture ptr");
    
}

//----Debug-------

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API DebugShowTexture()
{
    
    cv::drawMatches( gray, tracker.m_queryKeypoints, trainImg, imageTarget.keypoints,
                    tracker.m_matches, debugMatches, Scalar(0,255,0), Scalar(0,0,255),
                    vector<char>(), DrawMatchesFlags::DEFAULT );
    
    if(tracker.m_trackingInfo.found){
        tracker.m_trackingInfo.draw2dContour(debugMatches, Scalar(0,255,255));
        tracker.m_trackingInfo.showAxes(cameraCalibration, tracker.m_trackingInfo.pose3d, debugMatches);

    }
    
    cv::imshow("Debug", debugMatches);
    
    if(!tracker.m_warpedImg.empty())
        cv::imshow("warped", tracker.m_warpedImg);
    
}

// --------------------------------------------------------------------------
// Build And Load ImageTargets


extern "C" int UNITY_INTERFACE_API BuildImageTargetDatabase(Color32* img, int width, int height, const char* imgName)
{
    cv::Mat imgMat(height, width, CV_8UC4, img);
    
    //scale image
    cv::Mat scaledMat;
    float maxWidth = 300;
    float aspect = (float)width / height;
    cv::resize(imgMat, scaledMat, Size(maxWidth, maxWidth / aspect));
    
    //create new ImageTarget
    ImageTarget imageTarget;
    imageTarget.BuildFromImage(scaledMat, imgName);
    imageTarget.ExportDatabase();
    
    return 0;
}
extern "C" int UNITY_INTERFACE_EXPORT InitImageTarget(const char* imgName){

    imageTarget.ImportDatabase(imgName);
    
    Debug::Log("imported imagetarget data");
    Debug::Log(imageTarget.descriptors.rows);
    Debug::Log(imageTarget.descriptors.cols);

    return 0;
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API Train()
{
    tracker.train(imageTarget);
    Debug::Log("init tracker done");

}

extern "C"
{
    //Imagetarget Tracked
    typedef void(*ImageTargetTrackedCallback)(float* transformationMat);
    static ImageTargetTrackedCallback imageTargetTracked = nullptr;
    void RegisterImageTargetTrackedCallback(ImageTargetTrackedCallback cb){
        imageTargetTracked = cb;
    }
    
    //Imagetarget Found
    typedef void(*ImageTargetFoundCallback)();
    static ImageTargetFoundCallback imageTargetFound = nullptr;
    void RegisterImageTargetFoundCallback(ImageTargetFoundCallback cb){
        imageTargetFound = cb;
    }
    
    //Imagetarget Lost
    typedef void(*ImageTargetLostCallback)();
    static ImageTargetLostCallback imageTargetLost = nullptr;
    void RegisterImageTargetLostCallback(ImageTargetLostCallback cb){
        imageTargetLost = cb;
    }
}

// --------------------------------------------------------------------------
// UnitySetInterfaces

static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

static IUnityInterfaces* s_UnityInterfaces = NULL;
static IUnityGraphics* s_Graphics = NULL;

extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* unityInterfaces)
{
	s_UnityInterfaces = unityInterfaces;
	s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
	s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);
	
	// Run OnGraphicsDeviceEvent(initialize) manually on plugin load
	OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
{
	s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
}


// --------------------------------------------------------------------------
// GraphicsDeviceEvent


static RenderAPI* s_CurrentAPI = NULL;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;


static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType)
{
	// Create graphics API implementation upon initialization
	if (eventType == kUnityGfxDeviceEventInitialize)
	{
		assert(s_CurrentAPI == NULL);
		s_DeviceType = s_Graphics->GetRenderer();
		s_CurrentAPI = CreateRenderAPI(s_DeviceType);
	}

	// Let the implementation process the device related events
	if (s_CurrentAPI)
	{
		s_CurrentAPI->ProcessDeviceEvent(eventType, s_UnityInterfaces);
	}

	// Cleanup graphics API implementation upon shutdown
	if (eventType == kUnityGfxDeviceEventShutdown)
	{
		delete s_CurrentAPI;
		s_CurrentAPI = NULL;
		s_DeviceType = kUnityGfxRendererNull;
	}
}

// --------------------------------------------------------------------------
// OnRenderEvent
// This will be called for GL.IssuePluginEvent script calls; eventID will
// be the integer passed to IssuePluginEvent. In this example, we just ignore
// that value.

static void RenderWebcamTexture()
{
    Debug::Log("Modify Texture Pixels");
    
    void* textureHandle = g_TextureHandle;
    int width = g_TextureWidth;
    int height = g_TextureHeight;
    if (!textureHandle)
    {
        Debug::Log("null handle");
        return;
    }
    
    int textureRowPitch;
    void* textureDataPtr = s_CurrentAPI->BeginModifyTexture(textureHandle, width, height, &textureRowPitch);
    if (!textureDataPtr)
    {
        Debug::Log("null dataptr");
        return;
    }
    
    cap >> webcamImage;

    unsigned char* dst = (unsigned char*)textureDataPtr;
    for(int y = 0; y < webcamImage.rows; y++)
    {
        unsigned char* ptr = dst;
        for(int x = 0; x < webcamImage.cols; x++)
        {
            cv::Vec3b pixel = webcamImage.at<cv::Vec3b>(y, x);
            
            ptr[0] = pixel.val[2]; //red
            ptr[1] = pixel.val[1];   //green
            ptr[2] = pixel.val[0];   //blue
            ptr[3] = 255; //alpha
            
            ptr += 4;
        }
        dst += textureRowPitch;
    }
    
   s_CurrentAPI->EndModifyTexture(textureHandle, width, height, textureRowPitch, textureDataPtr);
}


static void UNITY_INTERFACE_API OnRenderEvent(int eventID)
{
    Debug::Log("OnRenderEvent rcv");

	// Unknown / unsupported graphics device type? Do nothing
	if (s_CurrentAPI == NULL)
		return;
    
	RenderWebcamTexture();
    
    //track imagetarget
    cv::resize(webcamImage, gray, Size(300,169));
    cv::cvtColor(gray, gray, CV_BGR2GRAY);
    bool found = tracker.findPattern(gray);
    
    //Found Target
    if(found && !tracker.m_trackingInfo.found)
    {
        if(imageTargetFound != nullptr){
            imageTargetFound();
        }
    }
    //Lost Target
    else if(!found && tracker.m_trackingInfo.found)
    {
        if(imageTargetLost != nullptr){
            imageTargetLost();
        }
    }
    //Target Tracked
    if(found){
        if (imageTargetTracked != nullptr){
            tracker.m_trackingInfo.computePose(imageTarget, cameraCalibration);
            imageTargetTracked(tracker.m_trackingInfo.pose3d.getMat44().data);
        }
    }
    tracker.m_trackingInfo.found = found;

}


// --------------------------------------------------------------------------
// GetRenderEventFunc, an example function we export which is used to get a rendering event callback function.

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc()
{
	return OnRenderEvent;
}

// --------------------------------------------------------------------------
// Plugin End

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API End()
{
    std::stringstream ss;
    ss << "End imagineARPlugin";
    Debug::Log(ss);
}



