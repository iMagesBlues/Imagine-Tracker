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
static cv::Mat webcamImage, gray;

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

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API DebugShowTexture()
{
    cv::imshow("Gray Image", gray);
}

// --------------------------------------------------------------------------
// Build And Load ImageTargets

extern "C" int UNITY_INTERFACE_API BuildImageTargetDatabase(Color32* img, int width, int height, const char* imgName, char* data, int* size)
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
    imageTarget.ExportDatabase(data, size);
    
    return 0;
}
extern "C" int UNITY_INTERFACE_EXPORT InitImageTarget(char* data, int size){

    imageTarget.ImportDatabase(data, size);
    
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
    tracker.findPattern(gray);
    //tracker.m_trackingInfo.draw2dContour(gray, CV_RGB(0,200,0));
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
