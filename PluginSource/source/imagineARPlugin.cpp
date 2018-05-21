// Example low level rendering Unity plugin

#include "PlatformBase.h"
#include "RenderAPI.h"

#include <assert.h>
#include <math.h>
#include <vector>

#include <glew.h>
#include <opencv2/opencv.hpp>
#include <DebugCPP.hpp>


// --------------------------------------------------------------------------
// SetTextureFromUnity, an example function we export which is called by one of the scripts.

static void* g_TextureHandle = NULL;
static int   g_TextureWidth  = 0;
static int   g_TextureHeight = 0;

static cv::VideoCapture cap;
static cv::Mat webcamImage, gray;
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
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetTextureFromUnity(void* textureHandle, int w, int h)
{
	g_TextureHandle = textureHandle;
	g_TextureWidth = w;
	g_TextureHeight = h;
    
    Debug::Log("init texture ptr");
    
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API DebugShowTexture()
{
    cv::imshow("Webcam Image", webcamImage);
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

static void ModifyTexturePixels()
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
    
//    cap >> webcamImage;
    
//    GLuint gltex = (GLuint)(size_t)(textureDataPtr);
//    glGenTextures(1, &gltex);
//    glBindTexture(GL_TEXTURE_2D, gltex);
//
//    Debug::Log("Bind Texture");
//
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//
//    // Set texture clamping method
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//
//    Debug::Log("Texture Params");
//
//    glTexImage2D(   GL_TEXTURE_2D,     // Type of texture
//                    0,                 // Pyramid level (for mip-mapping) - 0 is the top level
//                    GL_RGBA,            // Internal colour format to convert to
//                    webcamImage.cols,          // Image width  i.e. 640 for Kinect in standard mode
//                    webcamImage.rows,          // Image height i.e. 480 for Kinect in standard mode
//                    0,                 // Border width in pixels (can either be 1 or 0)
//                    GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
//                    GL_UNSIGNED_BYTE,  // Image data type
//                    webcamImage.ptr());        // The actual image data itself
//
//    Debug::Log("Done glTexImage");
    
    //end conversion
   s_CurrentAPI->EndModifyTexture(textureHandle, width, height, textureRowPitch, textureDataPtr);
}


static void UNITY_INTERFACE_API OnRenderEvent(int eventID)
{
    Debug::Log("OnRenderEvent rcv");

	// Unknown / unsupported graphics device type? Do nothing
	if (s_CurrentAPI == NULL)
		return;
    
	ModifyTexturePixels();
}


// --------------------------------------------------------------------------
// GetRenderEventFunc, an example function we export which is used to get a rendering event callback function.

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc()
{
	return OnRenderEvent;
}

