// Example low level rendering Unity plugin

#include "PlatformBase.h"
#include "RenderAPI.h"

#include <assert.h>
#include <math.h>
#include <vector>

#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include <opencv2/opencv.hpp>
#include "DebugCPP.hpp"
#include "ImageTarget.hpp"
#include "Tracker.hpp"
#include "ARUtils.hpp"


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
static double deltaTime;
static bool foundInLastFrame = false;
static bool found = false;

struct Color32
{
    uchar r;
    uchar g;
    uchar b;
    uchar a;
};

//GL
//static void InitGL(){ // we use openGL to quickly convert opencvMat to openGL textures much faster
//    GLFWwindow *window;
//    //initialize library
//    if(!glfwInit())
//    {
//        glfwTerminate();
//    }
//    //Create windowed mode and OpenGL Context
//    window = glfwCreateWindow(400, 225, "GL Window", NULL, NULL);
//
//
//    if(!window){
//        glfwTerminate();
//        return  -1;
//    }
//    //make window context current
//    glfwMakeContextCurrent(window);
//
//    GLuint PixelFormat;
//    static PIXELFORMATDESCRIPTOR pfd;
//    hDC = GetDC(NULL);
//    PixelFormat = ChoosePixelFormat(hDC, &pfd);
//    SetPixelFormat(hDC, PixelFormat, &pfd);
//    hRC = wglCreateContext(hDC);
//    wglMakeCurrent(hDC, hRC);
//}
//static void TerminateGL(){
//    //glfwTerminate();
//}


// ------------ WEBCAM FUNCTIONS ----------------
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API OpenWebcam(int* w, int* h)
{
    //InitGL();
    
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
    
    /*stringstream ss;
    ss << "init train imagetarget name: " << imageTarget.name << ".jpg\n";
    Debug::Log(ss);
    trainImg = cv::imread("Assets/Imagetargets/" + imageTarget.name + ".jpg");
    ARUtils::Resize(trainImg, trainImg);*/
     
    Size min = ARUtils::GetScaledSize(Size(webcamImage.cols, webcamImage.rows));
    cameraCalibration =  CameraCalibration(min.width, min.width, min.width / 2, min.height / 2);

    
    return;
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API CloseWebcam()
{
    cap.release();
    Debug::Log("Video capture released");
    
    //TerminateGL();
    
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
    cv::Mat gray2;
    gray.copyTo(gray2);
    
    cv::drawMatches( gray2, tracker.m_queryKeypoints, trainImg, imageTarget.keypoints,
                    tracker.m_matches, debugMatches, Scalar(0,255,0), Scalar(0,0,255),
                    vector<char>(), DrawMatchesFlags::DEFAULT );
    
    if(found){
        tracker.m_trackingInfo.drawRawOutline(debugMatches, Scalar(0,255,255));
        tracker.m_trackingInfo.showAxes(cameraCalibration, tracker.m_trackingInfo.raw_pose3d, debugMatches);
        
        if(!tracker.m_trackingInfo.kf_homography.empty() && tracker.m_trackingInfo.kf_has_prediction)
        {
            tracker.m_trackingInfo.drawKalmanOutline(debugMatches);
            /*cv:Mat kf_warped;
            cv::warpPerspective(gray, kf_warped, tracker.m_trackingInfo.kf_homography, tracker.m_trackingInfo.kf_imagetarget.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
            
            cv::imshow("kf_warped", kf_warped);*/
        }

    }
    
    cv::imshow("Debug", debugMatches);
    
    /*if(!tracker.m_warpedImg.empty())
        cv::imshow("warped", tracker.m_warpedImg)*/;

    ////
    
    
}

// --------------------------------------------------------------------------
// Build And Load ImageTargets


extern "C" int UNITY_INTERFACE_API BuildImageTargetDatabase(Color32* img, int width, int height, const char* imgName)
{
    //TODO: Pass image path and load from plugin for better results eg. pandatest
    
    cv::Mat imgMat(height, width, CV_8UC4, img);
    
    //scale image
    cv::Mat scaledMat;
    ARUtils::Resize(imgMat, scaledMat);
    
    //create new ImageTarget
    ImageTarget imageTarget;
    imageTarget.BuildFromImage(scaledMat, imgName);
    imageTarget.ExportDatabase();
    
    return 0;
}
extern "C" int UNITY_INTERFACE_EXPORT InitImageTarget(const char* imgName){

    ////////////////
    cv::Mat img;
    stringstream ss1;
    ss1 <<"Assets/Imagetargets/" << imgName << ".jpg";
    img = cv::imread(ss1.str());
    
    ARUtils::Resize(img, img);
    
    ImageTarget imagetarget;
    imagetarget.BuildFromImage(img, imgName);
    imagetarget.ExportDatabase();
    ///////////////
    
    imageTarget.ImportDatabase(imgName);
    
    Debug::Log("imported imagetarget data");
    Debug::Log(imageTarget.descriptors.rows);
    Debug::Log(imageTarget.descriptors.cols);
    
    stringstream ss;
    ss << "Assets/Imagetargets/" << imgName << ".jpg";
    Debug::Log(ss);
    trainImg = cv::imread(ss.str());
    ARUtils::Resize(trainImg, trainImg);
    
    tracker.train(imageTarget);
    Debug::Log("init tracker done");
    
    //init kalman
    tracker.m_trackingInfo.initKalman(imageTarget, cameraCalibration);
    
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


static GLuint MatToTex(cv::Mat& image)
{
    GLuint tex = NULL;
    
    glGenFramebuffers(1, &tex);
    glBindFramebuffer(GL_TEXTURE_2D, tex);

    
    if(image.empty()){
        std::cout << "image empty" << std::endl;
    }else{
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        
        // Set texture clamping method
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        
        cv::cvtColor(image, image, CV_RGB2BGR);
        
        glTexImage2D(GL_TEXTURE_2D,         // Type of texture
                     0,                   // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,              // Internal colour format to convert to
                     image.cols,          // Image width
                     image.rows,          // Image height
                     0,                   // Border width in pixels (can either be 1 or 0)
                     GL_RGB,              // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,    // Image data type
                     image.ptr());        // The actual image data itself
    }
    
    return tex;
}


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
    
    //cap >> webcamImage;

    /*unsigned char* dst = (unsigned char*)textureDataPtr;
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
    }*/
    textureDataPtr = webcamImage.data;
    
   s_CurrentAPI->EndModifyTexture(textureHandle, width, height, textureRowPitch, textureDataPtr);
}




static void UNITY_INTERFACE_API OnRenderEvent(int eventID)
{
    Debug::Log("OnRenderEvent rcv");

	// Unknown / unsupported graphics device type? Do nothing
	if (s_CurrentAPI == NULL)
		return;
    cap >> webcamImage;

    
    
    //process frame
    ARUtils::Resize(webcamImage, gray);
    if(!tracker.homographyFoundInLastFrame)
        ARUtils::GetGraySharp(gray, gray);
    else
        ARUtils::GetGraySharp(gray, gray);
    
    //track imagetarget
    found = tracker.findPattern(gray);
    
    //Found Target
    if(found && !foundInLastFrame)
    {
        if(imageTargetFound != nullptr){
            imageTargetFound();
        }
    }
    //Lost Target
    else if(!found && foundInLastFrame)
    {
        if(imageTargetLost != nullptr){
            imageTargetLost();
        }
    }
    
    
    //Target Tracked
    if(found){
        tracker.m_trackingInfo.computeRawPose(imageTarget, cameraCalibration);
    }
    
    tracker.m_trackingInfo.updateKalman(deltaTime);
    
    if(found){
        if (imageTargetTracked != nullptr){
            imageTargetTracked(tracker.m_trackingInfo.kf_pose3d.getMat44().data);
        }
    }

    foundInLastFrame = found;
    
    RenderWebcamTexture();


}


// --------------------------------------------------------------------------
// GetRenderEventFunc, an example function we export which is used to get a rendering event callback function.

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc(double dT)
{
    deltaTime = dT;
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



