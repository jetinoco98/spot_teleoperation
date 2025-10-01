#pragma once

// OpenGL
#include <GL/glew.h>

// SDL
#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_syswm.h>

// Oculus SDK
#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Local includes
#include "Shader.hpp"
#include "StreamCapture.hpp"


class OculusRenderer {
public:
    OculusRenderer(ovrSession& session);
    ~OculusRenderer();

    bool initialize(int captureWidth, int captureHeight);
    bool update(VideoCaptureFrameBuffer& buffer);
    void shutdown();
    void setViewportOffset(int offset);

private:
    // Initialization Part 1
    void initializeSDL();
    void initializeGL();

    // Initialization Part 2
    void initializeCaptureTextures();
    void setEyeTextureSizes();
    void createTextureSwapChain();
    void initializeFrameBuffer();
    void initializeMirrorTexture();
    void initializeTracking();
    void initializeRectangleBuffers();
    void setupShaderAttributes();

    // Update 
    void grabFrame(VideoCaptureFrameBuffer& buffer);
    void renderToOculus(VideoCaptureFrameBuffer& buffer);

    // Internal functions
    static void BindCVMat2GLTexture(cv::Mat& image, GLuint& imageTexture);

    // ====== Internal variables ======
    
    // Objects of local libraries
    std::unique_ptr<Shader> shader_;

    // Objects from external libraries (Oculus, SDL, OpenGL, OpenCV)
    ovrSession& session_;
    SDL_Window* window_ = nullptr;
    SDL_GLContext glContext_ = nullptr;
    SDL_Event events_;
    ovrHmdDesc hmdDesc_;
    ovrTextureSwapChain textureChain_ = nullptr;
    ovrMirrorTexture mirrorTexture_ = nullptr;
    GLuint captureTextureID_[2] = {0, 0};
    GLuint fboID_ = 0, depthBuffID_ = 0, mirrorFBOID_ = 0;
    ovrSizei textureSize0_, textureSize1_;
    ovrSizei bufferSize_;
    ovrPosef eyeRenderPose_[2];
    ovrPosef hmdToEyeOffset_[2];
    GLuint rectVBO_[3] = {0, 0, 0};

    // Regular variables and normal types
    int winWidth_ = 0, winHeight_ = 0;
    int captureWidth_ = 0, captureHeight_ = 0;
    long long frameIndex_ = 0;
    double sensorSampleTime_ = 0.0;
    bool initialized_ = false;  // Represents the initialization of the whole class
    bool isVisible_ = true;     // Starts as visible by default
    int viewportOffset_ = 110;  // Default viewport offset value, acceptable range is 80 to 180
};