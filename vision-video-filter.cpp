#include "vision-video-filter.h"

#include <QDebug>
#include <QOpenGLContext>
#include <QOpenGLFunctions>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

VisionVideoFilter::VisionVideoFilter(QObject* parent) : QAbstractVideoFilter(parent) {
}

class VisionVideoFilterRunnable : public QVideoFilterRunnable {
public:
    explicit VisionVideoFilterRunnable();
    QVideoFrame run(QVideoFrame* input, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags);
private:
    struct Conversion {
        int inputType;
        cv::ColorConversionCodes conversion;
        Conversion(int inputType, cv::ColorConversionCodes conversion): inputType(inputType), conversion(conversion) {}
    };
    static Conversion conversionFromPixelFormat(QVideoFrame::PixelFormat);
    QOpenGLFunctions* glFunctions;
    cv::Mat rgbMat;
    cv::Mat outputMat;
};

QVideoFilterRunnable* VisionVideoFilter::createFilterRunnable() {
    return new VisionVideoFilterRunnable();
}

VisionVideoFilterRunnable::VisionVideoFilterRunnable() {
    QOpenGLContext* context = QOpenGLContext::currentContext();
    glFunctions  = context->functions();
}

QVideoFrame VisionVideoFilterRunnable::run(QVideoFrame* inputFrame, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags) {
    Q_UNUSED(surfaceFormat);
    Q_UNUSED(flags);

    QVideoFrame::PixelFormat pixelFormat = inputFrame->pixelFormat();
    Conversion conversion = conversionFromPixelFormat(pixelFormat);
    QSize size = inputFrame->size();
    int height = size.height();
    int width = size.width();

    int matHeight = height;
    if (pixelFormat == QVideoFrame::Format_YUV420P) {
        matHeight += matHeight / 2;
    }
    int matWidth = width;

    bool map = inputFrame->map(QAbstractVideoBuffer::ReadOnly);
    //qWarning() << *inputFrame << inputFrame->bits() << inputFrame->mappedBytes() << inputFrame->bytesPerLine();

    cv::Mat inputMat;
    if (map) {
        inputMat = cv::Mat(matHeight, matWidth, conversion.inputType, inputFrame->bits(), inputFrame->bytesPerLine());
    } else if (inputFrame->handleType() == QAbstractVideoBuffer::GLTextureHandle) {
        uint textureId = inputFrame->handle().toUInt();
        inputMat = cv::Mat(matHeight, matWidth, conversion.inputType);
        // http://code.qt.io/cgit/qt/qtmultimedia.git/tree/examples/multimedia/video/qmlvideofilter_opencl/rgbframehelper.h
        GLuint prevFbo;
        glFunctions->glGetIntegerv(GL_FRAMEBUFFER_BINDING, reinterpret_cast<GLint*>(&prevFbo));
        GLuint fbo;
        glFunctions->glGenFramebuffers(1, &fbo);
        glFunctions->glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFunctions->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureId, 0);
        glFunctions->glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, inputMat.data);
        glFunctions->glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
    } else {
        qCritical() << inputFrame->handleType();
        qFatal("unmappable handle type");
    }
    //qWarning() << inputMat.type() << inputMat.cols << inputMat.rows << inputMat.step;

    cv::cvtColor(inputMat, rgbMat, conversion.conversion);
    //qWarning() << rgbMat.type() << rgbMat.cols << rgbMat.rows << rgbMat.step;

    if (map) {
        inputFrame->unmap();
    }

    cv::blur(rgbMat, outputMat, cv::Size(width / 64, height / 64));
    //qWarning() << outputMat.type() << outputMat.cols << outputMat.rows << outputMat.step;

    QImage outputImage(outputMat.data, outputMat.cols, outputMat.rows, outputMat.step, QImage::Format_RGBA8888);
    //qWarning() << outputImage << outputImage.bits();
    QVideoFrame outputFrame(outputImage);
    //qWarning() << outputFrame << outputFrame.bits() << outputFrame.mappedBytes() << outputFrame.bytesPerLine();

    return outputFrame;
}

VisionVideoFilterRunnable::Conversion VisionVideoFilterRunnable::conversionFromPixelFormat(QVideoFrame::PixelFormat pixelFormat) {
    switch(pixelFormat) {
    case QVideoFrame::Format_YUV420P:
        return Conversion(CV_8UC1, cv::COLOR_YUV420p2RGBA);
    case QVideoFrame::Format_BGR32:
        return Conversion(CV_8UC4, cv::COLOR_BGRA2RGBA);
    default:
        qCritical() << pixelFormat;
        qFatal("unknown pixel format");
    }
}
