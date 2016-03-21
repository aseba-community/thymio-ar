#include "vision-video-filter.h"

#include <atomic>
#include <QDebug>
#include <QFile>
#include <QThread>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <opencv2/core.hpp>
#ifdef THYMIO_AR_IMWRITE
#include <opencv2/imgcodecs.hpp>
#endif

#include "thymio-tracker/src/ThymioTracker.h"




const auto outputWidth(640);
const auto outputHeight(480);
const auto NaN(std::numeric_limits<double>::quiet_NaN());




template<typename T>
class TripleBuffer {
public:
	TripleBuffer(): buffers(), next(next_t{0, false}), write(1), read(2) {}

	T& writeBuffer() {
		return buffers[write];
	}
	bool writeSwap() {
		auto next(this->next.exchange(next_t{write, true}));
		write = next.buffer;
		return next.valid;
	}
	bool readSwap() {
		auto next(this->next.exchange(next_t{read, false}));
		read = next.buffer;
		return next.valid;
	}
	T& readBuffer() {
		return buffers[read];
	}

private:
	typedef std::array<T, 3> buffers_t;
	typedef std::uint_fast8_t index_t;
	buffers_t buffers;
	struct next_t {
		std::uint_fast8_t buffer;
		bool valid;
	};
	std::atomic<next_t> next;
	index_t write;
	index_t read;
};




class Tracker : public QObject {
	Q_OBJECT
public:
	explicit Tracker(cv::FileStorage& calibration, std::istream& geomHashing);
	struct Input {
		cv::Vec3d orientation;
		cv::Mat image;
		GLuint framebuffer = 0;
		GLuint renderbuffer = 0;
	};
	struct Output {
		QVector3D rotation;
		bool robotFound;
		QMatrix4x4 robotPose;
	};
	Input& inputBuffer();
	void inputSwap();
	void outputSwap();
	Output& outputBuffer();
private slots:
	void track();
signals:
	void tracked();
private:
	TripleBuffer<Input> inputs;
	TripleBuffer<Output> outputs;
	thymio_tracker::ThymioTracker tracker;
};

Tracker::Tracker(cv::FileStorage& calibration, std::istream& geomHashing): tracker(calibration, geomHashing) {
}

static int getCvType(QVideoFrame::PixelFormat pixelFormat) {
	switch(pixelFormat) {
	case QVideoFrame::Format_ARGB32:
		return CV_8UC4;
	case QVideoFrame::Format_ARGB32_Premultiplied:
		return CV_8UC4;
	case QVideoFrame::Format_RGB32:
		return CV_8UC4;
	case QVideoFrame::Format_RGB24:
		return CV_8UC3;
	case QVideoFrame::Format_RGB565:
		return CV_8UC2;
	case QVideoFrame::Format_RGB555:
		return CV_8UC2;
	case QVideoFrame::Format_ARGB8565_Premultiplied:
		return CV_8UC3;
	case QVideoFrame::Format_BGRA32:
		return CV_8UC4;
	case QVideoFrame::Format_BGRA32_Premultiplied:
		return CV_8UC4;
	case QVideoFrame::Format_BGR32:
		return CV_8UC4;
	case QVideoFrame::Format_BGR24:
		return CV_8UC3;
	case QVideoFrame::Format_BGR565:
		return CV_8UC2;
	case QVideoFrame::Format_BGR555:
		return CV_8UC2;
	case QVideoFrame::Format_BGRA5658_Premultiplied:
		return CV_8UC3;
	case QVideoFrame::Format_YUV420P:
		return CV_8UC1;
	default:
		qCritical() << pixelFormat;
		qFatal("unknown pixel format");
	}
}

static cv::ColorConversionCodes getCvtCode(QVideoFrame::PixelFormat pixelFormat) {
	switch (pixelFormat) {
	case QVideoFrame::Format_BGR32:
		return cv::COLOR_BGR2GRAY;
	case QVideoFrame::Format_YUV420P:
		// no conversion: just take the Y
		return cv::COLOR_COLORCVT_MAX;
	default:
		qCritical() << pixelFormat;
		qFatal("unknown pixel format");
	}
}

Tracker::Input& Tracker::inputBuffer() {
	return inputs.writeBuffer();
}

void Tracker::inputSwap() {
	if (!inputs.writeSwap()) {
		QMetaObject::invokeMethod(this, "track", Qt::QueuedConnection);
	}
}

void Tracker::outputSwap() {
	outputs.readSwap();
}

Tracker::Output& Tracker::outputBuffer() {
	return outputs.readBuffer();
}

void Tracker::track() {
	inputs.readSwap();
	const auto& input(inputs.readBuffer());

	if (std::isnan(input.orientation[0]) && std::isnan(input.orientation[1]) && std::isnan(input.orientation[2])) {
		// nan nan nan, Batman!
		tracker.update(input.image, nullptr);
	} else {
		auto orientationMat(cv::Mat(input.orientation, false));
		tracker.update(input.image, &orientationMat);
	}
	const auto& detection(tracker.getDetectionInfo());

	auto& output(outputs.writeBuffer());

	const auto& orientation(input.orientation.val);
	output.rotation = QVector3D(orientation[0], orientation[1], orientation[2]);

	output.robotFound = detection.robotFound;

	const auto& robotPose(detection.robotPose.matrix.val);
	output.robotPose = QMatrix4x4(
		robotPose[0], robotPose[1], robotPose[2], robotPose[3],
		-robotPose[4], -robotPose[5], -robotPose[6], -robotPose[7],
		-robotPose[8], -robotPose[9], -robotPose[10], -robotPose[11],
		robotPose[12], robotPose[13], robotPose[14], robotPose[15]
	);
	output.robotPose.optimize();

	if (!outputs.writeSwap()) {
		emit tracked();
	}
}




class VisionVideoFilterRunnable : public QVideoFilterRunnable {
public:
	explicit VisionVideoFilterRunnable(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing);
	~VisionVideoFilterRunnable();
	QVideoFrame run(QVideoFrame* input, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags);
private:
	VisionVideoFilter* filter;
	QThread thread;
	Tracker tracker;
	cv::Mat temp1;
	cv::Mat temp2;
	QOpenGLExtraFunctions* gl;
	QOpenGLShaderProgram program;
	GLint imageLocation;
};

VisionVideoFilterRunnable::VisionVideoFilterRunnable(VisionVideoFilter* f, cv::FileStorage& calibration, std::istream& geomHashing)
		: filter(f), tracker(calibration, geomHashing), gl(nullptr) {
	tracker.moveToThread(&thread);

	auto update([this]() {
		const auto& output(tracker.outputBuffer());

		filter->robotFound = output.robotFound;
		filter->robotPose = output.robotPose;

		auto reading(filter->sensor.reading());
		if (reading != nullptr) {
			auto rotation(QVector3D(reading->x(), reading->y(), reading->z()));
			auto diff(output.rotation - rotation);
			auto quaternion(QQuaternion::fromEulerAngles(diff));

			filter->robotPose.rotate(quaternion);
		}

		emit filter->updated();
	});

	QObject::connect(&tracker, &Tracker::tracked, filter, [this, update]() {
		tracker.outputSwap();
		update();
	}, Qt::QueuedConnection);

	QObject::connect(&filter->sensor, &QSensor::readingChanged, filter, update);

	thread.start();
}

VisionVideoFilterRunnable::~VisionVideoFilterRunnable() {
	thread.quit();
	thread.wait();
}

QVideoFrame VisionVideoFilterRunnable::run(QVideoFrame* inputFrame, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags) {
	Q_UNUSED(surfaceFormat);
	Q_UNUSED(flags);

	auto& input(tracker.inputBuffer());

	auto inputReading(filter->sensor.reading());
	if (inputReading != nullptr) {
		input.orientation = cv::Vec3d(inputReading->x(), inputReading->y(), inputReading->z());
	} else {
		input.orientation = cv::Vec3d::all(NaN);
	}
	//qWarning() << outputReading.val[0] << outputReading.val[1] << outputReading.val[2];

	if (inputFrame->handleType() == QAbstractVideoBuffer::HandleType::GLTextureHandle) {

		if (gl == nullptr) {
			auto context(QOpenGLContext::currentContext());
			gl = context->extraFunctions();

			auto version(context->isOpenGLES() ? "#version 300 es\n" : "#version 130\n");

			QString vertex(version);
			vertex += R"(
				out vec2 coord;
				void main(void) {
					int id = gl_VertexID;
					coord = vec2((id << 1) & 2, id & 2);
					gl_Position = vec4(coord * 2.0 - 1.0, 0.0, 1.0);
				}
			)";

			QString fragment(version);
			fragment += R"(
				in lowp vec2 coord;
				uniform sampler2D image;
				const lowp vec3 luma = vec3(0.2126, 0.7152, 0.0722);
				out lowp float fragment;
				void main(void) {
					vec3 color = texture(image, coord).rgb;
					fragment = dot(color, luma);
				}
			)";

			program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex);
			program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment);
			program.link();
			imageLocation = program.uniformLocation("image");
		}

		if (input.renderbuffer == 0) {
			gl->glGenRenderbuffers(1, &input.renderbuffer);
			gl->glBindRenderbuffer(GL_RENDERBUFFER, input.renderbuffer);
			gl->glRenderbufferStorage(GL_RENDERBUFFER, QOpenGLTexture::R8_UNorm, outputWidth, outputHeight);
			gl->glBindRenderbuffer(GL_RENDERBUFFER, 0);
		}

		if (input.framebuffer == 0) {
			gl->glGenFramebuffers(1, &input.framebuffer);
			gl->glBindFramebuffer(GL_FRAMEBUFFER, input.framebuffer);
			gl->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, input.renderbuffer);
			gl->glBindFramebuffer(GL_FRAMEBUFFER, 0);
		}

		gl->glActiveTexture(GL_TEXTURE0);
		gl->glBindTexture(QOpenGLTexture::Target2D, inputFrame->handle().toUInt());

		program.bind();
		program.setUniformValue(imageLocation, 0);
		program.enableAttributeArray(0);
		gl->glBindFramebuffer(GL_FRAMEBUFFER, input.framebuffer);
		gl->glViewport(0, 0, outputWidth, outputHeight);
		gl->glDisable(GL_BLEND);
		gl->glDrawArrays(GL_TRIANGLES, 0, 3);

		input.image.create(outputHeight, outputWidth, CV_8UC1);
		gl->glPixelStorei(GL_PACK_ALIGNMENT, 1);
		gl->glReadPixels(0, 0, outputWidth, outputHeight, QOpenGLTexture::Red, QOpenGLTexture::UInt8, input.image.data);

	} else {

		auto pixelFormat(inputFrame->pixelFormat());
		auto size(inputFrame->size());
		auto height(size.height());
		auto width(size.width());
		//qWarning() << pixelFormat << height << width;

		inputFrame->map(QAbstractVideoBuffer::ReadOnly);

		auto inputType(getCvType(pixelFormat));
		auto cvtCode(getCvtCode(pixelFormat));

		auto bits(inputFrame->bits());
		auto bytesPerLine(inputFrame->bytesPerLine());

		//qWarning() << inputType << bits << bytesPerLine;
		auto inputMat(cv::Mat(height, width, inputType, bits, bytesPerLine));

		const auto outputSize(cv::Size(outputWidth, outputHeight));
		const auto outputType(CV_8UC1);

		auto resize(inputMat.size() != outputSize);
		auto convert(cvtCode != cv::COLOR_COLORCVT_MAX);
		auto flip(surfaceFormat.scanLineDirection() == QVideoSurfaceFormat::BottomToTop || QSysInfo::productType() == "android");

		if (resize && convert && flip) {
			cv::cvtColor(inputMat, temp1, cvtCode, outputType);
			cv::resize(temp1, temp2, outputSize);
			cv::flip(temp2, input.image, 0);
		} else if (resize && convert) {
			cv::cvtColor(inputMat, temp1, cvtCode, outputType);
			cv::resize(temp1, input.image, outputSize);
		} else if (resize && flip) {
			cv::resize(inputMat, temp1, outputSize);
			cv::flip(temp1, input.image, 0);
		} else if (convert && flip) {
			cv::cvtColor(inputMat, temp1, cvtCode, outputType);
			cv::flip(temp1, input.image, 0);
		} else if (resize) {
			cv::resize(inputMat, input.image, outputSize);
		} else if (convert) {
			cv::cvtColor(inputMat, input.image, cvtCode, outputType);
		} else if (flip) {
			cv::flip(inputMat, input.image, 0);
		} else {
			inputMat.copyTo(input.image);
		}

		inputFrame->unmap();

	}

#ifdef THYMIO_AR_IMWRITE
	static bool first = true;
	if (first) {
		qWarning()
				<< input.image.data[0x00] << input.image.data[0x01] << input.image.data[0x02] << input.image.data[0x03]
				<< input.image.data[0x04] << input.image.data[0x05] << input.image.data[0x06] << input.image.data[0x07]
				<< input.image.data[0x08] << input.image.data[0x09] << input.image.data[0x0A] << input.image.data[0x0B]
				<< input.image.data[0x0C] << input.image.data[0x0D] << input.image.data[0x0E] << input.image.data[0x0F]
				<< input.image.data[0x10] << input.image.data[0x11] << input.image.data[0x12] << input.image.data[0x13]
				<< input.image.data[0x14] << input.image.data[0x15] << input.image.data[0x16] << input.image.data[0x17]
				<< input.image.data[0x18] << input.image.data[0x19] << input.image.data[0x1A] << input.image.data[0x1B]
				<< input.image.data[0x1C] << input.image.data[0x1D] << input.image.data[0x1E] << input.image.data[0x1F];
		cv::imwrite(QSysInfo::productType() == "android" ? "/storage/emulated/0/DCIM/100ANDRO/toto.png" : "toto.png", input.image);
	}
	first = false;
#endif

	tracker.inputSwap();

	return *inputFrame;
}




VisionVideoFilter::VisionVideoFilter(QObject* parent)
		: QAbstractVideoFilter(parent), robotFound(false) {
	sensor.start();
}

static std::string readFile(QString path) {
	QFile file(path);
	if (!file.open(QFile::ReadOnly)) {
		qFatal("Cannot open file %s", path.toLocal8Bit().constData());
	}
	return file.readAll().toStdString();
}

QVideoFilterRunnable* VisionVideoFilter::createFilterRunnable() {
	cv::FileStorage calibration(readFile(":/thymio-ar/calibration.xml"), cv::FileStorage::READ | cv::FileStorage::MEMORY);
	std::istringstream geomHashing(readFile(":/thymio-ar/geomHashing.dat"));
	return new VisionVideoFilterRunnable(this, calibration, geomHashing);
}

// This file declares Q_OBJECT classes, so we need to
#include "vision-video-filter.moc"
