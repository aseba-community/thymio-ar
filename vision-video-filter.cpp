#include "vision-video-filter.h"

#include <atomic>
#include <QDebug>
#include <QFile>
#include <QThread>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <opencv2/core.hpp>

#include "thymio-tracker/src/ThymioTracker.h"

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
	explicit Tracker(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing);
	void send(QVideoFrame* frame);
public slots:
	void step();
private:
	struct Conversion {
		int inputType;
		cv::ColorConversionCodes conversion;
		Conversion(int inputType, cv::ColorConversionCodes conversion): inputType(inputType), conversion(conversion) {}
	};
	static Conversion conversionFromPixelFormat(QVideoFrame::PixelFormat);
	VisionVideoFilter* filter;
	TripleBuffer<cv::Mat> buffers;
	QVideoFrame* free;
	thymio_tracker::ThymioTracker tracker;
};

Tracker::Tracker(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing)
		: filter(filter), tracker(calibration, geomHashing) {
}

void Tracker::send(QVideoFrame* inputFrame) {
	inputFrame->map(QAbstractVideoBuffer::ReadOnly);

	auto pixelFormat(inputFrame->pixelFormat());
	auto conversion(conversionFromPixelFormat(pixelFormat));

	auto bits(inputFrame->bits());
	auto bytesPerLine(inputFrame->bytesPerLine());
	auto lines(inputFrame->mappedBytes() / bytesPerLine);
	auto width(inputFrame->width());

	//qWarning() << bytesPerLine << lines << width << height;

	cv::Mat inputMat(lines, width, conversion.inputType, bits, bytesPerLine);
	//qWarning() << inputMat.type() << inputMat.cols << inputMat.rows << inputMat.step;

	auto& buffer(buffers.writeBuffer());

	cv::cvtColor(inputMat, buffer, conversion.conversion);
	//qWarning() << buffer.type() << buffer.cols << buffer.rows << buffer.step;

	inputFrame->unmap();

	if (!buffers.writeSwap()) {
		QMetaObject::invokeMethod(this, "step", Qt::QueuedConnection);
	}
}

Tracker::Conversion Tracker::conversionFromPixelFormat(QVideoFrame::PixelFormat pixelFormat) {
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

void Tracker::step() {
	buffers.readSwap();
	tracker.update(buffers.readBuffer());

	auto& detection(tracker.getDetectionInfo());
	auto robotFound(detection.robotFound);
	filter->robotFound = robotFound;
	if (robotFound) {
		const auto& doubles(detection.robotPose.matrix.val);
		float* floats(filter->robotPose.data());
		for (size_t i = 0; i < sizeof(doubles) / sizeof(double); ++i) {
			floats[i] = doubles[i];
		}
	}
	emit filter->updated();
}




class VisionVideoFilterRunnable : public QVideoFilterRunnable {
public:
	explicit VisionVideoFilterRunnable(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing);
	~VisionVideoFilterRunnable();
	QVideoFrame run(QVideoFrame* input, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags);
private:
	QThread thread;
	Tracker tracker;
};

VisionVideoFilterRunnable::VisionVideoFilterRunnable(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing)
		: tracker(filter, calibration, geomHashing) {
	tracker.moveToThread(&thread);
	thread.start();
}

VisionVideoFilterRunnable::~VisionVideoFilterRunnable() {
	thread.quit();
	thread.wait();
}

QVideoFrame VisionVideoFilterRunnable::run(QVideoFrame* input, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags) {
	Q_UNUSED(surfaceFormat);
	Q_UNUSED(flags);

	tracker.send(input);

	return *input;
}




VisionVideoFilter::VisionVideoFilter(QObject* parent)
		: QAbstractVideoFilter(parent), robotFound(false) {
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
