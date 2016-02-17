#include "vision-video-filter.h"

#include <atomic>
#include <QDebug>
#include <QFile>
#include <QThread>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <opencv2/core.hpp>

#include "thymio-tracker/src/ThymioTracker.h"




const auto outputSize(cv::Size(640, 480));
const auto outputType(CV_8UC1);
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
	explicit Tracker(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing);
	void send(QVideoFrame* frame);
public slots:
	void step();
private:
	VisionVideoFilter* filter;
	cv::Mat resized;
	TripleBuffer<std::pair<cv::Mat, cv::Vec3d>> buffers;
	QVideoFrame* free;
	thymio_tracker::ThymioTracker tracker;
};

Tracker::Tracker(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing)
		: filter(filter), tracker(calibration, geomHashing) {
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

void Tracker::send(QVideoFrame* inputFrame) {
	auto& buffer(buffers.writeBuffer());

	auto& outputMat(buffer.first);
	auto& outputReading(buffer.second);

	auto inputReading(filter->rotation.reading());
	outputReading.val[0] = inputReading ? inputReading->x() : NaN;
	outputReading.val[1] = inputReading ? inputReading->y() : NaN;
	outputReading.val[2] = inputReading ? inputReading->z() : NaN;
	//qWarning() << outputReading.val[0] << outputReading.val[1] << outputReading.val[2];

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

	auto resize(inputMat.size() != outputSize);
	auto convert(cvtCode != cv::COLOR_COLORCVT_MAX);
	if (resize && convert) {
		cv::resize(inputMat, resized, outputSize);
		cv::cvtColor(resized, outputMat, cvtCode, outputType);
	} else if (resize) {
		cv::resize(inputMat, outputMat, outputSize);
	} else if (convert) {
		cv::cvtColor(inputMat, outputMat, cvtCode, outputType);
	} else {
		inputMat.copyTo(outputMat);
	}

	inputFrame->unmap();

	if (!buffers.writeSwap()) {
		QMetaObject::invokeMethod(this, "step", Qt::QueuedConnection);
	}
}

void Tracker::step() {
	buffers.readSwap();
	const auto& buffer(buffers.readBuffer());
	const auto& image(buffer.first);
	const auto& orientation(buffer.second);
	if (std::isnan(orientation[0]) && std::isnan(orientation[1]) && std::isnan(orientation[2])) {
		// nan nan nan, Batman!
		tracker.update(image, nullptr);
	} else {
		auto orientationMat(cv::Mat(orientation, false));
		tracker.update(image, &orientationMat);
	}

	const auto& detection(tracker.getDetectionInfo());
	const auto& robotFound(detection.robotFound);
	const auto& robotPose(detection.robotPose);

	filter->robotFound = robotFound;
	if (robotFound) {
		const auto& val(robotPose.matrix.val);
		filter->robotPose = QMatrix4x4(
			val[0], val[1], val[2], val[3],
			-val[4], -val[5], -val[6], -val[7],
			-val[8], -val[9], -val[10], -val[11],
			val[12], val[13], val[14], val[15]
		);
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
	rotation.start();
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
