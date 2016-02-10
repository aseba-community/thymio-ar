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
	VisionVideoFilter* filter;
	cv::Mat resized;
	TripleBuffer<cv::Mat> buffers;
	QVideoFrame* free;
	thymio_tracker::ThymioTracker tracker;
};

Tracker::Tracker(VisionVideoFilter* filter, cv::FileStorage& calibration, std::istream& geomHashing)
		: filter(filter), tracker(calibration, geomHashing) {
	buffers.writeBuffer().create(480, 640, CV_8UC1);
	buffers.readBuffer().create(480, 640, CV_8UC1);
	buffers.readSwap();
	buffers.readBuffer().create(480, 640, CV_8UC1);
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
		return cv::COLOR_BGRA2GRAY;
	case QVideoFrame::Format_YUV420P:
		// no conversion: just take the Y
		return cv::COLOR_COLORCVT_MAX;
	default:
		qCritical() << pixelFormat;
		qFatal("unknown pixel format");
	}
}

void Tracker::send(QVideoFrame* frame) {
	auto& output(buffers.writeBuffer());

	auto pixelFormat(frame->pixelFormat());
	auto size(frame->size());
	auto height(size.height());
	auto width(size.width());
	//qWarning() << pixelFormat << height << width;

	frame->map(QAbstractVideoBuffer::ReadOnly);

	auto inputType(getCvType(pixelFormat));
	auto cvtCode(getCvtCode(pixelFormat));

	auto bits(frame->bits());
	auto bytesPerLine(frame->bytesPerLine());

	//qWarning() << inputType << bits << bytesPerLine;
	auto input(cv::Mat(height, width, inputType, bits, bytesPerLine));

	auto resize(input.size() != output.size());
	auto convert(cvtCode != cv::COLOR_COLORCVT_MAX);
	if (resize && convert) {
		cv::resize(input, resized, output.size());
		cv::cvtColor(resized, output, cvtCode, output.type());
	} else if (resize) {
		cv::resize(input, output, output.size());
	} else if (convert) {
		cv::cvtColor(input, output, cvtCode, output.type());
	} else {
		input.copyTo(output);
	}

	frame->unmap();

	if (!buffers.writeSwap()) {
		QMetaObject::invokeMethod(this, "step", Qt::QueuedConnection);
	}
}

void Tracker::step() {
	buffers.readSwap();
	auto& buffer(buffers.readBuffer());
	//qWarning() << buffer.type() << buffer.cols << buffer.rows << buffer.step;
	tracker.update(buffer);

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
