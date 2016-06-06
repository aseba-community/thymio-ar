#ifndef VISION_H
#define VISION_H

#include <QAbstractVideoFilter>
#include <QList>
#include <QMatrix4x4>
#include <QQmlListProperty>
#include <QtSensors/QRotationSensor>

struct TrackerResult {
	bool found;
	float confidence;
#ifndef NDEBUG
	QMatrix4x4 homography;
#endif
	QMatrix4x4 pose;
};

class Landmark : public QObject {
	Q_OBJECT
	Q_PROPERTY(QString fileName MEMBER fileName)
	Q_PROPERTY(const bool found READ found NOTIFY changed)
	Q_PROPERTY(const float confidence READ confidence NOTIFY changed)
#ifndef NDEBUG
	Q_PROPERTY(const QMatrix4x4 homography READ homography NOTIFY changed)
#endif
	Q_PROPERTY(const QMatrix4x4 pose READ pose NOTIFY changed)
public:
	QString fileName;
	TrackerResult result;
	bool found() { return result.found; }
	float confidence() { return result.confidence; }
#ifndef NDEBUG
	const QMatrix4x4& homography() { return result.homography; }
#endif
	const QMatrix4x4& pose() { return result.pose; }
signals:
	void changed();
};

class VisionVideoFilter : public QAbstractVideoFilter {
	Q_OBJECT

	Q_PROPERTY(Landmark* robot READ getRobot)
	Q_PROPERTY(QQmlListProperty<Landmark> landmarks READ getLandmarks)

	Q_PROPERTY(bool calibrationRunning MEMBER calibrationRunning NOTIFY updatedCalibration)
	Q_PROPERTY(const float calibrationProgress MEMBER calibrationProgress NOTIFY updatedCalibration)
	Q_PROPERTY(const bool calibrationDone MEMBER calibrationDone NOTIFY updatedCalibration)
	Q_PROPERTY(const bool calibrationRight MEMBER calibrationRight NOTIFY updatedCalibration)
	Q_PROPERTY(const QMatrix4x4 calibrationTransform MEMBER calibrationTransform NOTIFY updatedCalibration)
public:
    explicit VisionVideoFilter(QObject* parent = 0);
	QVideoFilterRunnable* createFilterRunnable();
	QRotationSensor sensor;

	Landmark robot;
	QList<Landmark*> landmarks;
	Landmark* getRobot() { return &robot; }
	QQmlListProperty<Landmark> getLandmarks() { return QQmlListProperty<Landmark>(this, landmarks); }

	bool calibrationRunning = false;
	float calibrationProgress = 0.0;
	bool calibrationDone = false;
	bool calibrationRight = false;
	QMatrix4x4 calibrationTransform;
signals:
	void updatedCalibration();
};

#endif // VISION_H
