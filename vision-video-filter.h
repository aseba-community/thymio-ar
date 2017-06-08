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
	Q_PROPERTY(QString identifier MEMBER identifier)
	Q_PROPERTY(const bool found READ found NOTIFY changed)
	Q_PROPERTY(const float confidence READ confidence NOTIFY changed)
#ifdef THYMIO_AR_HOMOGRAPHY
	Q_PROPERTY(const QMatrix4x4 homography READ homography NOTIFY changed)
#endif
	// Pose of the marker relative to the device which detects the marker, e.g. the camera.
	Q_PROPERTY(const QMatrix4x4 pose READ pose NOTIFY changed)

	// Pose of the marker relative to another reference, e.g. another marker.
	Q_PROPERTY(QMatrix4x4 relativePose READ readRelativePose NOTIFY relativePoseUpdated)

	// Stores whether a marker is visible or not.
	Q_PROPERTY(bool visible READ readVisibility NOTIFY visibilityUpdated)

public:
	QString fileName;
	TrackerResult result;
	bool found() { return result.found; }
	float confidence() { return result.confidence; }

	QString identifier;
	QMatrix4x4 relativePose;
	bool visible;

#ifdef THYMIO_AR_HOMOGRAPHY
	const QMatrix4x4& homography() { return result.homography; }
#endif
	const QMatrix4x4& pose() { return result.pose; }

	QMatrix4x4 readRelativePose() { return relativePose; }
	bool readVisibility() { return visible; }

signals:
	void changed();
	void relativePoseUpdated();
	void visibilityUpdated();
};

class VisionVideoFilter : public QAbstractVideoFilter {
	Q_OBJECT

	Q_PROPERTY(Landmark* robot READ getRobot)
	Q_PROPERTY(QQmlListProperty<Landmark> landmarks READ getLandmarks)

	Q_PROPERTY(const QMatrix4x4 lens MEMBER lens NOTIFY updatedCalibration)
	Q_PROPERTY(bool calibrationRunning MEMBER calibrationRunning NOTIFY updatedCalibration)
	Q_PROPERTY(const float calibrationProgress MEMBER calibrationProgress NOTIFY updatedCalibration)
	Q_PROPERTY(const bool calibrationDone MEMBER calibrationDone NOTIFY updatedCalibration)
	Q_PROPERTY(const bool calibrationRight MEMBER calibrationRight NOTIFY updatedCalibration)
	Q_PROPERTY(const QMatrix4x4 calibrationTransform MEMBER calibrationTransform NOTIFY updatedCalibration)
	Q_PROPERTY(const QMatrix4x4 gyroscopeToCameraTransform MEMBER gyroscopeToCameraTransform)
public:
    explicit VisionVideoFilter(QObject* parent = 0);
	QVideoFilterRunnable* createFilterRunnable();
	QRotationSensor sensor;

	Landmark robot;
	QList<Landmark*> landmarks;
	Landmark* getRobot() { return &robot; }
	QQmlListProperty<Landmark> getLandmarks() { return QQmlListProperty<Landmark>(this, landmarks); }

	QMatrix4x4 lens;
	bool calibrationRunning = false;
	float calibrationProgress = 0.0;
	bool calibrationDone = false;
	bool calibrationRight = false;
	QMatrix4x4 calibrationTransform;
	QMatrix4x4 gyroscopeToCameraTransform;
signals:
	void updatedCalibration();
};

#endif // VISION_H
