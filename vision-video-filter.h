#ifndef VISION_H
#define VISION_H

#include <QAbstractVideoFilter>
#include <QList>
#include <QMatrix4x4>
#include <QtSensors/QRotationSensor>

class VisionVideoFilter : public QAbstractVideoFilter {
	Q_OBJECT
	Q_PROPERTY(QStringList landmarkFileNames MEMBER landmarkFileNames)
	Q_PROPERTY(const QMatrix4x4 robotPose MEMBER robotPose NOTIFY updatedRobot)
	Q_PROPERTY(const QVariantList& landmarkPoses READ getLandmarkPoses NOTIFY updatedLandmarks)
	Q_PROPERTY(const bool calibrationRunning MEMBER calibrationRunning NOTIFY updatedLandmarks)
	Q_PROPERTY(const float calibrationProgress MEMBER calibrationProgress NOTIFY updatedLandmarks)
public:
    explicit VisionVideoFilter(QObject* parent = 0);
	QVideoFilterRunnable* createFilterRunnable();
	QRotationSensor sensor;

	QStringList landmarkFileNames;
	QMatrix4x4 robotPose;
	QVariantList landmarkPoses;
	const QVariantList& getLandmarkPoses();

	bool calibrationRunning = false;
	float calibrationProgress = 0.0;
signals:
	void updatedRobot();
	void updatedLandmarks();
};

#endif // VISION_H
