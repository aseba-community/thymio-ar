#ifndef VISION_H
#define VISION_H

#include <QAbstractVideoFilter>
#include <QList>
#include <QMatrix4x4>
#include <QtSensors/QRotationSensor>

class VisionVideoFilter : public QAbstractVideoFilter {
	Q_OBJECT
	Q_PROPERTY(QStringList landmarkFileNames MEMBER landmarkFileNames)
	Q_PROPERTY(const float updatesPerSecond MEMBER updatesPerSecond NOTIFY updated)
	Q_PROPERTY(const QMatrix4x4 robotPose MEMBER robotPose NOTIFY updated)
	Q_PROPERTY(const QVariantList& landmarkPoses READ getLandmarkPoses NOTIFY updated)
public:
    explicit VisionVideoFilter(QObject* parent = 0);
	QVideoFilterRunnable* createFilterRunnable();
	QRotationSensor sensor;

	QStringList landmarkFileNames;
	float updatesPerSecond;
	QMatrix4x4 robotPose;
	QVariantList landmarkPoses;
	const QVariantList& getLandmarkPoses();
signals:
	void updated();
};

#endif // VISION_H
