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
	Q_PROPERTY(const bool robotFound MEMBER robotFound NOTIFY updated)
	Q_PROPERTY(const QMatrix4x4 robotPose MEMBER robotPose NOTIFY updated)
public:
    explicit VisionVideoFilter(QObject* parent = 0);
	QVideoFilterRunnable* createFilterRunnable();
	Q_INVOKABLE QMatrix4x4 landmarkPose(int index);
	QRotationSensor sensor;

	QStringList landmarkFileNames;
	float updatesPerSecond;
	bool robotFound;
	QMatrix4x4 robotPose;
	QList<QMatrix4x4> landmarkPoses;
signals:
	void updated();
};

#endif // VISION_H
