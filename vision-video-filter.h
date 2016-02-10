#ifndef VISION_H
#define VISION_H

#include <QAbstractVideoFilter>
#include <QMatrix4x4>
#include <QtSensors/QRotationSensor>

class VisionVideoFilter : public QAbstractVideoFilter {
    Q_OBJECT
	Q_PROPERTY(const bool robotFound MEMBER robotFound NOTIFY updated)
	Q_PROPERTY(const QMatrix4x4 robotPose MEMBER robotPose NOTIFY updated)
public:
    explicit VisionVideoFilter(QObject* parent = 0);
	QVideoFilterRunnable* createFilterRunnable();
	bool robotFound;
	QMatrix4x4 robotPose;
	QRotationSensor rotation;
signals:
	void updated();
};

#endif // VISION_H
