#include "thymio-ar.h"
#include "vision-video-filter.h"
#include "aseba.h"
#include <QtQml>

void thymioARInit() {
	qmlRegisterType<VisionVideoFilter>("ThymioAR", 1, 0, "VisionVideoFilter");
	qmlRegisterType<AsebaClient>("ThymioAR", 1, 0, "AsebaClient");
	qmlRegisterType<AsebaClient>("ThymioAR", 1, 0, "AsebaNode");
}
