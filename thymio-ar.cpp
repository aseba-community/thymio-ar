#include "thymio-ar.h"
#include "vision-video-filter.h"
#include <QtQml>

void thymioARInit() {
	qmlRegisterType<Landmark>("ThymioAR", 1, 0, "Landmark");
	qmlRegisterType<VisionVideoFilter>("ThymioAR", 1, 0, "VisionVideoFilter");
}
