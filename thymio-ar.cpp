#include "thymio-ar.h"
#include "vision-video-filter.h"
#include "thymio-vpl2.h"
#include <QtQml>

void thymioARInit() {
	thymioVPL2Init();
	qmlRegisterType<Landmark>("ThymioAR", 1, 0, "Landmark");
	qmlRegisterType<VisionVideoFilter>("ThymioAR", 1, 0, "VisionVideoFilter");
}
