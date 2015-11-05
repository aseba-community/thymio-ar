#ifndef VISION_H
#define VISION_H

#include <QAbstractVideoFilter>

class VisionVideoFilter : public QAbstractVideoFilter {
    Q_OBJECT
public:
    explicit VisionVideoFilter(QObject* parent = 0);
    QVideoFilterRunnable* createFilterRunnable();
};

#endif // VISION_H
