import QtQuick 2.0
import ThymioAR 1.0
import QtMultimedia 5.4

Item {
    Component.onCompleted: {
        source.start()
    }
    Camera {
        id: source
        captureMode: Camera.CaptureViewfinder
    }
    VisionVideoFilter {
        id: visionVideoFilter
    }
    VideoOutput {
        anchors.fill: parent
        source: source
        filters: [ visionVideoFilter ]
    }
}
