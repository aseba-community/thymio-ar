import QtQuick 2.5
import ThymioAR 1.0
import QtMultimedia 5.5

Item {
	property alias robotFound: filter.robotFound
	property alias robotPose: filter.robotPose
	Camera {
		id: source
		captureMode: Camera.CaptureViewfinder
	}
	VisionVideoFilter {
		id: filter
	}
	VideoOutput {
		anchors.fill: parent
		source: source
		filters: [ filter ]
		fillMode: VideoOutput.PreserveAspectCrop
	}
}
