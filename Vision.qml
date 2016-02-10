import QtQuick 2.5
import ThymioAR 1.0
import QtMultimedia 5.5

Item {
	Camera {
		id: source
		captureMode: Camera.CaptureViewfinder
	}
	VisionVideoFilter {
		id: visionVideoFilter
		onUpdated: console.warn(robotFound, robotPose)
	}
	VideoOutput {
		anchors.fill: parent
		source: source
		filters: [ visionVideoFilter ]
		fillMode: VideoOutput.PreserveAspectCrop
	}
}
