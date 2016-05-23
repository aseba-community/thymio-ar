import QtQuick 2.5
import QtQuick.Scene3D 2.0
import ThymioAR 1.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import QtMultimedia 5.5 as QtMultimedia

Item {
	id: vision

	default property alias data: frameGraph.data

	property alias camera: camera

	property alias landmarkFileNames: filter.landmarkFileNames

	property alias robotPose: filter.robotPose
	property alias landmarkPoses: filter.landmarkPoses

	property alias calibrationRunning: filter.calibrationRunning
	property alias calibrationProgress: filter.calibrationProgress
	property alias calibrationDone: filter.calibrationDone

	readonly property matrix4x4 invalidPose: Qt.matrix4x4()

	QtMultimedia.Camera {
		id: camera
		captureMode: QtMultimedia.Camera.CaptureViewfinder
	}

	QtMultimedia.VideoOutput {
		anchors.fill: parent
		source: camera
		filters: [
			VisionVideoFilter {
				id: filter
			}
		]
		fillMode: QtMultimedia.VideoOutput.PreserveAspectCrop
	}

	Rectangle {
		visible: calibrationRunning

		height: parent.height
		width: parent.height
		opacity: 0.5

		anchors.right: filter.calibrationRight ? parent.right : undefined
		transform: [
			Scale {
				xScale: 1 / parent.height
				yScale: 1 / parent.height
			},
			Matrix4x4 {
				matrix: filter.calibrationTransform
			},
			Scale {
				xScale: parent.height
				yScale: parent.height
			}
		]
	}

	Scene3D {
		anchors.fill: parent

		Entity {
			id: frameGraph
			components: RenderSettings {
				ForwardRenderer {
					viewportRect: Qt.rect(0, 0, 1, 1)
					clearColor: Qt.rgba(0, 0, 0, 0)
					camera: Entity {
						components: [
							CameraLens {
								projectionType: CameraLens.PerspectiveProjection
								fieldOfView: 45
								nearPlane : 0.01
								farPlane : 1000.0
								aspectRatio: vision.width / vision.height
							},
							Transform {
								matrix: landmarkPoses[0]
							}
						]
					}
				}
			}
		}
	}
}
