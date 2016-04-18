import QtQuick 2.5
import QtQuick.Scene3D 2.0
import ThymioAR 1.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import QtMultimedia 5.5 as QtMultimedia

Item {
	id: vision

	default property alias data: frameGraph.data
	property alias landmarkFileNames: filter.landmarkFileNames
	property alias robotPose: filter.robotPose
	property alias landmarkPoses: filter.landmarkPoses

	readonly property matrix4x4 invalidPose: Qt.matrix4x4()

	QtMultimedia.VideoOutput {
		anchors.fill: parent
		source: QtMultimedia.Camera {
			captureMode: QtMultimedia.Camera.CaptureViewfinder
		}
		filters: [
			VisionVideoFilter {
				id: filter
			}
		]
		fillMode: QtMultimedia.VideoOutput.PreserveAspectCrop
	}

	Scene3D {
		anchors.fill: parent

		Entity {
			id: frameGraph
			components: FrameGraph {
				Viewport {
					rect: Qt.rect(0, 0, 1, 1)
					clearColor: Qt.rgba(0, 0, 0, 0)
					CameraSelector {
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

						ClearBuffer {
							buffers: ClearBuffer.ColorDepthBuffer
						}
					}
				}
			}
		}
	}
}
