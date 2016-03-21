import QtQuick 2.5
import QtQuick.Scene3D 2.0
import ThymioAR 1.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import QtMultimedia 5.5 as QtMultimedia

Item {
	id: vision

	// https://bugreports.qt.io/browse/QTBUG-26810
	property list<Entity> entities
	default property alias alias: vision.entities

	property alias updatesPerSecond: filter.updatesPerSecond
	property alias robotFound: filter.robotFound
	property alias robotPose: filter.robotPose

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
									matrix: robotPose
								}
							]
						}

						ClearBuffer {
							buffers: ClearBuffer.ColorDepthBuffer
						}
					}
				}
			}

			Component.onCompleted: {
				for (var i = 0; i < entities.length; ++i) {
					var entity = entities[i];
					entity.parent = this;
				}
			}

		}
	}

	Rectangle {
		color: "white"
		width: childrenRect.width
		height: childrenRect.height
		Text {
			text: updatesPerSecond
		}
	}
}
