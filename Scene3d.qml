import QtQuick 2.6
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

Item {
	default property alias data: frameGraph.data
	property matrix4x4 camera
	Scene3D {
		x: cameraRect.x
		y: cameraRect.y
		width: cameraRect.width
		height: cameraRect.height

		Entity {
			id: frameGraph
			components: RenderSettings {
				ForwardRenderer {
					viewportRect: Qt.rect(0, 0, 1, 1)
					clearColor: "transparent"
					camera: Entity {
						components: [
							CameraLens {
								projectionType: CameraLens.PerspectiveProjection
								fieldOfView: 35 // FIXME: should this come from calibration?
								nearPlane : 0.01
								farPlane : 10.0
								aspectRatio: cameraRect.width / cameraRect.height
							},
							Transform {
								matrix: camera
							}
						]
					}
				}
			}
		}
	}
}
