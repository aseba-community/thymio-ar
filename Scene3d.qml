import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0

Scene3D {
	id: scene3D
	default property alias data: frameGraph.data
	property matrix4x4 camera

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
							fieldOfView: 35 // FIXME: should this come from calibration?
							nearPlane : 0.01
							farPlane : 10.0
							aspectRatio: scene3D.width / scene3D.height
						},
						Transform {
							matrix: camera
						}
					]
				}
			}
		}
		Entity {

		}
	}
}
