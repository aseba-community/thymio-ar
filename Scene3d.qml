import QtQuick 2.6
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

Item {
	default property alias data: frameGraph.data
	property matrix4x4 robotPose
	property matrix4x4 lens
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
								projectionMatrix: lens
							},
							Transform {
								matrix: camera
							}
						]
					}
				}
			}

			// draw a transparent robot at the pose of the actual robot
			Entity {
				components: [
					Mesh {
						meshName: "alphaThymio"
						source: "qrc:/thymio-ar/assets/thymio-body-simplified.obj"
					},
					//CuboidMesh { xExtent: 0.11; yExtent: 0.11; zExtent: 0.06 },
					//CuboidMesh { xExtent: 0.02; yExtent: 0.02; zExtent: 0.10 },
					Material {
						effect: Effect {
							techniques: [
								Technique {
									// our shaders are valid both for OpenGL 2.0 and OpenGL ES 2.0
//									graphicsApiFilter {
//										api: GraphicsApiFilter.OpenGL
//										profile: GraphicsApiFilter.NoProfile
//										majorVersion: 2
//										minorVersion: 0
//									}
									filterKeys: FilterKey {	name: "renderingStyle"; value: "forward" }
									renderPasses: RenderPass {
										shaderProgram: ShaderProgram {
											vertexShaderCode: loadSource("qrc:/thymio-ar/shaders/settransparent.vert")
											fragmentShaderCode: loadSource("qrc:/thymio-ar/shaders/settransparent.frag")
										}
									}
								}
							]
						}
					},
					Transform {
						matrix: robotPose
					}
				]
			}
		}
	}
}
