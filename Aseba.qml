import QtQuick 2.0
import ThymioAR 1.0

Item {
	property string target: Qt.platform.os === "android" ? "android:" : "tcp:localhost;33333" //"ser:device=/dev/ttyACM0"
	property alias nodes: client.nodes
	signal userMessage(int type, var data)

	AsebaClient {
		id: client
		onConnectionError: {
			timer.start();
		}
	}

	function startClient() {
		client.start(target);
	}

	Component.onCompleted: {
		client.userMessage.connect(userMessage);
		startClient();
	}

	Timer {
		id: timer
		interval: 1000
		onTriggered: startClient()
	}
}
