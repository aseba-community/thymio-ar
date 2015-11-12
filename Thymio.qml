import QtQuick 2.0
import ThymioAR 1.0

Item {
    property var node: {
        for (var i = 0; i < aseba.nodes.length; ++i) {
            var node = aseba.nodes[i];
            if (node.name === "thymio-II") {
                return node;
            }
        }
    }
    onNodeChanged: {
        console.warn(node);
    }
}
