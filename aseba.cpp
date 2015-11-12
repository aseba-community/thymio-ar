#include "aseba.h"
#include <memory>

#include <QDebug>
#include <sstream>

void DashelHub::connect(QString target) {
	Dashel::Hub::connect(target.toStdString());
}

void DashelHub::run() {
	Dashel::Hub::run();
}

AsebaClient::AsebaClient() {
	hub.moveToThread(&thread);

	QObject::connect(&hub, &DashelHub::connectionCreated, &hub, [this](Dashel::Stream* stream) {
		this->stream = stream;
		Aseba::GetDescription().serialize(stream);
		stream->flush();
	}, Qt::DirectConnection);

	QObject::connect(&hub, &DashelHub::incomingData, &hub, [this](Dashel::Stream* stream) {
		auto message(Aseba::Message::receive(stream));
		std::unique_ptr<Aseba::Message> ptr(message);
		Q_UNUSED(ptr);

		std::wostringstream dump;
		message->dump(dump);
		qDebug() << "received" << QString::fromStdWString(dump.str());

		switch (message->type) {
		case ASEBA_MESSAGE_DESCRIPTION:
		case ASEBA_MESSAGE_NAMED_VARIABLE_DESCRIPTION:
		case ASEBA_MESSAGE_LOCAL_EVENT_DESCRIPTION:
		case ASEBA_MESSAGE_NATIVE_FUNCTION_DESCRIPTION:
		case ASEBA_MESSAGE_DISCONNECTED:
			manager.processMessage(message);
			break;
		}

		emit this->message(message);
	}, Qt::DirectConnection);

	QObject::connect(&manager, &AsebaDescriptionsManager::nodeDescriptionReceived, this, [this](unsigned nodeId) {
		auto description = manager.getDescription(nodeId);
		nodes.append(new AsebaNode(this, description));
		emit this->nodesChanged();
	});

	thread.start();
}

AsebaClient::~AsebaClient() {
	hub.stop();
	thread.quit();
	thread.wait();
}

void AsebaClient::start(QString target) {
	QMetaObject::invokeMethod(&hub, "connect", Qt::QueuedConnection, Q_ARG(QString, target));
	QMetaObject::invokeMethod(&hub, "run", Qt::QueuedConnection);
}

void AsebaClient::send(Aseba::Message* message) {
	if (stream) {
		message->serialize(stream);
	}
}
