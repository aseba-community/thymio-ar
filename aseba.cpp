#include "aseba.h"
#include <memory>

#include <QDebug>
#include <sstream>

static const char* exceptionSource(Dashel::DashelException::Source source) {
	switch(source) {
	case Dashel::DashelException::SyncError: return "SyncError";
	case Dashel::DashelException::InvalidTarget: return "InvalidTarget";
	case Dashel::DashelException::InvalidOperation: return "InvalidOperation";
	case Dashel::DashelException::ConnectionLost: return "ConnectionLost";
	case Dashel::DashelException::IOError: return "IOError";
	case Dashel::DashelException::ConnectionFailed: return "ConnectionFailed";
	case Dashel::DashelException::EnumerationError: return "EnumerationError";
	case Dashel::DashelException::PreviousIncomingDataNotRead: return "PreviousIncomingDataNotRead";
	case Dashel::DashelException::Unknown: return "Unknown";
	}
	qFatal("undeclared dashel exception source %i", source);
}

void DashelHub::connect(QString target) {
	try {
		Dashel::Hub::connect(target.toStdString());
	} catch(Dashel::DashelException& e) {
		qFatal("DashelException(%s, %s, %s, %p)", exceptionSource(e.source), strerror(e.sysError), e.what(), e.stream);
	}
}

void DashelHub::run() {
	try {
		Dashel::Hub::run();
	} catch(Dashel::DashelException& e) {
		qFatal("DashelException(%s, %s, %s, %p)", exceptionSource(e.source), strerror(e.sysError), e.what(), e.stream);
	}
}

AsebaClient::AsebaClient() {
	hub.moveToThread(&thread);

	QObject::connect(&hub, &DashelHub::connectionCreated, &hub, [this](Dashel::Stream* stream) {
		this->stream = stream;
		{
			Aseba::GetDescription message;
			send(&message);
		}
		{
			Aseba::ListNodes message;
			send(&message);
		}
	}, Qt::DirectConnection);

	QObject::connect(&hub, &DashelHub::incomingData, &hub, [this](Dashel::Stream* stream) {
		auto message(Aseba::Message::receive(stream));
		std::unique_ptr<Aseba::Message> ptr(message);
		Q_UNUSED(ptr);

		std::wostringstream dump;
		message->dump(dump);
		qDebug() << "received" << QString::fromStdWString(dump.str());

		switch (message->type) {
		case ASEBA_MESSAGE_NODE_PRESENT:
			if (manager.getDescription(message->source) == nullptr) {
				Aseba::GetNodeDescription response(message->source);
				send(&response);
			}
			break;
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
		nodes.append(new AsebaNode(this, nodeId, description));
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
		stream->flush();
	}
}

AsebaNode::AsebaNode(AsebaClient* parent, unsigned nodeId, const Aseba::TargetDescription* description)
	: QObject(parent), nodeId(nodeId), description(*description) {
	unsigned dummy;
	variablesMap = description->getVariablesMap(dummy);
}

void AsebaNode::setVariable(QString name, QList<int> value) {
	uint16 start = variablesMap[name.toStdWString()].first;
	Aseba::SetVariables::VariablesVector variablesVector(value.begin(), value.end());
	Aseba::SetVariables message(nodeId, start, variablesVector);
	parent()->send(&message);
}
