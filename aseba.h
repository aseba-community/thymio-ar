#ifndef ASEBA_CLIENT_H
#define ASEBA_CLIENT_H

#include <QThread>
#include "dashel/dashel.h"
#include "aseba/common/msg/descriptions-manager.h"

class DashelHub: public QObject, public Dashel::Hub {
	Q_OBJECT
public slots:
	void connect(QString target);
	void run();
signals:
	void connectionCreated(Dashel::Stream* stream) Q_DECL_OVERRIDE;
	void incomingData(Dashel::Stream* stream) Q_DECL_OVERRIDE;
	void connectionClosed(Dashel::Stream* stream, bool abnormal) Q_DECL_OVERRIDE;
};

class AsebaDescriptionsManager: public QObject, public Aseba::DescriptionsManager {
	Q_OBJECT
signals:
	void nodeProtocolVersionMismatch(const std::wstring &nodeName, uint16 protocolVersion) Q_DECL_OVERRIDE;
	void nodeDescriptionReceived(unsigned nodeId) Q_DECL_OVERRIDE;
};

class AsebaNode;

class AsebaClient: public QObject {
	Q_OBJECT
	Q_PROPERTY(const QList<QObject*> nodes MEMBER nodes NOTIFY nodesChanged)
	QThread thread;
	DashelHub hub;
	Dashel::Stream* stream;
	AsebaDescriptionsManager manager;
	QList<QObject*> nodes;
public:
	AsebaClient();
	~AsebaClient();
public slots:
	void start(QString target = ASEBA_DEFAULT_TARGET);
	void send(Aseba::Message*);
signals:
	void message(Aseba::Message*);
	void nodesChanged();
};

class AsebaNode: public QObject {
	Q_OBJECT
	Q_PROPERTY(QString name READ name CONSTANT)
	Aseba::TargetDescription description;
public:
	explicit AsebaNode(AsebaClient* parent, const Aseba::TargetDescription* description): QObject(parent), description(*description) {}
	QString name() { return QString::fromStdWString(description.name); }
};

#endif // ASEBA_CLIENT_H
