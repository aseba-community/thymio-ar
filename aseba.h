#ifndef ASEBA_CLIENT_H
#define ASEBA_CLIENT_H

#include <QThread>
#include "dashel/dashel.h"
#include "aseba/common/msg/msg.h"
#include "aseba/common/msg/descriptions-manager.h"

class DashelHub: public QObject, public Dashel::Hub {
	Q_OBJECT
public slots:
	void start(QString target);
signals:
	void connectionCreated(Dashel::Stream* stream) Q_DECL_OVERRIDE;
	void incomingData(Dashel::Stream* stream) Q_DECL_OVERRIDE;
	void connectionClosed(Dashel::Stream* stream, bool abnormal) Q_DECL_OVERRIDE;
	void error(QString source, QString reason);
};

class AsebaDescriptionsManager: public QObject, public Aseba::DescriptionsManager {
	Q_OBJECT
signals:
	void nodeProtocolVersionMismatch(unsigned nodeId, const std::wstring &nodeName, uint16 protocolVersion) Q_DECL_OVERRIDE;
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
	void userMessage(unsigned type, QList<int> data);
	void nodesChanged();
	void connectionError(QString source, QString reason);
};

class AsebaNode: public QObject {
	Q_OBJECT
	Q_PROPERTY(QString name READ name CONSTANT)
	unsigned nodeId;
	Aseba::TargetDescription description;
	Aseba::VariablesMap variablesMap;
public:
	explicit AsebaNode(AsebaClient* parent, unsigned nodeId, const Aseba::TargetDescription* description);
	AsebaClient* parent() const { return static_cast<AsebaClient*>(QObject::parent()); }
	QString name() { return QString::fromStdWString(description.name); }
public slots:
	void setVariable(QString name, QList<int> value);
	void setProgram(QString aesl);
};

#endif // ASEBA_CLIENT_H
