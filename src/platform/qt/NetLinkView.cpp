/* Network link cable (Cartucho netlink) dialog.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "NetLinkView.h"

#include <QMessageBox>
#include <QNetworkInterface>

#include "CoreController.h"
#include "Window.h"

#include <mgba/internal/gba/sio/netlink.h>

using namespace QGBA;

NetLinkView::NetLinkView(Window* window, QWidget* parent)
	: QDialog(parent, Qt::WindowTitleHint | Qt::WindowSystemMenuHint | Qt::WindowCloseButtonHint)
	, m_window(window)
{
	m_ui.setupUi(this);

	connect(window, &QObject::destroyed, this, &QWidget::close);
	connect(m_ui.host, &QAbstractButton::clicked, this, &NetLinkView::host);
	connect(m_ui.join, &QAbstractButton::clicked, this, &NetLinkView::join);
	connect(m_ui.disconnect, &QAbstractButton::clicked, this, &NetLinkView::detach);
	connect(m_ui.fastLink, &QAbstractButton::toggled, this, [this](bool on) {
		std::shared_ptr<CoreController> controller = m_window->controller();
		if (controller) {
			controller->setNetLinkFast(on);
		}
	});
	connect(&m_timer, &QTimer::timeout, this, &NetLinkView::updateStatus);
	m_timer.start(500);

	updateStatus();
}

QString NetLinkView::localAddress() {
	QString fallback;
	for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
		if (!(iface.flags() & QNetworkInterface::IsUp) || (iface.flags() & QNetworkInterface::IsLoopBack)) {
			continue;
		}
		for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
			QHostAddress addr = entry.ip();
			if (addr.protocol() != QAbstractSocket::IPv4Protocol || addr.isLoopback()) {
				continue;
			}
			QString name = iface.name();
			if (name.startsWith("wl") || name.startsWith("en") || name.startsWith("eth")) {
				return addr.toString();
			}
			if (fallback.isEmpty()) {
				fallback = addr.toString();
			}
		}
	}
	return fallback;
}

bool NetLinkView::prepareController() {
	if (!m_window->controller()) {
		return false;
	}
	if (m_window->controller()->platform() != mPLATFORM_GBA) {
		return false;
	}
	m_controller = m_window->controller();
	connect(m_controller.get(), &CoreController::stopping, this, &NetLinkView::detach, Qt::UniqueConnection);
	return true;
}

void NetLinkView::host() {
	if (!prepareController()) {
		QMessageBox::warning(this, tr("Network link cable"), tr("Load a Game Boy Advance game first."));
		return;
	}
	if (!m_controller->hostNetLink(m_ui.hostPort->value())) {
		QMessageBox::warning(this, tr("Network link cable"), tr("Could not open port %1.").arg(m_ui.hostPort->value()));
	}
	updateStatus();
}

void NetLinkView::join() {
	if (!prepareController()) {
		QMessageBox::warning(this, tr("Network link cable"), tr("Load a Game Boy Advance game first."));
		return;
	}
	QString address = m_ui.address->text().trimmed();
	if (address.isEmpty()) {
		return;
	}
	if (!m_controller->joinNetLink(address, m_ui.port->value())) {
		QMessageBox::warning(this, tr("Network link cable"), tr("Could not connect to %1:%2.").arg(address).arg(m_ui.port->value()));
	}
	updateStatus();
}

void NetLinkView::detach() {
	if (m_controller) {
		m_controller->detachNetLink();
		m_controller.reset();
	}
	updateStatus();
}

void NetLinkView::updateStatus() {
	std::shared_ptr<CoreController> controller = m_window->controller();
	GBASIONetLink* link = controller ? controller->netLink() : nullptr;
	bool active = link != nullptr;
	QString text;
	if (!active) {
		text = tr("Not connected");
	} else {
		int id = 0;
		int attached = 1;
		bool connected = false;
		uint32_t transfers = 0;
		GBASIONetLinkStatus(link, &id, &attached, &connected, &transfers);
		uint32_t timeouts = 0;
		uint32_t avgUs = 0;
		uint32_t maxUs = 0;
		GBASIONetLinkStats(link, &timeouts, &avgUs, &maxUs);
		uint32_t fastBlocks = 0;
		uint32_t fastAborts = 0;
		bool fastActive = false;
		GBASIONetLinkFastStats(link, &fastBlocks, &fastAborts, &fastActive);
		if (link->host) {
			text = tr("Hosting on %1:%2 — %3 player(s) — %4 transfers — latency %5 ms (max %6) — %7 timeouts\nFast link: %8 — %9 blocks, %10 aborts")
			           .arg(localAddress()).arg(link->port).arg(attached).arg(transfers).arg(avgUs / 1000).arg(maxUs / 1000).arg(timeouts)
			           .arg(fastActive ? tr("active") : tr("off")).arg(fastBlocks).arg(fastAborts);
		} else if (!connected) {
			text = tr("Connecting to %1:%2…").arg(link->address).arg(link->port);
		} else {
			text = tr("Connected as player %1 — %2 player(s) — %3 transfers").arg(id + 1).arg(attached).arg(transfers);
		}
	}
	m_ui.status->setText(text);
	m_ui.host->setDisabled(active);
	m_ui.join->setDisabled(active);
	m_ui.port->setDisabled(active);
	m_ui.hostPort->setDisabled(active);
	m_ui.address->setDisabled(active);
	m_ui.disconnect->setEnabled(active);
}
