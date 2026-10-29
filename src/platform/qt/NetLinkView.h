/* Network link cable (Cartucho netlink) dialog.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#pragma once

#include "ui_NetLinkView.h"

#include <QTimer>
#include <memory>

namespace QGBA {

class CoreController;
class Window;

class NetLinkView : public QDialog {
Q_OBJECT

public:
	NetLinkView(Window* window, QWidget* parent = nullptr);

public slots:
	void host();
	void join();
	void detach();

private slots:
	void updateStatus();

private:
	bool prepareController();
	static QString localAddress();

	Ui::NetLinkView m_ui;
	std::shared_ptr<CoreController> m_controller;
	Window* m_window;
	QTimer m_timer;
};

}
