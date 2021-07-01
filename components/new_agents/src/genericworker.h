/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <cstdint>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#include <utility>
#endif
#include <ui_mainUI.h>



#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<>;

class ComputeThread : public QThread {
    Q_OBJECT

    int Period;
    std::function<void()> Fn;

    void run() override {

        QTimer timer;
        timer.callOnTimeout(Fn);
        timer.start(Period);

        exec();
        timer.stop();
    }

public:
    explicit ComputeThread(std::function<void()> fn) : Period(100), Fn(std::move(fn))
    {}

    void stopThread()
    {
        QThread::quit();
    }

    //Set the timer Period and starts the thread if it is not started.
    void setPeriod(int period)
    {
        Period = period;
        if (not isRunning())
        {
            start();
        }
    }

};

class GenericWorker : public QMainWindow, public Ui_guiDlg
{
Q_OBJECT
public:
	explicit GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(std::unordered_map<std::string, std::string>& params) = 0;
	QMutex *mutex;

protected:

	int Period;
    std::unique_ptr<ComputeThread> computeThread;

signals:
	void kill();
};

#endif
