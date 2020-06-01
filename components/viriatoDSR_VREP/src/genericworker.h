/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include <stdint.h>
#include <qlog/qlog.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#include <CommonBehavior.h>

#include <CameraRGBDSimple.h>
#include <DSRGetID.h>
#include <GenericBase.h>
#include <JoystickAdapter.h>
#include <Laser.h>
#include <OmniRobot.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompCameraRGBDSimple;
using namespace RoboCompDSRGetID;
using namespace RoboCompGenericBase;
using namespace RoboCompJoystickAdapter;
using namespace RoboCompLaser;
using namespace RoboCompOmniRobot;

using TuplePrx = std::tuple<RoboCompDSRGetID::DSRGetIDPrxPtr>;


class GenericWorker : public QMainWindow, public Ui_guiDlg
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	DSRGetIDPrxPtr dsrgetid_proxy;

	virtual void CameraRGBDSimple_getAll(TImage &im, TDepth &dep) = 0;
	virtual void CameraRGBDSimple_getDepth(TDepth &dep) = 0;
	virtual void CameraRGBDSimple_getImage(TImage &im) = 0;
	virtual TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState) = 0;
	virtual LaserConfData Laser_getLaserConfData() = 0;
	virtual TLaserData Laser_getLaserData() = 0;
	virtual void OmniRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void OmniRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void OmniRobot_resetOdometer() = 0;
	virtual void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void OmniRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void OmniRobot_setSpeedBase(float advx, float advz, float rot) = 0;
	virtual void OmniRobot_stopBase() = 0;
	virtual void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data) = 0;

protected:

	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
