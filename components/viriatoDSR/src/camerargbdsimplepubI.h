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
#ifndef CAMERARGBDSIMPLEPUB_H
#define CAMERARGBDSIMPLEPUB_H

// Ice includes
#include <Ice/Ice.h>
#include <CameraRGBDSimplePub.h>

#include <config.h>
#include "genericworker.h"


class CameraRGBDSimplePubI : public virtual RoboCompCameraRGBDSimplePub::CameraRGBDSimplePub
{
public:
	CameraRGBDSimplePubI(GenericWorker *_worker);
	~CameraRGBDSimplePubI();

	void pushRGBD(RoboCompCameraRGBDSimple::TImage im, RoboCompCameraRGBDSimple::TDepth dep, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
