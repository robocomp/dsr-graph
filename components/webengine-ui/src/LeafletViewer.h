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

/**
	\brief
	@author authorname
*/



#ifndef CUSTOMWIDGET_H
#define CUSTOMWIDGET_H

#include <QtWidgets>
//sudo apt-get install  qtwebengine5-dev
#include <QWebEngineView>
#include <QWebChannel>
#include <QWebEngineProfile>
#include <QWebEngineScript>
#include <QWebEngineScriptCollection>

#include <QTimer>

#include "dsr/api/dsr_api.h"

class Position : public QObject{
Q_OBJECT
public:

    float x{0.0}, y {0.0};
    QString name;

    Position() : x{0.0}, y {0.0}, name(""){}
    Position(float x_, float y_, QString name_) : x{x_}, y{y_}, name{std::move(name_)} {}
    Position (const Position& o)
    {
        x = o.x;
        y = o.y;
        name = o.name;
    }
    Position& operator=(const Position& o)
    {
        x = o.x;
        y = o.y;
        name = o.name;

        return *this;
    }

    float getx() const {return x;}
    float gety() const {return y;}
    QString getname_()  { return name; }

    QJsonObject toObject() const{
        QJsonObject obj;
        obj["name"] = name;
        obj["x"] = x;
        obj["y"] = y;
        return obj;
    }

    Q_PROPERTY(float x READ getx );
    Q_PROPERTY(float y READ gety );
    Q_PROPERTY(QString name READ getname_)
};

class PositionMap : public QObject{
Q_OBJECT
public:
    QMap<QString, Position> positions;
    Q_INVOKABLE QJsonObject get_pos (const QString& k) { return positions[k].toObject(); }
signals:
    void pos_changed(QString name);
};


class LeafLetGPSViewer : public  QWebEngineView
{
Q_OBJECT

public:

    explicit LeafLetGPSViewer(DSR::DSRGraph * G_) :
        QWebEngineView(), G(G_)
    {
        Q_INIT_RESOURCE(webpage);

        setContextMenuPolicy(Qt::NoContextMenu);

        com_channel = new QWebChannel(page());
        com_channel->registerObject(QString("ObjectPositions"), &object_positions);
        page()->setWebChannel(com_channel);
        load(QUrl("qrc:/page.html" ));

        //Descomentar esto para hacer debug en js.
        //inspector.setWindowTitle("Web inspector");
        //inspector.load(QUrl("http://127.0.0.1:6666"));

        //connect(this, &LeafLetGPSViewer::loadFinished, this, [this](auto ok) {
        //    page()->setDevToolsPage(inspector.page());
        //    inspector.show();
        //});
        // run bin/webengine_ui etc/config  --remote-debugging-port=6666

        connect(G, &DSR::DSRGraph::update_node_attr_signal, this, [this](auto id, auto attrs) {

            std::optional<float> lat, lon;
            if (std::find(attrs.begin(), attrs.end(), gps_latitude_att::attr_name.data()) != attrs.end()
                or std::find(attrs.begin(), attrs.end(), gps_longitude_att::attr_name.data()) != attrs.end())
            {
                lat = G->get_attrib_by_name<gps_latitude_att>(id);
                lon = G->get_attrib_by_name<gps_longitude_att>(id);

                if (lat.has_value() && lon.has_value()) {
                    std::optional<std::string> name = G->get_name_from_id(id);
                    if (name.has_value()) {
                        QString qname(name->c_str());
                        Position pos{lat.value(), lon.value(), qname};
                        object_positions.positions.insert(qname, pos);
                        emit object_positions.pos_changed(qname);
                    }
                }
            }

        }, Qt::QueuedConnection);


    }

	~LeafLetGPSViewer()
    {
        Q_CLEANUP_RESOURCE(webpage);
    }

    //QWebEngineView inspector;
    PositionMap object_positions;
    QWebChannel *com_channel;
    DSR::DSRGraph *G;

};
#endif
