#include "dsr_to_graphicscene_viewer.h"

using namespace DSR ;

DSRtoGraphicsceneViewer::DSRtoGraphicsceneViewer(std::shared_ptr<CRDT::CRDTGraph> G_, float scaleX, float scaleY, QGraphicsView *parent) : QGraphicsView(parent)
{
qDebug()<<"***************INIT DSRtoGraphicsceneViewer********************";
    G = G_;
    m_scaleX = scaleX;
    m_scaleY = scaleY;
    this->setMinimumSize(400,400);

    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(-5000, -5000, 10000, 10000);
	this->scale(1, -1);
    this->setScene(&scene);
    this->setCacheMode(QGraphicsView::CacheBackground);
	this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	this->setRenderHint(QPainter::Antialiasing);
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	this->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	
 	setMouseTracking(true);
    this->viewport()->setMouseTracking(true);
//REMOVE WHEN NOT NEEDED
    //center position
    scene.addRect(-100, -100, 200, 200, QPen(QColor("black")),QBrush(QColor("black")));
    //edge => x red, z (y) blue
    scene.addRect(-5000, -5000, 1000, 30, QPen(QColor("red")),QBrush(QColor("red")));
    scene.addRect(-5000, -5000, 30, 1000, QPen(QColor("blue")),QBrush(QColor("blue")));

    //initial graph
    createGraph();

    //update signals
    connect(G.get(), &CRDT::CRDTGraph::update_node_signal, this, &DSRtoGraphicsceneViewer::add_or_assign_node_slot);
	connect(G.get(), &CRDT::CRDTGraph::update_edge_signal, this, &DSRtoGraphicsceneViewer::add_or_assign_edge_slot);

	//connect(G.get(), &CRDT::CRDTGraph::del_edge_signal, this, &DSRtoGraphicsceneViewer::delEdgeSLOT);
	//connect(G.get(), &CRDT::CRDTGraph::del_node_signal, this, &DSRtoGraphicsceneViewer::delNodeSLOT);

}

void DSRtoGraphicsceneViewer::createGraph()
{
    innermodel = G->get_inner_api();
    try
    {
        auto map = G->getCopy();
		for(const auto &[k, node] : map)
		    add_or_assign_node_slot(k,  node.type());
    }
	catch(const std::exception &e)
    {
        std::cout << e.what() << " Error accessing "<< __FUNCTION__<<":"<<__LINE__<< std::endl;
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
//////////////////////////////////////////////////////////////////////////////////////

void DSRtoGraphicsceneViewer::add_or_assign_node_slot(const std::int32_t id, const std::string &type)
{
qDebug() << __FUNCTION__ ;
qDebug()<<"*************************";
    
    auto node = G->get_node(id);
std::cout << node.value().name() << " " << node.value().id() << std::endl;
    if(node.has_value())
    { 
//maybe we could check wich nodes must be drawn

        if( type == "plane" )
            add_or_assign_plane(node.value());
        if( type == "mesh")
            add_or_assign_mesh(node.value());
        if( type == "person")
            add_or_assign_person(node.value());
    }
}
void DSRtoGraphicsceneViewer::add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type)
{
    std::string edge_key = std::to_string(from) + "_" + std::to_string(to);
    for (int node_id : edgeMap[edge_key])
    {
std::cout << "******UPDATE EDGE "<<from << " " << to <<" update node " << node_id<<std::endl;
        update_scene_object_pose(node_id);
    }
}

//Compute 2d projection and zvalue
void DSRtoGraphicsceneViewer::get_2d_projection(std::string node_name, std::vector<int> size, QPolygon &polygon, int &zvalue)
{
    QVector<QPoint> polygon_vec;
    zvalue = -99999;
    //transform cube points
    std::optional<RTMat> rt = innermodel->getTransformationMatrixS("world", node_name);
    if (rt.has_value())
    {
        for (unsigned int i=0;i< cube_positions.size();i++)
        {
            QVec vec = rt.value() * QVec::vec4(size[0]*cube_positions[i][0], size[1]*cube_positions[i][1], size[2]*cube_positions[i][2], 1.0 );
            polygon_vec.append(QPoint(vec[0], vec[2]));
            if (zvalue < vec[1])
                zvalue = vec[1];
        }
    }
    else
    {
        qDebug()<<"Error gettting transformation matrix for node:"<<QString::fromStdString(node_name);   
    }
    polygon = QPolygon(polygon_vec);
} 


void DSRtoGraphicsceneViewer::add_or_assign_plane(Node &node)
{
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::string color = G->get_attrib_by_name<std::string>(node, "color").value_or("orange");
    std::string texture = G->get_attrib_by_name<std::string>(node, "texture").value_or("");
    int width = G->get_attrib_by_name<std::int32_t>(node, "width").value_or(0);
    int height = G->get_attrib_by_name<std::int32_t>(node, "height").value_or(0);
    int depth = G->get_attrib_by_name<std::int32_t>(node, "depth").value_or(0);

    qDebug()<<"Draw plane"<<QString::fromStdString(node.name())<<"("<<width<<","<<height<<","<<depth<<")";
    
    QPolygon polygon;
    int zvalue;
    get_2d_projection(node.name(), {width, height, depth}, polygon, zvalue);
    QRect rect = QPolygon(polygon).boundingRect();

    //minimum size
    if(rect.width() < 100)
        rect.setWidth(100);
    if(rect.height() < 100)
        rect.setHeight(100);

    //texture
    QBrush brush = QBrush(QColor(QString::fromStdString(color)));
    if (texture != "")
    {
        if(std::filesystem::exists(texture))
            brush = QBrush(QImage(QString::fromStdString(texture)));
        else
            brush = QBrush(QColor(QString::fromStdString(texture)));
    }
    
    QGraphicsRectItem * sceneRect;
    if (sceneMap.find(node.id()) == sceneMap.end())
    {
        sceneRect = scene.addRect(rect, QPen(QString::fromStdString(color)), brush);
        sceneMap[node.id()] = (QGraphicsItem*) sceneRect;
        create_parent_list(node.id());
    }
    else
    {
        sceneRect = (QGraphicsRectItem*) sceneMap[node.id()];
        sceneRect->setRect(rect);
    }

    sceneRect->setZValue(zvalue);

}

void  DSRtoGraphicsceneViewer::add_or_assign_mesh(Node &node)
{   
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::string color = G->get_attrib_by_name<std::string>(node, "color").value_or("orange");
    std::string filename = G->get_attrib_by_name<std::string>(node, "path").value_or("");
    int scalex = G->get_attrib_by_name<std::int32_t>(node, "scalex").value_or(0);
    int scaley = G->get_attrib_by_name<std::int32_t>(node, "scaley").value_or(0);
    int scalez = G->get_attrib_by_name<std::int32_t>(node, "scalez").value_or(0);

    qDebug()<<"Draw mesh"<<QString::fromStdString(node.name())<<"("<<scalex<<","<<scaley<<","<<scalez<<")";




}

void  DSRtoGraphicsceneViewer::add_or_assign_person(Node &node)
{   
qDebug() << "********************************";
qDebug() << __FUNCTION__ ;
    std::optional<QVec> pose;
    try
    {
        pose = innermodel->transformS6D("world", node.name());
    }catch(...){}
    if (pose.has_value())
    {
pose.value().print(QString::fromStdString(node.name()));

        //check if person already exists
        QGraphicsPixmapItem * scenePixmap;
        if (sceneMap.find(node.id()) == sceneMap.end())
        {
            QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(600,300);
            scenePixmap = scene.addPixmap(pixmap);
            scenePixmap->setZValue(pose.value().y());
            scenePixmap->setPos(pose.value().x() - 300,  pose.value().z() - 150);
            scenePixmap->setRotation(pose.value().ry());
            sceneMap[node.id()] = (QGraphicsItem*) scenePixmap;
            create_parent_list(node.id());
        }
        else
        {
            scenePixmap = (QGraphicsPixmapItem*) sceneMap[node.id()];
            scenePixmap->setPos(pose.value().x() - 300, pose.value().z() - 150);
qDebug()<<"angle"<<pose.value().ry()<<qRadiansToDegrees(pose.value().ry());
            scenePixmap->setRotation(-qRadiansToDegrees(pose.value().ry())+180);
        }
    
    }
    else
    {
        qDebug()<<"Error gettion tranformation from person"<<QString::fromStdString(node.name())<<"to world";
    }


}

void DSRtoGraphicsceneViewer::create_parent_list(std::int32_t node_id)
{
try{
    auto node = G->get_node(node_id);
    int parent_id;
    int actual_id = node_id;
//std::cout<<"Node: "<<node_id<<" => ";
    do
    {
        parent_id = node.value().attrs()["parent"].value().dec();
        std::string edge_name = std::to_string(parent_id) + "_" + std::to_string(actual_id);
        edgeMap[edge_name].push_back(node_id);
        actual_id = parent_id;
        node = G->get_node(actual_id);
    }while(node.value().type() != "world");


}catch(...){}
}


//update pose on edge changes
void DSRtoGraphicsceneViewer::update_scene_object_pose(std::int32_t node_id)
{
std::cout << "*************UPDATE NODE ******" << node_id<<std::endl;
    auto node = G->get_node(node_id);
    if (node.has_value())
    {
        QGraphicsItem *item = sceneMap[node_id];
        std::optional<QVec> pose;
        try
        {
            pose = innermodel->transformS6D("world", node.value().name());
        }catch(...){};
        if (pose.has_value())
        {
            item->setPos(pose.value().x() - item->boundingRect().center().x(), pose.value().z() - item->boundingRect().center().y());
            item->setRotation(qRadiansToDegrees(pose.value().ry())+180);
        }
        else   
        {
            qDebug()<<"Error gettion tranformation from person"<<QString::fromStdString(node.value().name())<<"to world";
        }

    }
}
/*
void DSRtoGraphicsceneViewer::delete_scene_rect(std::int32_t node_id)
{

}

*/


//////////////////////////////////////////////////////////////
//                  MOUSE                                   //
//////////////////////////////////////////////////////////////
void DSRtoGraphicsceneViewer::wheelEvent(QWheelEvent* event)
{
//    qDebug()<<"wheel";
    const QGraphicsView::ViewportAnchor anchor = this->transformationAnchor();
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	qreal factor;
	if (event->angleDelta().y() > 0) 
	{
		factor = 1.1;
		QRectF r = scene.sceneRect();
		scene.setSceneRect(r);
	}
	else
	{
		factor = 0.9;
		QRectF r = scene.sceneRect();
		scene.setSceneRect(r);
	}
	this->scale(factor, factor);
	this->setTransformationAnchor(anchor);
}

void DSRtoGraphicsceneViewer::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        _pan = true;
        _panStartX = event->x();
        _panStartY = event->y();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    event->ignore();
}

void DSRtoGraphicsceneViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        _pan = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    event->ignore();
}

void DSRtoGraphicsceneViewer::mouseMoveEvent(QMouseEvent *event)
{
    if (_pan)
    {
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - _panStartX));
        verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - _panStartY));
        _panStartX = event->x();
        _panStartY = event->y();
        event->accept();
        return;
    }
    event->ignore();
}

void DSRtoGraphicsceneViewer::resizeEvent(QResizeEvent *e)
{  
//	qDebug() << "resize_graph_view" << x() << y()<<e->size(); 
	this->resize(e->size());
} 