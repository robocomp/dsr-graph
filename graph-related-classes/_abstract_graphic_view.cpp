//
// Created by robolab on 12/06/20.
//

#include "_abstract_graphic_view.h"


using namespace DSR ;



AbstractGraphicViewer::AbstractGraphicViewer(QWidget* parent) :  QGraphicsView(parent)
{
	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	scene.setSceneRect(-200, -200, 400, 400);
	this->setScene(&scene);
	this->setCacheMode(QGraphicsView::CacheBackground);
	this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	this->setRenderHint(QPainter::Antialiasing);
	this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	this->setMinimumSize(400, 400);
	this->fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	this->adjustSize();
	this->setMouseTracking(true);
	this->viewport()->setMouseTracking(true);
}


//////////////////////////////////////////////////////////////////////////////////////
///// EVENTS
//////////////////////////////////////////////////////////////////////////////////////

void AbstractGraphicViewer::wheelEvent(QWheelEvent* event)
{
	qreal factor;
	if (event->angleDelta().y() > 0)
	{
		factor = 1.1;

	}
	else
	{
		factor = 0.9;

	}
	auto view_pos = event->pos();
	auto scene_pos = this->mapToScene(view_pos);
	this->centerOn(scene_pos);
	this->scale(factor, factor);
	auto delta = this->mapToScene(view_pos) - this->mapToScene(this->viewport()->rect().center());
	this->centerOn(scene_pos - delta);
}

void AbstractGraphicViewer::resizeEvent(QResizeEvent *e)
{
//	qDebug() << "resize_graph_view" << x() << y()<<e->size();
	QGraphicsView::resizeEvent(e);
}


void AbstractGraphicViewer::mousePressEvent(QMouseEvent *event)
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

void AbstractGraphicViewer::mouseReleaseEvent(QMouseEvent *event)
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

void AbstractGraphicViewer::mouseMoveEvent(QMouseEvent *event)
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