//
// Created by robolab on 8/4/21.
//
#include <QPushButton>
#include <utility>
#include <3rd_party/QtMaterialDesignIcons/sources/MaterialDesignIcons.h>
#include <QMessageBox>
#include <QCursor>
#include "GraphEditorView.h"
#include "GraphNewElementDialog.h"


GraphEditorView::GraphEditorView(std::shared_ptr<DSR::DSRGraph> G_, QMainWindow* parent)
        :GraphViewer(std::move(G_), parent)
{
    this->setInteractive(true);
    this->current_tool = GraphTool::edit_tool;
    this->dragging = false;
    this->drag_initial_position = QPoint(0,0);
    this->dragged_item = nullptr;
    this->temp_from_node = nullptr;
    this->temp_to_node = nullptr;
    this->temp_edge = nullptr;
    this->selecting = false;
    selection_box = new QGraphicsRectItem(0,0,0,0);
    selection_box->setPen(QPen( Qt::lightGray, 1, Qt::DotLine ) );
    rt = G->get_rt_api();

    this->edit_modes_toolbar = new QToolBar(parent);
    parent->addToolBar(this->edit_modes_toolbar);

    this->edit_modes_toolbar->setObjectName("edit_modes_toolbar");
    this->edit_modes_toolbar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);


    auto *editMenu = new QMenu(tr("&Edit"));
    parent->menuBar()->addMenu(editMenu);

    modeActionsGroup = new QActionGroup(this);
    modeActionsGroup->setExclusive(true);
    modeMoveAction = editMenu->addAction(material::pixmap("cursor-move", QSize(32,32)), tr("Move"));
    modeMoveAction->setCheckable(true);
    modeActionsGroup->addAction(modeMoveAction);

    modeCreateAction = editMenu->addAction(material::pixmap("plus", QSize(32,32)), tr("Create Nodes"));
    modeCreateAction->setCheckable(true);
    modeCreateAction->setChecked(true);
    modeActionsGroup->addAction(modeCreateAction);

    modeDeleteAction = editMenu->addAction(material::pixmap("eraser", QSize(32,32)), tr("Delete Nodes"));
    modeDeleteAction->setCheckable(true);
    modeActionsGroup->addAction(modeDeleteAction);



    GraphEditorView::connect(modeMoveAction, &QAction::triggered, this, &GraphEditorView::enableMoveMode);
    GraphEditorView::connect(modeCreateAction, &QAction::triggered, this, &GraphEditorView::enableEditMode);
    GraphEditorView::connect(modeDeleteAction, &QAction::triggered, this, &GraphEditorView::enableDeleteMode);

    this->edit_modes_toolbar->addAction(modeCreateAction);
    this->edit_modes_toolbar->addAction(modeDeleteAction);
    this->edit_modes_toolbar->addAction(modeMoveAction);
//    this->edit_modes_toolbar->addAction(modeNodesAction);
//    this->edit_modes_toolbar->addAction(modeTransformAction);
//    this->edit_modes_toolbar->addAction(modeFactorAction);
}


void GraphEditorView::mousePressEvent(QMouseEvent *event)
{
    press_point = event->pos();
    release_point = QPoint();
    distance_dragged = 0;
    // Check the tool being used
    if (event->button() == Qt::LeftButton) {
        QPointF position = mapToScene(event->pos());
        auto item = this->scene.itemAt(position, QTransform());
        if(item) {
            qDebug()<<__FUNCTION__ <<">> Taken "<<item;
            this->dragged_item = item;
            this->dragging = true;
            this->drag_initial_position = event->pos();
            auto* one_node = dynamic_cast<GraphNode*>(item);
            if (one_node!=nullptr) {
                emit this->graph_node_clicked(one_node->id_in_graph);
            }

        }
    }
    event->accept();
    switch (this->current_tool) {

    case GraphTool::delete_tool:
    case GraphTool::move_tool:
    case GraphTool::selection_tool:
        selection_box->setRect(QRectF(mapToScene(press_point), mapToScene(press_point)));
        if(!dragging) {
            this->scene.clearSelection();
            this->selecting = true;
            this->scene.addItem(selection_box);
        }
        GraphViewer::mousePressEvent(event);
        break;
    case GraphTool::edit_tool:
        if(!dragging)
        {
            // Create visual FROM node in position of the event
            this->temp_from_node = this->new_visual_node(0, "interacting", "_from_", false);
            this->temp_from_node->setPos(mapToScene(event->pos()));
            this->dragging = true;
            this->drag_initial_position = event->pos();
        }
    }

}

void GraphEditorView::mouseReleaseEvent(QMouseEvent* event)
{
    release_point = event->pos();
    QLineF distance(this->press_point, this->release_point);
    GraphNode* from_node = nullptr;
    GraphNode* to_node = nullptr;
    // Look for FROM node
    if (this->dragging and this->dragged_item) {
        from_node = dynamic_cast<GraphNode*>(this->dragged_item);
    }
    auto items = this->scene.items(mapToScene(release_point));
    if (!items.empty()) {
                foreach (auto item, items) {
                to_node = dynamic_cast<GraphNode*>(item);
                if (to_node) {
                    if (to_node==this->temp_from_node)
                    {
                        to_node = nullptr;
                        continue;
                    }
                    else if (to_node!=this->temp_to_node and to_node!=this->temp_from_node) {

                        break;
                    }
                    else {
                        to_node = nullptr;
                    }
                }
            }
    }

    switch (this->current_tool) {
    case GraphTool::edit_tool: {
        auto a = EditAction(calculate_action(from_node, to_node));
        switch (a)
        {
        case EditAction::self_edge:
        {
            qDebug()<<__FUNCTION__ << "EditActions::self_edge";
            QMessageBox::warning(this,QString("WARNING in ")+ __FUNCTION__, "Self edges not implemented yet.");
            break;
        }
        case EditAction::two_node_one_edge:
        {
            qDebug()<<__FUNCTION__ << "EditActions::two_node_one_edge";
            this->create_two_connected_nodes(mapToScene(this->drag_initial_position), mapToScene(event->pos()));
            break;
        };
        case EditAction::one_node_to_to:
        case EditAction::one_node_to_from:
        {
            qDebug()<<__FUNCTION__ << "EditActions::one_node_(one_edge)";
            bool new_node_is_to = (to_node==nullptr);
            auto new_node_pos = new_node_is_to ? event->pos() : this->drag_initial_position;
            auto existing_node = new_node_is_to ? from_node : to_node;
            if (!this->create_new_connected_node(mapToScene(new_node_pos), existing_node->id_in_graph,
                    !new_node_is_to)) {
                QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__, "Could not create node and edge.");
            }
            break;
        };
        case EditAction::one_node:
        {
            qDebug()<<__FUNCTION__ << "EditActions::one_node";
            auto from_id_optional = this->create_new_node(mapToScene(this->drag_initial_position));
            if (!from_id_optional.has_value()){
                QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__, "Problem creating from node");
                qDebug() << "Problem creating node";
            }
            break;
        };
        case EditAction::one_edge:
        {
            qDebug()<<__FUNCTION__ << "EditActions::one_edge";
            this->create_new_edge(from_node->id_in_graph, to_node->id_in_graph);
            break;
        };
        default:
            qDebug()<<__FUNCTION__ << "OTRA:"<<QString::number(int(a),2);
            QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__, QString("Unknown mouse action: ")+QString::number(int(a),2));
            break;
        }
        event->accept();
        break;
    }
    case GraphTool::move_tool: {
        update_selected_nodes();
        if (event->button()==Qt::LeftButton) {
            if (distance.length()<20) {
                auto all_items = this->scene.items(mapToScene(this->release_point));
                if (!all_items.empty()) {
                    bool node_found, edge_found = false;
                            foreach (auto item, all_items) {
                            auto* one_node = dynamic_cast<GraphNode*>(item);
                            if (one_node!=nullptr) {
                                emit this->graph_node_clicked(one_node->id_in_graph);
                                node_found = true;
                            }
                            auto* one_edge = dynamic_cast<GraphEdge*>(item);
                            if (one_edge!=nullptr) {
                                emit this->graph_edge_clicked(one_edge->sourceNode()->id_in_graph,
                                        one_edge->destNode()->id_in_graph, one_edge->type());
                                edge_found = true;
                            }
                            if (node_found and edge_found)
                                break;
                        }
                }
            }
        }
        GraphViewer::mouseReleaseEvent(event);
        break;
    }
    case GraphTool::delete_tool: {
        update_selected_nodes();
        this->safe_delete_slot();
        break;
    }
    case GraphTool::selection_tool:
    default: { GraphViewer::mouseReleaseEvent(event); }
    }

    this->dragging = false;
    this->dragged_item = nullptr;
    this->delete_temps();
    press_point = QPoint();
}

/* Translates some checks to a number to be able to switch mouse actions
+--------------------+-------------------+---------+-----------+------+------------+--------------+
| ACTION/CONDITIONS  | distance > radius | temp to | temp from | same | dragged to | dragged from |
+--------------------+-------------------+---------+-----------+------+------------+--------------+
| self_edge          |                 0 |       0 |         0 |    1 |          1 |            1 |
| two_node_one_edge  |                 1 |       1 |         1 |    0 |          0 |            0 |
| one_node_to_to     |                 1 |       1 |         1 |    0 |          1 |            0 |
| one_node_to_from   |                 1 |       1 |         0 |    0 |          0 |            1 |
| one_node           |                 0 |       0 |         1 |    0 |          0 |            0 |
| one_edge           |                 1 |       1 |         0 |    0 |          1 |            1 |
+--------------------+-------------------+---------+-----------+------+------------+--------------+
 */
int GraphEditorView::calculate_action(GraphNode* from_node,GraphNode* to_node)
{
    int result = 0;
    if(from_node)
        result |= (1u);
    if(to_node)
        result |= (1u << 1);
    if(from_node==to_node and from_node)
        result |= (1u << 2);
    if(temp_from_node)
        result |= (1u << 3);
    if(temp_to_node)
        result |= (1u << 4);
    if(distance_dragged > 20)
        result |= (1u << 5);
    return result;
}

void GraphEditorView::mouseMoveEvent(QMouseEvent* event)
{
    switch (this->current_tool) {
        // If we are editing nodes
    case GraphTool::edit_tool:
        // If dragging
        if (this->dragging) {
            double new_distance = QLineF(mapToScene(last_dragged_pos), mapToScene(event->pos())).length();
            distance_dragged+=new_distance;
            last_dragged_pos = event->pos();
            // update the position of the temp TO node
            if (this->temp_to_node) {
                this->temp_to_node->setPos(mapToScene(event->pos()));
            }
            else
            {
                QLineF distance;
                GraphNode* source_node;
                if (this->temp_from_node)
                {
                    distance = QLineF(event->pos(),this->temp_from_node->pos());
                    source_node = this->temp_from_node;
                }
                if (this->dragged_item and dynamic_cast<GraphNode*>(this->dragged_item))
                {
                    distance = QLineF(event->pos(),this->dragged_item->pos());
                    source_node = dynamic_cast<GraphNode*>(this->dragged_item);
                }
                if (distance.length() > 30)
                {
                    this->temp_to_node = this->new_visual_node(9999,"plane", "_to_", false);
                    this->temp_edge = this->new_visual_edge(source_node, this->temp_to_node, "_e_");
                }
            }
        }
        event->accept();
        break;
    case GraphTool::delete_tool:
        update_selection(event);
        event->accept();
    case GraphTool::move_tool:
        update_selection(event);
        GraphViewer::mouseMoveEvent(event);
        break;
    default:
        GraphViewer::mouseMoveEvent(event);
    }
}
void GraphEditorView::update_selection(const QMouseEvent* event)
{
    if(selecting)
        selection_box->setRect(QRectF(mapToScene(press_point), mapToScene(event->pos())));
}

void GraphEditorView::delete_temps()
{
    if (this->temp_edge)
        this->scene.removeItem(this->temp_edge);
    if (this->temp_from_node)
        this->scene.removeItem(this->temp_from_node);
    if (this->temp_to_node)
        this->scene.removeItem(this->temp_to_node);
    this->temp_from_node = nullptr;
    this->temp_to_node = nullptr;
    this->temp_edge = nullptr;
}

std::optional<uint64_t> GraphEditorView::create_new_node(QPointF position)
{
    //get node type
    bool ok;
    QStringList results = GraphNewElementDialog::getNodeParams(this, ok);
    qDebug()<<results;
    if(not ok or results.size()<2)
        return std::optional<uint64_t>();
    return this->_create_new_G_node(results[0], results[1], position);
}


std::optional<uint64_t> GraphEditorView::_create_new_G_node(const QString& name, const QString& type, QPointF position)
{
    DSR::Node node;
    node.type(type.toStdString());
    node.name(name.toStdString());
    this->G->add_or_modify_attrib_local<pos_x_att>(node, float(position.x()));
    this->G->add_or_modify_attrib_local<pos_y_att>(node, float(position.y()));
    this->G->add_or_modify_attrib_local<color_att>(node, std::string("GoldenRod"));
    if (G->size()==0) {
        G->add_or_modify_attrib_local<level_att>(node, 0);
    }

    try
    {
        return this->G->insert_node(node);
    }
    catch(const std::exception& e)
    {
        std::cout << __FUNCTION__ <<  e.what() << std::endl;
        QMessageBox::critical(this,QString("Error in ")+ __FUNCTION__,e.what());
    }
    return {};
}



bool GraphEditorView::create_new_edge(uint64_t from, uint64_t to) {
    bool ok;

    QStringList results = GraphNewElementDialog::getEdgeParams(this, ok);
    if(not ok or results.empty())
        return false;

    return this->_create_new_G_edge(results[0], from, to);
}


bool GraphEditorView::_create_new_G_edge(const QString& type, uint64_t from_id, uint64_t to_id) {
    std::optional<DSR::Node> from_node = G->get_node(from_id);
    std::optional<DSR::Node> to_node = G->get_node(to_id);
    if (from_node.has_value() and to_node.has_value()) {
        if(type == "RT")
        {
            // Check levels
            std::vector<float> trans{0.f, 0.f, 0.f};
            std::vector<float> rot{0, 0.f, 0};

            auto from_node_level = G->get_node_level(from_node.value());
            auto to_node_level = G->get_node_level(to_node.value());
            if (from_node_level.has_value() and !to_node_level.has_value()) {
                try {
                    DSR::Node real_node = from_node.value();
                    rt->insert_or_assign_edge_RT(real_node, to_id, trans, rot);
                    return true;
                }
                catch (const std::exception& e) {
                    QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__,e.what());
                    std::cout << __FUNCTION__ << e.what() << std::endl;
                }
            }
            else if(!from_node_level.has_value() and to_node_level.has_value())
            {
                try {
                    DSR::Node real_node = to_node.value();
                    rt->insert_or_assign_edge_RT(real_node, from_id, trans, rot);
                    return true;
                }
                catch (const std::exception& e) {
                    QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__,e.what());
                    std::cout << __FUNCTION__ << e.what() << std::endl;
                }
            }
            else if(!from_node_level.has_value() and !to_node_level.has_value())
            {
                QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__, "No RT edge can be created.\nNONE of the nodes have LEVEL!");
                qDebug()<< __FUNCTION__ <<" NONE of the nodes have LEVEL!";
                return false;
            }
            else
            {
                QMessageBox::warning(this,QString("Error in ")+ __FUNCTION__, "No RT edge can be created.\nBOTH of the nodes already had LEVEL attribute!");
                qDebug()<< __FUNCTION__ <<" BOTH of the nodes have LEVEL!";
                return false;
            }
        }
        else
        {
            DSR::Edge edge;
            edge.type(type.toStdString());
            //get two ids
            edge.from(from_id);
            edge.to(to_id);
            if(not G->insert_or_assign_edge(edge))
            {
                std::cout<<"Error inserting new edge: "<<from_id<<"->"<<to_id<<" type: "<<type.toStdString()<<std::endl;
                return false;
            }
            return true;
        }
    }
    else{
        QMessageBox::warning(this,QString("WARNING in ")+ __FUNCTION__, QString("Selected node from or to does not exist ")+from_id+"("+from_node.has_value()+")->"+to_id+"("+to_node.has_value()+") type: "+type);
        qDebug()<<__FUNCTION__ <<">> Selected node from or to does not exist "<<from_id<<"("<<from_node.has_value()<<")->"<<to_id<<"("<<to_node.has_value()<<") type: "<<type;
    }
    return false;
}


bool GraphEditorView::create_two_connected_nodes(QPointF position1, QPointF position2, bool reverse) {
    bool ok;
    QStringList results = GraphNewElementDialog::getAllParams(this, ok);
    if(not ok or results.size()<2)
        return ok;

    // Create nodes
    std::optional<uint64_t> new_node_id1 = this->_create_new_G_node(results[0], results[1], position1);
    std::optional<uint64_t> new_node_id2 = this->_create_new_G_node(results[2], results[3], position2);
    if (new_node_id1.has_value() and new_node_id2.has_value())
        if(reverse)
            return this->_create_new_G_edge(results[4], new_node_id2.value(), new_node_id1.value());
        else
            return this->_create_new_G_edge(results[4], new_node_id1.value(), new_node_id2.value());
    else
        return false;

}

bool GraphEditorView::create_new_connected_node(QPointF position, uint64_t node_id, bool reverse)
{
    bool ok;
    QStringList results = GraphNewElementDialog::getConnectedNodeParams(this, ok);
    if(not ok or results.size()<2)
        return ok;

    // Create node
    try {
        std::optional<uint64_t> new_node_id = this->_create_new_G_node(results[0], results[1], position);
        if (new_node_id.has_value())
        {
            if(reverse)
                return this->_create_new_G_edge(results[2], new_node_id.value(), node_id);
            else
                return this->_create_new_G_edge(results[2],  node_id, new_node_id.value());
        }
        else
            return false;
    }
    catch (const std::exception& e) {
        std::cout << __FUNCTION__ <<  e.what() << std::endl;
        QMessageBox::critical(this,QString("Error in ")+ __FUNCTION__,e.what());
        return false;
    }



}
void GraphEditorView::enableMoveMode()
{
    this->current_tool = GraphTool::move_tool;
    selection_box->setRect(0,0,0,0);
    this->scene.removeItem(selection_box);
}

void GraphEditorView::enableEditMode()
{
    this->current_tool = GraphTool::edit_tool;
}

void GraphEditorView::enableDeleteMode()
{
    this->current_tool = GraphTool::delete_tool;
    delete_shortcut = new QShortcut(QKeySequence(Qt::CTRL+Qt::Key_Delete), this);
    connect(delete_shortcut, SIGNAL(activated()), this, SLOT(delete_slot()));

    safe_delete_shortcut = new QShortcut(QKeySequence(Qt::Key_Delete), this);
    connect(safe_delete_shortcut, SIGNAL(activated()), this, SLOT(safe_delete_slot()));


    setCursor(QCursor(material::pixmap("eraser", QSize(32,32)), 0, 0));
}

void GraphEditorView::safe_delete_slot(){
    QMessageBox::StandardButton reply;
    reply = QMessageBox::warning(this, QString("Deleting %1 elements").arg(this->scene.selectedItems().count()), QString("Are you sure that you want to delete %1 elements?").arg(this->scene.selectedItems().count()),
            QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        delete_slot();
    }
}

void GraphEditorView::delete_slot()
{
    for(auto item : this->scene.selectedItems()) {
        auto node = dynamic_cast<GraphNode*>(item);
        if(node!=nullptr)
        {
            if (not G->delete_node(node->id_in_graph)) {
                qDebug() << "Node" << node->id_in_graph << "could not be deleted";
                QMessageBox::critical(this,QString("Error in ")+ __FUNCTION__,  QString("Node")+node->id_in_graph+"could not be deleted");
            }
        }
    }
}
void GraphEditorView::update_selected_nodes()
{
    if (this->selecting) {

        for (auto item : this->selection_box->collidingItems()) {
            if (dynamic_cast<GraphNode*>(item))
                item->setSelected(true);
        }
        this->scene.removeItem(this->selection_box);
        this->selecting = false;
    }
}
