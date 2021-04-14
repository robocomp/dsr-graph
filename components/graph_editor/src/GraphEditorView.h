//
// Created by robolab on 8/4/21.
//

#ifndef TESTCOMP_GRAPHEDITORVIEW_H
#define TESTCOMP_GRAPHEDITORVIEW_H

#include <QMouseEvent>
#include <dsr/gui/viewers/graph_viewer/graph_viewer.h>
#include <dsr/gui/viewers/graph_viewer/graph_node.h>
#include <QToolBar>

enum class GraphTool{
    edit_tool,
    selecction_tool,
    move_tool
};

class GraphEditorView : public DSR::GraphViewer{
public:
    GraphEditorView(std::shared_ptr<DSR::DSRGraph> G_, QMainWindow *parent=0);
private:
    GraphTool current_tool;
    QGraphicsItem *dragged_item;
    QPoint drag_initial_position;
    std::unique_ptr<DSR::RT_API> rt;
    QToolBar *edit_modes_toolbar;
//    QAction *modeDefaultAction;
    QActionGroup* modeActionsGroup;
    QAction *modeMoveAction;
    QAction *modeCreatetAction;
    bool dragging;
    GraphNode* temp_from_node;
    GraphNode* temp_to_node;
    GraphEdge* temp_edge;

private:
    std::optional<uint64_t> create_new_node(QPointF position);
    std::optional<uint64_t> _create_new_G_node(QString name, QString type, QPointF position);
    bool create_two_connected_nodes(QPointF position1, QPointF position2, bool reverse=false);
    bool create_new_connected_node(QPointF position, uint64_t node_id, bool reverse=false);
    bool create_new_edge(uint64_t from, uint64_t to);
    bool _create_new_G_edge(QString type, uint64_t from_id, uint64_t to_id);
    void delete_temps();
//    void wheelEvent(QWheelEvent* event);
//    void resizeEvent(QResizeEvent* e);
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
//    void showEvent(QShowEvent *event);

    void enableMoveMode(bool action);
    void enableEditMode(bool action);

};

#endif //TESTCOMP_GRAPHEDITORVIEW_H
