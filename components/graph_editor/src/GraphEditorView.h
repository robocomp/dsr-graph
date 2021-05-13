//
// Created by robolab on 8/4/21.
//

#ifndef TESTCOMP_GRAPHEDITORVIEW_H
#define TESTCOMP_GRAPHEDITORVIEW_H

#include <QMouseEvent>
#include <dsr/gui/viewers/graph_viewer/graph_viewer.h>
#include <dsr/gui/viewers/graph_viewer/graph_node.h>
#include <QToolBar>
#include <QShortcut>

enum class GraphTool{
    edit_tool,
    selecction_tool,
    move_tool,
    delete_tool
};

class GraphEditorView : public DSR::GraphViewer{
Q_OBJECT
public:
    explicit GraphEditorView(std::shared_ptr<DSR::DSRGraph> G_, QMainWindow *parent=nullptr);
private:
    GraphTool current_tool;
    QGraphicsItem *dragged_item;
    QGraphicsRectItem* selection_box;
    QPoint drag_initial_position;
    std::unique_ptr<DSR::RT_API> rt;
    QToolBar *edit_modes_toolbar;
//    QAction *modeDefaultAction;
    QActionGroup* modeActionsGroup;
    QAction *modeMoveAction;
    QAction *modeCreatetAction;
    QAction *modeDeleteAction;
    QShortcut* delete_shortcut;
    QShortcut* safe_delete_shortcut;

    bool dragging;
    bool selecting;
    GraphNode* temp_from_node;
    GraphNode* temp_to_node;
    GraphEdge* temp_edge;
    int distance_moved{};
    QPoint press_point = QPoint();
    QPoint release_point = QPoint();

private:
    std::optional<uint64_t> create_new_node(QPointF position);
    std::optional<uint64_t> _create_new_G_node(const QString& name, const QString& type, QPointF position);
    bool create_two_connected_nodes(QPointF position1, QPointF position2, bool reverse=false);
    bool create_new_connected_node(QPointF position, uint64_t node_id, bool reverse=false);
    bool create_new_edge(uint64_t from, uint64_t to);
    bool _create_new_G_edge(const QString& type, uint64_t from_id, uint64_t to_id);
    void delete_temps();
//    void wheelEvent(QWheelEvent* event);
//    void resizeEvent(QResizeEvent* e);
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
//    void showEvent(QShowEvent *event);

    void enableMoveMode();
    void enableEditMode();
    void enableDeleteMode();

signals:
    void graph_node_clicked(uint64_t node_id);
    void graph_edge_clicked(uint64_t source_id, uint64_t dest_id, int type);
private slots:
    void delete_slot();
    void safe_delete_slot();


};

#endif //TESTCOMP_GRAPHEDITORVIEW_H
