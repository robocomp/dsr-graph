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
    selection_tool,
    move_tool,
    delete_tool
};

enum class EditAction{
    /*
     * 0b00 | distance_dragged > 20 | temp_to_node | temp_from_node | from_node==to_node and from_node and to_node | to_node | from_node
        +--------------------+-------------------+---------+-----------+------+------------+--------------+
        | ACTION/CONDITIONS  | distance > radius | temp to | temp from | same | dragged to | dragged from |
        +--------------------+-------------------+---------+-----------+------+------------+--------------+
        | self_edge          |                 - |       0 |         0 |    1 |          1 |            1 |
        | two_node_one_edge  |                 1 |       1 |         1 |    0 |          0 |            0 |
        | one_node_to_to     |                 1 |       1 |         1 |    0 |          1 |            0 |
        | one_node_to_from   |                 1 |       1 |         0 |    0 |          0 |            1 |
        | one_node           |                 0 |       0 |         1 |    0 |          0 |            0 |
        | one_edge           |                 1 |       1 |         0 |    0 |          1 |            1 |
        +--------------------+-------------------+---------+-----------+------+------------+--------------+
     */
    self_edge = 0b00100111,
    two_node_one_edge = 0b00111000,
    one_node_to_to = 0b00111010,
    one_node_to_from = 0b00110001,
    one_node = 0b00001000,
    one_edge = 0b00110011
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
    QAction *modeCreateAction;
    QAction *modeDeleteAction;
    QShortcut* delete_shortcut{};
    QShortcut* safe_delete_shortcut{};

    bool dragging;
    bool selecting;
    QPoint last_dragged_pos;
    GraphNode* temp_from_node;
    GraphNode* temp_to_node;
    GraphEdge* temp_edge;
    double distance_dragged = 0;
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
    void update_selected_nodes();
//    void wheelEvent(QWheelEvent* event);
//    void resizeEvent(QResizeEvent* e);
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
//    void showEvent(QShowEvent *event);

    void enableMoveMode();
    void enableEditMode();
    void enableDeleteMode();
    int calculate_action(GraphNode* from_node, GraphNode* to_node);

signals:
    void graph_node_clicked(uint64_t node_id);
    void graph_edge_clicked(uint64_t source_id, uint64_t dest_id, int type);
private slots:
    void delete_slot();
    void safe_delete_slot();

    void update_selection(const QMouseEvent* event);

};

#endif //TESTCOMP_GRAPHEDITORVIEW_H
