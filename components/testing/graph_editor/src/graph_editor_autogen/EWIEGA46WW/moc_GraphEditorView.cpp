/****************************************************************************
** Meta object code from reading C++ file 'GraphEditorView.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "GraphEditorView.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GraphEditorView.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_GraphEditorView_t {
    QByteArrayData data[14];
    char stringdata0[166];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GraphEditorView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GraphEditorView_t qt_meta_stringdata_GraphEditorView = {
    {
QT_MOC_LITERAL(0, 0, 15), // "GraphEditorView"
QT_MOC_LITERAL(1, 16, 18), // "graph_node_clicked"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 8), // "uint64_t"
QT_MOC_LITERAL(4, 45, 7), // "node_id"
QT_MOC_LITERAL(5, 53, 18), // "graph_edge_clicked"
QT_MOC_LITERAL(6, 72, 9), // "source_id"
QT_MOC_LITERAL(7, 82, 7), // "dest_id"
QT_MOC_LITERAL(8, 90, 4), // "type"
QT_MOC_LITERAL(9, 95, 11), // "delete_slot"
QT_MOC_LITERAL(10, 107, 16), // "safe_delete_slot"
QT_MOC_LITERAL(11, 124, 16), // "update_selection"
QT_MOC_LITERAL(12, 141, 18), // "const QMouseEvent*"
QT_MOC_LITERAL(13, 160, 5) // "event"

    },
    "GraphEditorView\0graph_node_clicked\0\0"
    "uint64_t\0node_id\0graph_edge_clicked\0"
    "source_id\0dest_id\0type\0delete_slot\0"
    "safe_delete_slot\0update_selection\0"
    "const QMouseEvent*\0event"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GraphEditorView[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    3,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    0,   49,    2, 0x08 /* Private */,
      10,    0,   50,    2, 0x08 /* Private */,
      11,    1,   51,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3, QMetaType::Int,    6,    7,    8,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,

       0        // eod
};

void GraphEditorView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GraphEditorView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->graph_node_clicked((*reinterpret_cast< uint64_t(*)>(_a[1]))); break;
        case 1: _t->graph_edge_clicked((*reinterpret_cast< uint64_t(*)>(_a[1])),(*reinterpret_cast< uint64_t(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 2: _t->delete_slot(); break;
        case 3: _t->safe_delete_slot(); break;
        case 4: _t->update_selection((*reinterpret_cast< const QMouseEvent*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (GraphEditorView::*)(uint64_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GraphEditorView::graph_node_clicked)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (GraphEditorView::*)(uint64_t , uint64_t , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GraphEditorView::graph_edge_clicked)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject GraphEditorView::staticMetaObject = { {
    &DSR::GraphViewer::staticMetaObject,
    qt_meta_stringdata_GraphEditorView.data,
    qt_meta_data_GraphEditorView,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *GraphEditorView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GraphEditorView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GraphEditorView.stringdata0))
        return static_cast<void*>(this);
    return DSR::GraphViewer::qt_metacast(_clname);
}

int GraphEditorView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = DSR::GraphViewer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void GraphEditorView::graph_node_clicked(uint64_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GraphEditorView::graph_edge_clicked(uint64_t _t1, uint64_t _t2, int _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
