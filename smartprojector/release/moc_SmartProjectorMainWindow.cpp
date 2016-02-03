/****************************************************************************
** Meta object code from reading C++ file 'SmartProjectorMainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../SmartProjectorMainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SmartProjectorMainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SmartProjectorMainWindow_t {
    QByteArrayData data[11];
    char stringdata0[137];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SmartProjectorMainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SmartProjectorMainWindow_t qt_meta_stringdata_SmartProjectorMainWindow = {
    {
QT_MOC_LITERAL(0, 0, 24), // "SmartProjectorMainWindow"
QT_MOC_LITERAL(1, 25, 10), // "openDeivce"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 7), // "timeout"
QT_MOC_LITERAL(4, 45, 12), // "startInspect"
QT_MOC_LITERAL(5, 58, 10), // "endInspect"
QT_MOC_LITERAL(6, 69, 16), // "calibrationImage"
QT_MOC_LITERAL(7, 86, 16), // "correctionScreen"
QT_MOC_LITERAL(8, 103, 15), // "changeHumanFovX"
QT_MOC_LITERAL(9, 119, 1), // "d"
QT_MOC_LITERAL(10, 121, 15) // "changeHumanFovY"

    },
    "SmartProjectorMainWindow\0openDeivce\0"
    "\0timeout\0startInspect\0endInspect\0"
    "calibrationImage\0correctionScreen\0"
    "changeHumanFovX\0d\0changeHumanFovY"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SmartProjectorMainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x0a /* Public */,
       3,    0,   55,    2, 0x0a /* Public */,
       4,    0,   56,    2, 0x0a /* Public */,
       5,    0,   57,    2, 0x0a /* Public */,
       6,    0,   58,    2, 0x0a /* Public */,
       7,    0,   59,    2, 0x0a /* Public */,
       8,    1,   60,    2, 0x0a /* Public */,
      10,    1,   63,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    9,
    QMetaType::Void, QMetaType::Double,    9,

       0        // eod
};

void SmartProjectorMainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SmartProjectorMainWindow *_t = static_cast<SmartProjectorMainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->openDeivce(); break;
        case 1: _t->timeout(); break;
        case 2: _t->startInspect(); break;
        case 3: _t->endInspect(); break;
        case 4: _t->calibrationImage(); break;
        case 5: _t->correctionScreen(); break;
        case 6: _t->changeHumanFovX((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->changeHumanFovY((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject SmartProjectorMainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_SmartProjectorMainWindow.data,
      qt_meta_data_SmartProjectorMainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SmartProjectorMainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SmartProjectorMainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SmartProjectorMainWindow.stringdata0))
        return static_cast<void*>(const_cast< SmartProjectorMainWindow*>(this));
    if (!strcmp(_clname, "Ui::CarInspectMainWindow"))
        return static_cast< Ui::CarInspectMainWindow*>(const_cast< SmartProjectorMainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int SmartProjectorMainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
