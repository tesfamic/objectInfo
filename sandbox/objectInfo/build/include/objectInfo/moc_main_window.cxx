/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Sat Apr 26 20:18:47 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/objectInfo/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_objectInfo__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      24,   23,   23,   23, 0x0a,
      57,   51,   23,   23, 0x0a,
      95,   89,   23,   23, 0x0a,
     141,   23,   23,   23, 0x0a,
     161,   23,   23,   23, 0x0a,
     177,   23,   23,   23, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_objectInfo__MainWindow[] = {
    "objectInfo::MainWindow\0\0"
    "on_actionAbout_triggered()\0check\0"
    "on_button_connect_clicked(bool)\0state\0"
    "on_checkbox_use_environment_stateChanged(int)\0"
    "updateLoggingView()\0updateDisplay()\0"
    "on_angleInput_valueChanged()\0"
};

void objectInfo::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_checkbox_use_environment_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->updateLoggingView(); break;
        case 4: _t->updateDisplay(); break;
        case 5: _t->on_angleInput_valueChanged(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData objectInfo::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject objectInfo::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_objectInfo__MainWindow,
      qt_meta_data_objectInfo__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &objectInfo::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *objectInfo::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *objectInfo::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_objectInfo__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int objectInfo::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
