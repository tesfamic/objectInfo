/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Sat Apr 26 18:39:51 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ceilbot/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ceilbot__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x0a,
      48,   20,   20,   20, 0x0a,
      74,   20,   20,   20, 0x0a,
      99,   20,   20,   20, 0x0a,
     128,   20,   20,   20, 0x0a,
     157,   20,   20,   20, 0x0a,
     186,   20,   20,   20, 0x0a,
     215,   20,   20,   20, 0x0a,
     244,   20,   20,   20, 0x0a,
     273,   20,   20,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ceilbot__MainWindow[] = {
    "ceilbot::MainWindow\0\0on_actionAbout_triggered()\0"
    "on_start_button_clicked()\0"
    "on_quit_button_clicked()\0"
    "on_sliderHmin_valueChanged()\0"
    "on_sliderHmax_valueChanged()\0"
    "on_sliderSmin_valueChanged()\0"
    "on_sliderSmax_valueChanged()\0"
    "on_sliderVmin_valueChanged()\0"
    "on_sliderVmax_valueChanged()\0"
    "updateLoggingView()\0"
};

void ceilbot::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_start_button_clicked(); break;
        case 2: _t->on_quit_button_clicked(); break;
        case 3: _t->on_sliderHmin_valueChanged(); break;
        case 4: _t->on_sliderHmax_valueChanged(); break;
        case 5: _t->on_sliderSmin_valueChanged(); break;
        case 6: _t->on_sliderSmax_valueChanged(); break;
        case 7: _t->on_sliderVmin_valueChanged(); break;
        case 8: _t->on_sliderVmax_valueChanged(); break;
        case 9: _t->updateLoggingView(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData ceilbot::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ceilbot::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ceilbot__MainWindow,
      qt_meta_data_ceilbot__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ceilbot::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ceilbot::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ceilbot::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ceilbot__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ceilbot::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
