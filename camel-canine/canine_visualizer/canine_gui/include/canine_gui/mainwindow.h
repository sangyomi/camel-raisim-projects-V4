#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include <QTimer>
#include <QPen>
#include "canine_util/SharedMemory.hpp"
#include "canine_util/RobotDescription.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void DisplayUpdate();
    void GraphUpdate();
    void GraphInitialize();
    void on_BT_CAN_ON_clicked();
    void on_BT_VISUAL_ON_clicked();
    void on_BT_MOTOR_ON_clicked();
    void on_BT_MOTOR_OFF_clicked();
    void on_BT_SIMUL_ON_clicked();
    void on_BT_SIMUL_OFF_clicked();
    void on_BT_STAND_clicked();
    void on_BT_TROT_SLOW_clicked();
    void on_BT_TROT_FAST_clicked();
    void on_BT_OVERLAP_TROT_FAST_clicked();
    void on_BT_HOME_clicked();
    void on_BT_READY_clicked();
    void on_BT_CUSTOM1_clicked();
    void on_BT_CUSTOM2_clicked();

private:
    Ui::MainWindow *ui;
    QTimer      *displayTimer;
    QTimer		*graphTimer;
    double graphOffset;
    bool mIsUpdateGraph;
    void InitTable(QTableWidget *table);
    void InitLineEdit();
};
#endif // MAINWINDOW_H
