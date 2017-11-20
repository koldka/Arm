#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "displaywidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool isSurfacique();
    bool isVolumique();
    float getSliderValue();
    void setSliderValue(int val);
    Ui::MainWindow *ui;

private slots:
    void on_actionOuvrir_triggered();

    void on_actionSauvegarder_triggered();

    void on_actionVolumique_toggled(bool arg1);

    void on_actionSurfacique_toggled(bool arg1);

    void on_actionPersonnalise_toggled(bool arg1);

private:
    QLayout* layout;
    bool loaded = false;
    DisplayWidget* display;
};

#endif // MAINWINDOW_H
