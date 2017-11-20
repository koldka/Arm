#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "filemanager.h"
#include "displaywidget.h"

#include <QFileDialog>
#include <QDir>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Tetkhaled");
    ui->slider->setValue(50);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool
MainWindow::isSurfacique()
{
    return this->ui->actionSurfacique->isChecked();
}

bool
MainWindow::isVolumique()
{
    return this->ui->actionVolumique->isChecked();
}

void MainWindow::on_actionOuvrir_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open File"), QDir::homePath(), tr("3D Files (*.pgm3d *.obj)"));
    if (!fileName.isEmpty())
    {
        if (this->loaded){
            ui->gridLayout->removeWidget(ui->gridLayoutWidget);
            this->display->deleteLater();
        }
        this->display = new DisplayWidget(fileName, this);
        ui->gridLayout->addWidget(display);
        this->loaded = true;
    }
}

float MainWindow::getSliderValue()
{
   return ui->slider->value();
}

void MainWindow::setSliderValue(int val)
{
    ui->slider->setValue(val);
}

void MainWindow::on_actionSauvegarder_triggered()
{
    if (this->loaded)
    {
        QString fileName = QFileDialog::getSaveFileName(this,
            tr("Save File"), QDir::homePath(), tr("Wavefront obj (*.obj)"));
        if (!fileName.isEmpty())
        {
            this->display->geometries->saveGeometry(fileName);
        }
    }
}

void MainWindow::on_actionVolumique_toggled(bool b)
{
    this->ui->actionSurfacique->setChecked(false);
    this->ui->actionPersonnalise->setChecked(false);
    this->ui->actionVolumique->setChecked(b);
    if(this->loaded){
        this->display->test();
        this->display->update();
    }
}

void MainWindow::on_actionSurfacique_toggled(bool b)
{
    this->ui->actionVolumique->setChecked(false);
    this->ui->actionPersonnalise->setChecked(false);
    this->ui->actionSurfacique->setChecked(b);
    if(this->loaded){
        this->display->test();
        this->display->update();
    }
}

void MainWindow::on_actionPersonnalise_toggled(bool b)
{
    this->ui->actionVolumique->setChecked(false);
    this->ui->actionSurfacique->setChecked(false);
    this->ui->actionPersonnalise->setChecked(b);
    if(this->loaded){
        this->display->test();
        this->display->update();
    }
}
