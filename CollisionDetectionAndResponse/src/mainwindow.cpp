#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_gl=new NGLScene();
    ui->gridLayout->addWidget (m_gl,0,0,2,1);

    this->setWindowTitle ("Collision Detection and Response");


    //manual signal-slot connection
    connect(ui->pushButton, SIGNAL(clicked(bool)),  m_gl, SLOT(testButtonClicked(bool)) );
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    //auto signal-slot connection
    std::cout<<"Button Clicked - auto signal-slot connection"<<std::endl;
    m_gl->m_rColor=1;
    m_gl->update();
}

void MainWindow::on_restcoefSpinBox_valueChanged(double val)
{
    //auto signal-slot connection
    std::cout<<"Resitution Coeficient Value Changed - auto signal-slot connection"<<std::endl;

    NGLScene::e=val;
    m_gl->update ();
}
