/********************************************************************************
** Form generated from reading UI file 'SmartProjectorMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SMARTPROJECTORMAINWINDOW_H
#define UI_SMARTPROJECTORMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qimagewidget.h"

QT_BEGIN_NAMESPACE

class Ui_CarInspectMainWindow
{
public:
    QAction *actionInspect;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_12;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_11;
    QFrame *frame_7;
    QVBoxLayout *verticalLayout_6;
    QFrame *frame_8;
    QHBoxLayout *horizontalLayout_9;
    QImageWidget *mVideoWidget1;
    QFrame *frame_3;
    QVBoxLayout *verticalLayout_2;
    QFrame *frame_4;
    QHBoxLayout *horizontalLayout_3;
    QImageWidget *mVideoWidget2;
    QHBoxLayout *horizontalLayout_10;
    QFrame *frame_5;
    QVBoxLayout *verticalLayout_5;
    QFrame *frame_6;
    QHBoxLayout *horizontalLayout_4;
    QImageWidget *mVideoWidget3;
    QFrame *frame_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QHBoxLayout *horizontalLayout;
    QImageWidget *mVideoWidget4;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_8;
    QLCDNumber *lcdNumber;
    QPushButton *mInspectStartBtn;
    QPushButton *mInspectEndBtn;
    QHBoxLayout *horizontalLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label;
    QDoubleSpinBox *mFovXSpinBox;
    QLabel *label_2;
    QDoubleSpinBox *mFovYSpinBox;
    QCheckBox *mAutoPlaneRotationCB;
    QCheckBox *mVerticalFlip;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_3;
    QDoubleSpinBox *mRotateXSpinBox;
    QLabel *label_4;
    QDoubleSpinBox *mRotateYSpinBox;
    QLabel *label_5;
    QDoubleSpinBox *mRotateZSpinBox;
    QCheckBox *mHomographyCB;
    QPlainTextEdit *mTextConsole;
    QHBoxLayout *horizontalLayout_7;
    QToolButton *mSaveBtn;
    QToolButton *mKinectCalibrationBtn;
    QToolButton *mCorrection;
    QStatusBar *m_statusBar;

    void setupUi(QMainWindow *CarInspectMainWindow)
    {
        if (CarInspectMainWindow->objectName().isEmpty())
            CarInspectMainWindow->setObjectName(QStringLiteral("CarInspectMainWindow"));
        CarInspectMainWindow->resize(1119, 574);
        actionInspect = new QAction(CarInspectMainWindow);
        actionInspect->setObjectName(QStringLiteral("actionInspect"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/whitecircle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionInspect->setIcon(icon);
        centralwidget = new QWidget(CarInspectMainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout_12 = new QHBoxLayout(centralwidget);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        frame_7 = new QFrame(centralwidget);
        frame_7->setObjectName(QStringLiteral("frame_7"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame_7->sizePolicy().hasHeightForWidth());
        frame_7->setSizePolicy(sizePolicy);
        frame_7->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        frame_7->setFrameShape(QFrame::Box);
        frame_7->setFrameShadow(QFrame::Plain);
        verticalLayout_6 = new QVBoxLayout(frame_7);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setSizeConstraint(QLayout::SetDefaultConstraint);
        frame_8 = new QFrame(frame_7);
        frame_8->setObjectName(QStringLiteral("frame_8"));
        sizePolicy.setHeightForWidth(frame_8->sizePolicy().hasHeightForWidth());
        frame_8->setSizePolicy(sizePolicy);
        frame_8->setStyleSheet(QStringLiteral("background-color: rgb(135, 135, 135);"));
        frame_8->setFrameShape(QFrame::Box);
        frame_8->setFrameShadow(QFrame::Plain);
        horizontalLayout_9 = new QHBoxLayout(frame_8);
        horizontalLayout_9->setSpacing(0);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        horizontalLayout_9->setContentsMargins(0, 0, 0, 0);
        mVideoWidget1 = new QImageWidget(frame_8);
        mVideoWidget1->setObjectName(QStringLiteral("mVideoWidget1"));

        horizontalLayout_9->addWidget(mVideoWidget1);


        verticalLayout_6->addWidget(frame_8);


        horizontalLayout_11->addWidget(frame_7);

        frame_3 = new QFrame(centralwidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        sizePolicy.setHeightForWidth(frame_3->sizePolicy().hasHeightForWidth());
        frame_3->setSizePolicy(sizePolicy);
        frame_3->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        frame_3->setFrameShape(QFrame::Box);
        frame_3->setFrameShadow(QFrame::Plain);
        verticalLayout_2 = new QVBoxLayout(frame_3);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        frame_4 = new QFrame(frame_3);
        frame_4->setObjectName(QStringLiteral("frame_4"));
        sizePolicy.setHeightForWidth(frame_4->sizePolicy().hasHeightForWidth());
        frame_4->setSizePolicy(sizePolicy);
        frame_4->setStyleSheet(QStringLiteral("background-color: rgb(135, 135, 135);"));
        frame_4->setFrameShape(QFrame::Box);
        frame_4->setFrameShadow(QFrame::Plain);
        horizontalLayout_3 = new QHBoxLayout(frame_4);
        horizontalLayout_3->setSpacing(0);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        mVideoWidget2 = new QImageWidget(frame_4);
        mVideoWidget2->setObjectName(QStringLiteral("mVideoWidget2"));

        horizontalLayout_3->addWidget(mVideoWidget2);


        verticalLayout_2->addWidget(frame_4);


        horizontalLayout_11->addWidget(frame_3);


        verticalLayout_7->addLayout(horizontalLayout_11);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        frame_5 = new QFrame(centralwidget);
        frame_5->setObjectName(QStringLiteral("frame_5"));
        sizePolicy.setHeightForWidth(frame_5->sizePolicy().hasHeightForWidth());
        frame_5->setSizePolicy(sizePolicy);
        frame_5->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        frame_5->setFrameShape(QFrame::Box);
        frame_5->setFrameShadow(QFrame::Plain);
        verticalLayout_5 = new QVBoxLayout(frame_5);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setSizeConstraint(QLayout::SetDefaultConstraint);
        frame_6 = new QFrame(frame_5);
        frame_6->setObjectName(QStringLiteral("frame_6"));
        sizePolicy.setHeightForWidth(frame_6->sizePolicy().hasHeightForWidth());
        frame_6->setSizePolicy(sizePolicy);
        frame_6->setStyleSheet(QStringLiteral("background-color: rgb(135, 135, 135);"));
        frame_6->setFrameShape(QFrame::Box);
        frame_6->setFrameShadow(QFrame::Plain);
        horizontalLayout_4 = new QHBoxLayout(frame_6);
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        mVideoWidget3 = new QImageWidget(frame_6);
        mVideoWidget3->setObjectName(QStringLiteral("mVideoWidget3"));

        horizontalLayout_4->addWidget(mVideoWidget3);


        verticalLayout_5->addWidget(frame_6);


        horizontalLayout_10->addWidget(frame_5);

        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        sizePolicy.setHeightForWidth(frame_2->sizePolicy().hasHeightForWidth());
        frame_2->setSizePolicy(sizePolicy);
        frame_2->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        frame_2->setFrameShape(QFrame::Box);
        frame_2->setFrameShadow(QFrame::Plain);
        verticalLayout = new QVBoxLayout(frame_2);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        frame = new QFrame(frame_2);
        frame->setObjectName(QStringLiteral("frame"));
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setStyleSheet(QStringLiteral("background-color: rgb(135, 135, 135);"));
        frame->setFrameShape(QFrame::Box);
        frame->setFrameShadow(QFrame::Plain);
        horizontalLayout = new QHBoxLayout(frame);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        mVideoWidget4 = new QImageWidget(frame);
        mVideoWidget4->setObjectName(QStringLiteral("mVideoWidget4"));

        horizontalLayout->addWidget(mVideoWidget4);


        verticalLayout->addWidget(frame);


        horizontalLayout_10->addWidget(frame_2);


        verticalLayout_7->addLayout(horizontalLayout_10);


        horizontalLayout_12->addLayout(verticalLayout_7);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        lcdNumber = new QLCDNumber(centralwidget);
        lcdNumber->setObjectName(QStringLiteral("lcdNumber"));
        lcdNumber->setFrameShape(QFrame::Panel);
        lcdNumber->setFrameShadow(QFrame::Raised);
        lcdNumber->setLineWidth(1);
        lcdNumber->setSmallDecimalPoint(true);
        lcdNumber->setSegmentStyle(QLCDNumber::Filled);

        horizontalLayout_8->addWidget(lcdNumber);

        mInspectStartBtn = new QPushButton(centralwidget);
        mInspectStartBtn->setObjectName(QStringLiteral("mInspectStartBtn"));

        horizontalLayout_8->addWidget(mInspectStartBtn);

        mInspectEndBtn = new QPushButton(centralwidget);
        mInspectEndBtn->setObjectName(QStringLiteral("mInspectEndBtn"));

        horizontalLayout_8->addWidget(mInspectEndBtn);


        verticalLayout_4->addLayout(horizontalLayout_8);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));

        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label = new QLabel(centralwidget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        label->setMinimumSize(QSize(42, 0));
        label->setMaximumSize(QSize(42, 16777215));
        label->setFrameShape(QFrame::Box);

        horizontalLayout_5->addWidget(label);

        mFovXSpinBox = new QDoubleSpinBox(centralwidget);
        mFovXSpinBox->setObjectName(QStringLiteral("mFovXSpinBox"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(mFovXSpinBox->sizePolicy().hasHeightForWidth());
        mFovXSpinBox->setSizePolicy(sizePolicy2);
        mFovXSpinBox->setMinimumSize(QSize(60, 0));
        mFovXSpinBox->setMaximumSize(QSize(50, 16777215));
        mFovXSpinBox->setDecimals(2);
        mFovXSpinBox->setMaximum(170);
        mFovXSpinBox->setSingleStep(0.1);
        mFovXSpinBox->setValue(75);

        horizontalLayout_5->addWidget(mFovXSpinBox);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy1.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy1);
        label_2->setMinimumSize(QSize(40, 0));
        label_2->setMaximumSize(QSize(42, 16777215));
        label_2->setFrameShape(QFrame::Box);

        horizontalLayout_5->addWidget(label_2);

        mFovYSpinBox = new QDoubleSpinBox(centralwidget);
        mFovYSpinBox->setObjectName(QStringLiteral("mFovYSpinBox"));
        sizePolicy2.setHeightForWidth(mFovYSpinBox->sizePolicy().hasHeightForWidth());
        mFovYSpinBox->setSizePolicy(sizePolicy2);
        mFovYSpinBox->setMinimumSize(QSize(60, 0));
        mFovYSpinBox->setMaximumSize(QSize(50, 16777215));
        mFovYSpinBox->setDecimals(2);
        mFovYSpinBox->setMaximum(170);
        mFovYSpinBox->setSingleStep(0.1);
        mFovYSpinBox->setValue(65);

        horizontalLayout_5->addWidget(mFovYSpinBox);

        mAutoPlaneRotationCB = new QCheckBox(centralwidget);
        mAutoPlaneRotationCB->setObjectName(QStringLiteral("mAutoPlaneRotationCB"));
        sizePolicy2.setHeightForWidth(mAutoPlaneRotationCB->sizePolicy().hasHeightForWidth());
        mAutoPlaneRotationCB->setSizePolicy(sizePolicy2);
        mAutoPlaneRotationCB->setMinimumSize(QSize(85, 0));
        mAutoPlaneRotationCB->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_5->addWidget(mAutoPlaneRotationCB);

        mVerticalFlip = new QCheckBox(centralwidget);
        mVerticalFlip->setObjectName(QStringLiteral("mVerticalFlip"));
        sizePolicy2.setHeightForWidth(mVerticalFlip->sizePolicy().hasHeightForWidth());
        mVerticalFlip->setSizePolicy(sizePolicy2);
        mVerticalFlip->setMinimumSize(QSize(60, 0));
        mVerticalFlip->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_5->addWidget(mVerticalFlip);


        verticalLayout_4->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy1.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy1);
        label_3->setMinimumSize(QSize(123, 0));
        label_3->setMaximumSize(QSize(90, 16777215));
        label_3->setFrameShape(QFrame::Box);

        horizontalLayout_6->addWidget(label_3);

        mRotateXSpinBox = new QDoubleSpinBox(centralwidget);
        mRotateXSpinBox->setObjectName(QStringLiteral("mRotateXSpinBox"));
        mRotateXSpinBox->setMinimumSize(QSize(60, 0));
        mRotateXSpinBox->setMaximumSize(QSize(60, 16777215));
        mRotateXSpinBox->setDecimals(4);
        mRotateXSpinBox->setMinimum(-80);
        mRotateXSpinBox->setMaximum(80);
        mRotateXSpinBox->setSingleStep(1);
        mRotateXSpinBox->setValue(0);

        horizontalLayout_6->addWidget(mRotateXSpinBox);

        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy1.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy1);
        label_4->setMinimumSize(QSize(28, 0));
        label_4->setMaximumSize(QSize(28, 16777215));
        label_4->setFrameShape(QFrame::Box);

        horizontalLayout_6->addWidget(label_4);

        mRotateYSpinBox = new QDoubleSpinBox(centralwidget);
        mRotateYSpinBox->setObjectName(QStringLiteral("mRotateYSpinBox"));
        mRotateYSpinBox->setMinimumSize(QSize(60, 0));
        mRotateYSpinBox->setMaximumSize(QSize(60, 16777215));
        mRotateYSpinBox->setDecimals(4);
        mRotateYSpinBox->setMinimum(-80);
        mRotateYSpinBox->setMaximum(80);
        mRotateYSpinBox->setSingleStep(1);
        mRotateYSpinBox->setValue(0);

        horizontalLayout_6->addWidget(mRotateYSpinBox);

        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy1.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy1);
        label_5->setMinimumSize(QSize(28, 0));
        label_5->setMaximumSize(QSize(28, 16777215));
        label_5->setFrameShape(QFrame::Box);

        horizontalLayout_6->addWidget(label_5);

        mRotateZSpinBox = new QDoubleSpinBox(centralwidget);
        mRotateZSpinBox->setObjectName(QStringLiteral("mRotateZSpinBox"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(60);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(mRotateZSpinBox->sizePolicy().hasHeightForWidth());
        mRotateZSpinBox->setSizePolicy(sizePolicy3);
        mRotateZSpinBox->setMaximumSize(QSize(60, 16777215));
        mRotateZSpinBox->setDecimals(4);
        mRotateZSpinBox->setMinimum(-80);
        mRotateZSpinBox->setMaximum(80);
        mRotateZSpinBox->setSingleStep(1);
        mRotateZSpinBox->setValue(1.5);

        horizontalLayout_6->addWidget(mRotateZSpinBox);


        verticalLayout_4->addLayout(horizontalLayout_6);

        mHomographyCB = new QCheckBox(centralwidget);
        mHomographyCB->setObjectName(QStringLiteral("mHomographyCB"));

        verticalLayout_4->addWidget(mHomographyCB);

        mTextConsole = new QPlainTextEdit(centralwidget);
        mTextConsole->setObjectName(QStringLiteral("mTextConsole"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(mTextConsole->sizePolicy().hasHeightForWidth());
        mTextConsole->setSizePolicy(sizePolicy4);
        mTextConsole->setMinimumSize(QSize(390, 0));

        verticalLayout_4->addWidget(mTextConsole);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(2);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        mSaveBtn = new QToolButton(centralwidget);
        mSaveBtn->setObjectName(QStringLiteral("mSaveBtn"));
        mSaveBtn->setEnabled(false);
        mSaveBtn->setMinimumSize(QSize(100, 80));
        mSaveBtn->setMaximumSize(QSize(100, 80));

        horizontalLayout_7->addWidget(mSaveBtn);

        mKinectCalibrationBtn = new QToolButton(centralwidget);
        mKinectCalibrationBtn->setObjectName(QStringLiteral("mKinectCalibrationBtn"));
        mKinectCalibrationBtn->setMinimumSize(QSize(140, 80));
        mKinectCalibrationBtn->setMaximumSize(QSize(140, 80));

        horizontalLayout_7->addWidget(mKinectCalibrationBtn);

        mCorrection = new QToolButton(centralwidget);
        mCorrection->setObjectName(QStringLiteral("mCorrection"));
        mCorrection->setMinimumSize(QSize(140, 80));
        mCorrection->setMaximumSize(QSize(140, 80));

        horizontalLayout_7->addWidget(mCorrection);


        verticalLayout_4->addLayout(horizontalLayout_7);


        horizontalLayout_12->addLayout(verticalLayout_4);

        CarInspectMainWindow->setCentralWidget(centralwidget);
        m_statusBar = new QStatusBar(CarInspectMainWindow);
        m_statusBar->setObjectName(QStringLiteral("m_statusBar"));
        m_statusBar->setSizeGripEnabled(false);
        CarInspectMainWindow->setStatusBar(m_statusBar);

        retranslateUi(CarInspectMainWindow);

        QMetaObject::connectSlotsByName(CarInspectMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *CarInspectMainWindow)
    {
        CarInspectMainWindow->setWindowTitle(QApplication::translate("CarInspectMainWindow", "Kinect Projector Calibrator by v0.1", 0));
        actionInspect->setText(QApplication::translate("CarInspectMainWindow", "Inspect", 0));
        mInspectStartBtn->setText(QApplication::translate("CarInspectMainWindow", "Play", 0));
        mInspectEndBtn->setText(QApplication::translate("CarInspectMainWindow", "Stop", 0));
        label->setText(QApplication::translate("CarInspectMainWindow", "FOV x  ", 0));
        label_2->setText(QApplication::translate("CarInspectMainWindow", "FOV y  ", 0));
        mAutoPlaneRotationCB->setText(QApplication::translate("CarInspectMainWindow", "Auto Rotate", 0));
        mVerticalFlip->setText(QApplication::translate("CarInspectMainWindow", "v flip", 0));
        label_3->setText(QApplication::translate("CarInspectMainWindow", "PlaneRotatation Rx:", 0));
        label_4->setText(QApplication::translate("CarInspectMainWindow", "Ry:  ", 0));
        label_5->setText(QApplication::translate("CarInspectMainWindow", "Rz:", 0));
        mHomographyCB->setText(QApplication::translate("CarInspectMainWindow", "Homography", 0));
        mSaveBtn->setText(QApplication::translate("CarInspectMainWindow", "\354\240\200\354\236\245", 0));
        mKinectCalibrationBtn->setText(QApplication::translate("CarInspectMainWindow", "Proejctor & Kinect \n"
"Calibration", 0));
        mCorrection->setText(QApplication::translate("CarInspectMainWindow", "\353\263\264\354\240\225 \354\210\230\355\226\211", 0));
    } // retranslateUi

};

namespace Ui {
    class CarInspectMainWindow: public Ui_CarInspectMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SMARTPROJECTORMAINWINDOW_H
