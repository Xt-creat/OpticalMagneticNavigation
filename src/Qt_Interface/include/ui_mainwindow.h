/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *m_SelectSavePathBtn;
    QLineEdit *m_DisplaySavePathLineEdit;
    QComboBox *m_DisplayModeComboBox;
    QComboBox *m_SizeComboBox;
    QSpinBox *m_SleepSpinBox;
    QLabel *m_DisplayCaptureLabel;
    QVBoxLayout *verticalLayout_2;
    QLabel *m_DisplayCaptureFramesLabel;
    QLabel *m_DisplayCaptureTimeLabel;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *m_LinkBtn;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *m_StartBtn;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *m_StopBtn;
    QSpacerItem *horizontalSpacer_4;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(800, 600);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(800, 600));
        MainWindow->setMaximumSize(QSize(800, 600));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        m_SelectSavePathBtn = new QPushButton(centralWidget);
        m_SelectSavePathBtn->setObjectName(QStringLiteral("m_SelectSavePathBtn"));

        horizontalLayout_3->addWidget(m_SelectSavePathBtn);

        m_DisplaySavePathLineEdit = new QLineEdit(centralWidget);
        m_DisplaySavePathLineEdit->setObjectName(QStringLiteral("m_DisplaySavePathLineEdit"));

        horizontalLayout_3->addWidget(m_DisplaySavePathLineEdit);

        m_DisplayModeComboBox = new QComboBox(centralWidget);
        m_DisplayModeComboBox->addItem(QString());
        m_DisplayModeComboBox->addItem(QString());
        m_DisplayModeComboBox->setObjectName(QStringLiteral("m_DisplayModeComboBox"));

        horizontalLayout_3->addWidget(m_DisplayModeComboBox);

        m_SizeComboBox = new QComboBox(centralWidget);
        m_SizeComboBox->addItem(QString());
        m_SizeComboBox->addItem(QString());
        m_SizeComboBox->setObjectName(QStringLiteral("m_SizeComboBox"));

        horizontalLayout_3->addWidget(m_SizeComboBox);

        m_SleepSpinBox = new QSpinBox(centralWidget);
        m_SleepSpinBox->setObjectName(QStringLiteral("m_SleepSpinBox"));

        horizontalLayout_3->addWidget(m_SleepSpinBox);


        verticalLayout->addLayout(horizontalLayout_3);

        m_DisplayCaptureLabel = new QLabel(centralWidget);
        m_DisplayCaptureLabel->setObjectName(QStringLiteral("m_DisplayCaptureLabel"));
        m_DisplayCaptureLabel->setScaledContents(true);

        verticalLayout->addWidget(m_DisplayCaptureLabel);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        m_DisplayCaptureFramesLabel = new QLabel(centralWidget);
        m_DisplayCaptureFramesLabel->setObjectName(QStringLiteral("m_DisplayCaptureFramesLabel"));

        verticalLayout_2->addWidget(m_DisplayCaptureFramesLabel);

        m_DisplayCaptureTimeLabel = new QLabel(centralWidget);
        m_DisplayCaptureTimeLabel->setObjectName(QStringLiteral("m_DisplayCaptureTimeLabel"));

        verticalLayout_2->addWidget(m_DisplayCaptureTimeLabel);


        verticalLayout->addLayout(verticalLayout_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(-1, 0, -1, -1);
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        m_LinkBtn = new QPushButton(centralWidget);
        m_LinkBtn->setObjectName(QStringLiteral("m_LinkBtn"));
        sizePolicy.setHeightForWidth(m_LinkBtn->sizePolicy().hasHeightForWidth());
        m_LinkBtn->setSizePolicy(sizePolicy);
        m_LinkBtn->setMinimumSize(QSize(160, 40));
        m_LinkBtn->setMaximumSize(QSize(160, 40));
        m_LinkBtn->setCheckable(true);

        horizontalLayout_2->addWidget(m_LinkBtn);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        m_StartBtn = new QPushButton(centralWidget);
        m_StartBtn->setObjectName(QStringLiteral("m_StartBtn"));
        sizePolicy.setHeightForWidth(m_StartBtn->sizePolicy().hasHeightForWidth());
        m_StartBtn->setSizePolicy(sizePolicy);
        m_StartBtn->setMinimumSize(QSize(160, 40));
        m_StartBtn->setMaximumSize(QSize(160, 40));

        horizontalLayout_2->addWidget(m_StartBtn);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);

        m_StopBtn = new QPushButton(centralWidget);
        m_StopBtn->setObjectName(QStringLiteral("m_StopBtn"));
        sizePolicy.setHeightForWidth(m_StopBtn->sizePolicy().hasHeightForWidth());
        m_StopBtn->setSizePolicy(sizePolicy);
        m_StopBtn->setMinimumSize(QSize(160, 40));
        m_StopBtn->setMaximumSize(QSize(160, 40));

        horizontalLayout_2->addWidget(m_StopBtn);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_4);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalLayout->setStretch(1, 7);
        verticalLayout->setStretch(2, 2);
        verticalLayout->setStretch(3, 1);
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        m_SelectSavePathBtn->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251\344\277\235\345\255\230\350\267\257\345\276\204", nullptr));
        m_DisplayModeComboBox->setItemText(0, QApplication::translate("MainWindow", "\345\215\225\346\240\217", nullptr));
        m_DisplayModeComboBox->setItemText(1, QApplication::translate("MainWindow", "\345\217\214\346\240\217", nullptr));

        m_SizeComboBox->setItemText(0, QApplication::translate("MainWindow", "\346\234\200\344\275\263", nullptr));
        m_SizeComboBox->setItemText(1, QApplication::translate("MainWindow", "\345\205\250\345\261\217", nullptr));

        m_DisplayCaptureLabel->setText(QString());
        m_DisplayCaptureFramesLabel->setText(QApplication::translate("MainWindow", "\351\207\207\351\233\206xx\345\270\247", nullptr));
        m_DisplayCaptureTimeLabel->setText(QApplication::translate("MainWindow", "\347\224\250\346\227\266xx\347\247\222", nullptr));
        m_LinkBtn->setText(QApplication::translate("MainWindow", "\350\277\236\346\216\245", nullptr));
        m_StartBtn->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213", nullptr));
#ifndef QT_NO_SHORTCUT
        m_StartBtn->setShortcut(QApplication::translate("MainWindow", "Return", nullptr));
#endif // QT_NO_SHORTCUT
        m_StopBtn->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242", nullptr));
#ifndef QT_NO_SHORTCUT
        m_StopBtn->setShortcut(QApplication::translate("MainWindow", "Space", nullptr));
#endif // QT_NO_SHORTCUT
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
