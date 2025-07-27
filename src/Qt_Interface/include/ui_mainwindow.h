/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.12
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
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout_3;
    QLabel *m_systemstatus;
    QLabel *m_OdataLabel;
    QLabel *m_MdataLabel;
    QLabel *m_FusionLabel;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *m_LinkBtn;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *m_CalibrateBtn;
    QSpacerItem *horizontalSpacer_6;
    QPushButton *m_TrackingBtn;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *m_StopBtn;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *m_DisplayBtn;
    QSpacerItem *horizontalSpacer_3;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(900, 600);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(800, 600));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        m_SelectSavePathBtn = new QPushButton(centralWidget);
        m_SelectSavePathBtn->setObjectName(QString::fromUtf8("m_SelectSavePathBtn"));

        horizontalLayout_3->addWidget(m_SelectSavePathBtn);

        m_DisplaySavePathLineEdit = new QLineEdit(centralWidget);
        m_DisplaySavePathLineEdit->setObjectName(QString::fromUtf8("m_DisplaySavePathLineEdit"));

        horizontalLayout_3->addWidget(m_DisplaySavePathLineEdit);

        m_DisplayModeComboBox = new QComboBox(centralWidget);
        m_DisplayModeComboBox->addItem(QString());
        m_DisplayModeComboBox->addItem(QString());
        m_DisplayModeComboBox->setObjectName(QString::fromUtf8("m_DisplayModeComboBox"));

        horizontalLayout_3->addWidget(m_DisplayModeComboBox);

        m_SizeComboBox = new QComboBox(centralWidget);
        m_SizeComboBox->addItem(QString());
        m_SizeComboBox->addItem(QString());
        m_SizeComboBox->setObjectName(QString::fromUtf8("m_SizeComboBox"));

        horizontalLayout_3->addWidget(m_SizeComboBox);

        m_SleepSpinBox = new QSpinBox(centralWidget);
        m_SleepSpinBox->setObjectName(QString::fromUtf8("m_SleepSpinBox"));

        horizontalLayout_3->addWidget(m_SleepSpinBox);


        verticalLayout->addLayout(horizontalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        m_systemstatus = new QLabel(centralWidget);
        m_systemstatus->setObjectName(QString::fromUtf8("m_systemstatus"));
        m_systemstatus->setMinimumSize(QSize(0, 0));

        verticalLayout_3->addWidget(m_systemstatus);


        verticalLayout_2->addLayout(verticalLayout_3);

        m_OdataLabel = new QLabel(centralWidget);
        m_OdataLabel->setObjectName(QString::fromUtf8("m_OdataLabel"));

        verticalLayout_2->addWidget(m_OdataLabel);

        m_MdataLabel = new QLabel(centralWidget);
        m_MdataLabel->setObjectName(QString::fromUtf8("m_MdataLabel"));

        verticalLayout_2->addWidget(m_MdataLabel);

        m_FusionLabel = new QLabel(centralWidget);
        m_FusionLabel->setObjectName(QString::fromUtf8("m_FusionLabel"));
        m_FusionLabel->setMaximumSize(QSize(16777215, 16777215));

        verticalLayout_2->addWidget(m_FusionLabel);


        verticalLayout->addLayout(verticalLayout_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(-1, 0, -1, -1);
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        m_LinkBtn = new QPushButton(centralWidget);
        m_LinkBtn->setObjectName(QString::fromUtf8("m_LinkBtn"));
        sizePolicy.setHeightForWidth(m_LinkBtn->sizePolicy().hasHeightForWidth());
        m_LinkBtn->setSizePolicy(sizePolicy);
        m_LinkBtn->setMinimumSize(QSize(160, 40));
        m_LinkBtn->setMaximumSize(QSize(160, 40));
        m_LinkBtn->setCheckable(true);

        horizontalLayout_2->addWidget(m_LinkBtn);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        m_CalibrateBtn = new QPushButton(centralWidget);
        m_CalibrateBtn->setObjectName(QString::fromUtf8("m_CalibrateBtn"));
        sizePolicy.setHeightForWidth(m_CalibrateBtn->sizePolicy().hasHeightForWidth());
        m_CalibrateBtn->setSizePolicy(sizePolicy);
        m_CalibrateBtn->setMinimumSize(QSize(160, 40));
        m_CalibrateBtn->setMaximumSize(QSize(160, 40));

        horizontalLayout_2->addWidget(m_CalibrateBtn);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_6);

        m_TrackingBtn = new QPushButton(centralWidget);
        m_TrackingBtn->setObjectName(QString::fromUtf8("m_TrackingBtn"));
        sizePolicy.setHeightForWidth(m_TrackingBtn->sizePolicy().hasHeightForWidth());
        m_TrackingBtn->setSizePolicy(sizePolicy);
        m_TrackingBtn->setMinimumSize(QSize(160, 40));
        m_TrackingBtn->setMaximumSize(QSize(160, 40));

        horizontalLayout_2->addWidget(m_TrackingBtn);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_4);

        m_StopBtn = new QPushButton(centralWidget);
        m_StopBtn->setObjectName(QString::fromUtf8("m_StopBtn"));
        m_StopBtn->setEnabled(true);
        m_StopBtn->setMinimumSize(QSize(160, 40));
        m_StopBtn->setMaximumSize(QSize(160, 40));

        horizontalLayout_2->addWidget(m_StopBtn);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_5);

        m_DisplayBtn = new QPushButton(centralWidget);
        m_DisplayBtn->setObjectName(QString::fromUtf8("m_DisplayBtn"));
        m_DisplayBtn->setEnabled(true);
        m_DisplayBtn->setMinimumSize(QSize(160, 40));

        horizontalLayout_2->addWidget(m_DisplayBtn);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalLayout->setStretch(1, 2);
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

        m_systemstatus->setText(QApplication::translate("MainWindow", "\347\263\273\347\273\237\347\212\266\346\200\201", nullptr));
        m_OdataLabel->setText(QApplication::translate("MainWindow", "\345\205\211\345\255\246\346\225\260\346\215\256", nullptr));
        m_MdataLabel->setText(QApplication::translate("MainWindow", "\347\224\265\347\243\201\346\225\260\346\215\256", nullptr));
        m_FusionLabel->setText(QApplication::translate("MainWindow", "\350\236\215\345\220\210\346\225\260\346\215\256", nullptr));
        m_LinkBtn->setText(QApplication::translate("MainWindow", "\350\277\236\346\216\245", nullptr));
        m_CalibrateBtn->setText(QApplication::translate("MainWindow", "\346\240\207\345\256\232", nullptr));
#ifndef QT_NO_SHORTCUT
        m_CalibrateBtn->setShortcut(QApplication::translate("MainWindow", "Return", nullptr));
#endif // QT_NO_SHORTCUT
        m_TrackingBtn->setText(QApplication::translate("MainWindow", "\350\267\237\350\270\252", nullptr));
#ifndef QT_NO_SHORTCUT
        m_TrackingBtn->setShortcut(QApplication::translate("MainWindow", "Space", nullptr));
#endif // QT_NO_SHORTCUT
        m_StopBtn->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242", nullptr));
        m_DisplayBtn->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
