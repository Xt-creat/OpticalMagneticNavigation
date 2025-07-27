/********************************************************************************
** Form generated from reading UI file 'CalibrationDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.12.12
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CALIBRATIONDIALOG_H
#define UI_CALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Calibration
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *label;
    QLabel *label_2;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *startButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *recordButton;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *calculateButton;
    QSpacerItem *horizontalSpacer_4;

    void setupUi(QDialog *Calibration)
    {
        if (Calibration->objectName().isEmpty())
            Calibration->setObjectName(QString::fromUtf8("Calibration"));
        Calibration->resize(640, 480);
        buttonBox = new QDialogButtonBox(Calibration);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(460, 430, 180, 30));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        label = new QLabel(Calibration);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(110, 40, 400, 100));
        label_2 = new QLabel(Calibration);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(100, 180, 400, 100));
        horizontalLayoutWidget = new QWidget(Calibration);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(100, 320, 430, 80));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        startButton = new QPushButton(horizontalLayoutWidget);
        startButton->setObjectName(QString::fromUtf8("startButton"));
        startButton->setEnabled(true);

        horizontalLayout->addWidget(startButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        recordButton = new QPushButton(horizontalLayoutWidget);
        recordButton->setObjectName(QString::fromUtf8("recordButton"));

        horizontalLayout->addWidget(recordButton);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        calculateButton = new QPushButton(horizontalLayoutWidget);
        calculateButton->setObjectName(QString::fromUtf8("calculateButton"));

        horizontalLayout->addWidget(calculateButton);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);


        retranslateUi(Calibration);
        QObject::connect(buttonBox, SIGNAL(accepted()), Calibration, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), Calibration, SLOT(reject()));

        QMetaObject::connectSlotsByName(Calibration);
    } // setupUi

    void retranslateUi(QDialog *Calibration)
    {
        Calibration->setWindowTitle(QApplication::translate("Calibration", "Dialog", nullptr));
        label->setText(QApplication::translate("Calibration", "TextLabel", nullptr));
        label_2->setText(QApplication::translate("Calibration", "TextLabel", nullptr));
        startButton->setText(QApplication::translate("Calibration", "\345\274\200\345\247\213", nullptr));
        recordButton->setText(QApplication::translate("Calibration", "\350\256\260\345\275\225", nullptr));
        calculateButton->setText(QApplication::translate("Calibration", "\350\256\241\347\256\227", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Calibration: public Ui_Calibration {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CALIBRATIONDIALOG_H
