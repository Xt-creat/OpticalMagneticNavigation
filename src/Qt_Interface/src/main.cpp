#include "mainwindow.h"
#include <QApplication>
#include <CombinedApi.h>

Q_DECLARE_METATYPE(std::vector<ToolData>)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

	qRegisterMetaType<std::vector<ToolData>>("std::vector<ToolData>");

    MainWindow w;
	w.setWindowTitle(QObject::tr("CaptureOM-BIT"));
    w.show();

    return a.exec();
}
