
#include <QApplication>
#include <QMainWindow>

#include "SmartProjectorMainWindow.h"
#include <qtextcodec.h>

int main(int argc, char *argv[])
{
    // �ѱ� ó���� ���� �ڵ�
	QTextCodec::setCodecForLocale(QTextCodec::codecForName("EUC-KR"));
    	
    // Qt���ø����̼� Ŭ���� ����
	QApplication a( argc, argv );
	
    // FisheyeDewarpingMainWindow ��ü ���� �Ҵ�
	SmartProjectorMainWindow fd_main;// = new CarInspectMainWindow;
	
	// ȭ�鿡 ���� ������ �����ֱ� 
	fd_main.show();

	// ���� �̺�Ʈ ����
	a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
	
	int rc = a.exec();

	// Application ����
    return rc;
}
