
#include <QApplication>
#include <QMainWindow>

#include "SmartProjectorMainWindow.h"
#include <qtextcodec.h>

int main(int argc, char *argv[])
{
    // 한굴 처리를 위한 코덱
	QTextCodec::setCodecForLocale(QTextCodec::codecForName("EUC-KR"));
    	
    // Qt어플리케이션 클래스 선언
	QApplication a( argc, argv );
	
    // FisheyeDewarpingMainWindow 객체 동적 할당
	SmartProjectorMainWindow fd_main;// = new CarInspectMainWindow;
	
	// 화면에 메인 윈도우 보여주기 
	fd_main.show();

	// 종료 이벤트 연결
	a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
	
	int rc = a.exec();

	// Application 수행
    return rc;
}
