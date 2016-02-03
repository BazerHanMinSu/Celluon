#ifndef QT_IMAGE_WIDGET_H_011
#define QT_IMAGE_WIDGET_H_011

#include <QWidget>
#include <cv.h>

class QImageWidget :public QWidget
{
	Q_OBJECT
public:
	QImageWidget(QWidget *parent=0);
	~QImageWidget(void);

	void SetShowImage(bool flag){
		m_ShowImage = flag;

	}

	void SetImage(QImage img){
		m_Image = img;
	}

	void setIndex(int i){
		m_Index = i;
	}
signals:
	void SigClickTrigger(int index);
protected:

	virtual void paintEvent ( QPaintEvent * ev);			// 화면에 다시 그려주는 이벤트
	virtual void mousePressEvent ( QMouseEvent * event );	// 마우스 버튼 클릭 이벤트
	virtual void 	mouseReleaseEvent ( QMouseEvent * event );

	bool m_ShowImage;
	QImage m_Image;

	
	bool m_MouseBtnClicked;
	bool m_FaceImageSelected;

	int m_Index;
};

#endif