#include "QImageWidget.h"
#include <qpainter.h>
#include <QPaintEvent>

QImageWidget::QImageWidget(QWidget *parent) : QWidget(parent){
	m_ShowImage = true;
	m_MouseBtnClicked = false;
//	m_FaceImageSelected = false;
	m_Index = 0;
}


QImageWidget::~QImageWidget(void)
{

}

void QImageWidget::paintEvent ( QPaintEvent * ev)			// ȭ�鿡 �ٽ� �׷��ִ� �̺�Ʈ
{
	if(m_ShowImage == false)
		return;

	if(m_Image.isNull())
		return;

	QPainter pt;
	pt.begin(this);
	QRect dirtyRect = ev->rect();
	pt.drawImage(dirtyRect, m_Image, m_Image.rect());

	if(m_MouseBtnClicked == true)
	{
		QPen pen;
		pen.setColor(QColor(255,255,0));
		pen.setWidth(4);
		pt.setPen(pen);
		pt.drawRect(2,2,this->width()-4,this->height()-4);
	}
	pt.end();
}

void QImageWidget::mousePressEvent ( QMouseEvent * event )	// ���콺 ��ư Ŭ�� �̺�Ʈ
{
	m_MouseBtnClicked = true;
	this->update();
}

void QImageWidget::mouseReleaseEvent ( QMouseEvent * event )
{
	m_MouseBtnClicked = false;
	this->update();
	emit SigClickTrigger(m_Index);

}