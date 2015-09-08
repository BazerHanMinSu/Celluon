#include "CommonHeader.h"

#define MAXHEIGHT 1081
#define MAXWIDTH 1921


class MappingTableController
{


public:
	MappingTableController();
	~MappingTableController();
	
	void InitValue();


	Mat		TemporaryDepthImage();							//�ӽ� ���� DepthImage ����

	//Set User Location
	void SetUserLocation(Point3d inData);
	//Set Project Location
	void SetProjectorLocation(Point3d inData);



	//Pixel ��ǥ�踦 �̿��Ͽ� WorldCoordinate����
	void TransPixelToWorldCoordiante(Mat inImg);		
	//void TransPixelToWorldCoordiante(Mat inImg, Point3f** outData);
	void InitUserLocation(Point3f inUserLct);



	//���������� �����÷��� ����
	void CreateWorldPlaneFromUser();
	//���������� VirtualScreen ����
	void CreateVirtualScreenCoordinate();
	//������� ȸ��
	void RotateVirtualScreen();

	//��������� �ٿ���� �� ��ֶ�����
	void NormalizeVirtualScreen();

	//�ٿ���� ���� ã��
	void FindVirtualScreenBoundary();

	//�ٿ���� ���� ������ marking
	void MakringPos();

	//�ٿ���� ����Ʈ ��Ī�ϱ�
	void MatchingBoundaryMappingPoint();

	//����Ʈ ��Ī�ϱ�
	void MatchingMappingPoint();

	//�������̺� �����
	void CreateMappingTable();

	//��������
	void RunWarping();




	//point matching
	void Matching(int _maskSize = 10);	

	void Mapping();

	Point3f m_vectorFromUserToPlaneCenter;

	Point3f** returnWorldCoordinateForPlane();
	Point3f **m_worldCoordinateForPlane;
	Point3f m_worldCoordinateBoundary[5];	
	Point3f **m_worldCoordinateFromUser;
	Point3f m_worldCoordinateBoundaryFromUser[5];
	Point3f **m_virtualScreen;				//�������
	Point3f m_virtualScreenBoundary[5];		//������� �ٿ����
	Point3f **m_virtualScreenRotated;		//ȸ����ȯ�� ������� (ȸ��)
	Point3f m_virtualScreenBoundaryRotated[5]; //ȸ���� ��	
	Point2f **tempCoordinate;				//���������� ���(������)
	Point2f m_userScreenFoundedBoundary[5];		//���������� ��� �ٿ����


	bool **m_bMarkingPos;					//�����ȿ� ������ marking

	MappingPos **m_matchedPoint;			//��Ī�� ������ ������ ����
	Point2f m_BoundaryMathedPoint[5];		//��Ī�� ������
	Point2f **m_mappingTable;				//�������̺�


private:
	double m_sizePerPixel;

	Point3d m_userLocation;		//������ġ
	Point3d m_projectorLoaction;	//���������� ��ġ

	double m_boundary[4];

	//������� World Coordinate
	


};