#include "CommonHeader.h"

#define MAXHEIGHT 1081
#define MAXWIDTH 1921


class MappingTableController
{


public:
	MappingTableController();
	~MappingTableController();
	
	void InitValue();


	Mat		TemporaryDepthImage();							//임시 사용용 DepthImage 생성

	//Set User Location
	void SetUserLocation(Point3d inData);
	//Set Project Location
	void SetProjectorLocation(Point3d inData);



	//Pixel 좌표계를 이용하여 WorldCoordinate생성
	void TransPixelToWorldCoordiante(Mat inImg);		
	//void TransPixelToWorldCoordiante(Mat inImg, Point3f** outData);
	void InitUserLocation(Point3f inUserLct);



	//유저기준의 투사플레인 생성
	void CreateWorldPlaneFromUser();
	//유저기준의 VirtualScreen 생성
	void CreateVirtualScreenCoordinate();
	//가상평면 회전
	void RotateVirtualScreen();

	//가상평면의 바운더리 점 노멀라이즈
	void NormalizeVirtualScreen();

	//바운더리 영역 찾기
	void FindVirtualScreenBoundary();

	//바운더리 영역 내부점 marking
	void MakringPos();

	//바운더리 포인트 매칭하기
	void MatchingBoundaryMappingPoint();

	//포인트 매칭하기
	void MatchingMappingPoint();

	//매핑테이블 만들기
	void CreateMappingTable();

	//워핑진행
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
	Point3f **m_virtualScreen;				//가상평면
	Point3f m_virtualScreenBoundary[5];		//가상평면 바운더리
	Point3f **m_virtualScreenRotated;		//회전변환된 가상평면 (회색)
	Point3f m_virtualScreenBoundaryRotated[5]; //회전된 점	
	Point2f **tempCoordinate;				//유저관점의 평면(빨간색)
	Point2f m_userScreenFoundedBoundary[5];		//유저관점의 평면 바운더리


	bool **m_bMarkingPos;					//영역안에 점인지 marking

	MappingPos **m_matchedPoint;			//매칭된 점들의 정보를 저장
	Point2f m_BoundaryMathedPoint[5];		//매칭된 점정보
	Point2f **m_mappingTable;				//매핑테이블


private:
	double m_sizePerPixel;

	Point3d m_userLocation;		//유저위치
	Point3d m_projectorLoaction;	//프로젝터의 위치

	double m_boundary[4];

	//투사면의 World Coordinate
	


};