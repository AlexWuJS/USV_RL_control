#pragma once
#include <iostream>
using namespace std;
#include <vector>

class Point
{
public:
	Point(float longitude, float latitude);

	float m_longitude;	/* 经度 */
	float m_latitude;	/* 纬度 */
};

class TrackingPoints
{
public:
	/* 构造函数 */
	TrackingPoints();

	int NowPointIndex(void);	/* 返回当前跟踪点的vector索引 */	
	Point NowPoint(void);	/* 返回当前点 */
	void StartNextPoint(void);		/* 开始下一个点 */	
	void ShowAllPoints(void);	/* 打印当前需要输入的所有点 */
	void AddPoint(float longitude, float latitude);		/* 添加需要跟踪的点 */
	void AddPoints(vector<Point> points);
	void ClearPoints(void);		/* 清除所有点 */
private:
	int m_nowPoint;
	vector<Point> m_points;

};

/* -- Rfunctions ----------------------------------------------- */
