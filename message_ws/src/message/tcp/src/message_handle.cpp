#include <string>
#include "tcp/message_handle.h"


class MessageHandleClass()
{
public:
	FormatConversionClass FormatConversion;
private:



}


/* -Class Point Definition ----------------------------*/
Point::Point(float longitude, float latitude)
{	
	m_longitude = longitude;
	m_latitude = latitude;
}
/* -End of Class Point Definition ---------------------*/

/* -Class TrackingPoints Definition -------------------*/
/* 构造函数 */
TrackingPoints::TrackingPoints()
{
	m_nowPoint = 0;
}

/* 返回当前跟踪点的vector索引 */
int TrackingPoints::NowPointIndex(void)
{
	return m_nowPoint;
}

/* 返回当前点 */
Point TrackingPoints::NowPoint(void)
{
	return m_points[m_nowPoint];
}

/* 开始下一个点 */
void TrackingPoints::StartNextPoint(void)
{
	m_nowPoint++;
}

/* 打印当前需要输入的所有点 */
void TrackingPoints::ShowAllPoints(void)
{
	for (vector<Point>::iterator it = m_points.begin(); it != m_points.end(); it++) {
		cout << "longtitude: " << it->m_longitude << endl;
		cout << "latitude: " << it->m_latitude << endl << endl;
	}
}

/* 添加需要跟踪的点 */
void TrackingPoints::AddPoint(float longitude, float latitude)
{
	Point point(longitude, latitude);

	m_points.push_back(point);
}

void TrackingPoints::AddPoints(vector<Point> points)
{
	for (vector<Point>::iterator it = points.begin(); it != points.end(); it++) {
		m_points.push_back(*it);
	}
}

/* 清除所有点 */
void TrackingPoints::ClearPoints(void)
{
	m_points.clear();
}
/* -End of Class TrackingPoints Definition -------------------*/