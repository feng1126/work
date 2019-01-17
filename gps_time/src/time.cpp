#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <memory>
#include <sstream>
#include "utm.h"

using namespace std;

struct KNGPSTIME
{
	int usYear;
	int byMonth;
	int byDay;
	int byHour;
	int byMinute;
	double fSecond;
};

struct gpsData
{
	double timestamp;
	double x, y, z;
	string state;
};

int UTC2GPS(KNGPSTIME stGPSTime, int iLeap, int &weekNo, double &secondOfweek);
int UTC2SEC(KNGPSTIME stGPSTime, int iLeap, int &weekNo, double &secondOfweek);
bool readGpsfile(std::string filename, vector<shared_ptr<gpsData>> &gps_all);
template <class T>
void convertFromString(T &value, const std::string &s);
bool saveGpsfile(std::string filename, vector<shared_ptr<gpsData>> &gps_all);

bool readGpsfile(std::string filename, vector<shared_ptr<gpsData>> &gps_all)
{
	ifstream gpsFile;

	gpsFile.open(filename.c_str());
	if (!gpsFile.is_open())
	{
		std::cout << "打开文件失败" << std::endl;
	};

	string line;
	while (gpsFile >> line)
	{

		gpsFile >> line;
		gpsFile >> line;
		shared_ptr<gpsData> gpsTemp(new gpsData);
		KNGPSTIME gpsTime;
		convertFromString(gpsTime.usYear, line);
		gpsFile >> line;
		convertFromString(gpsTime.byMonth, line);
		gpsFile >> line;
		convertFromString(gpsTime.byDay, line);
		gpsFile >> line;
		convertFromString(gpsTime.byHour, line);
		gpsFile >> line;
		convertFromString(gpsTime.byMinute, line);
		int sec, nsec;
		gpsFile >> line;
		convertFromString(sec, line);
		gpsFile >> line;
		convertFromString(nsec, line);
		gpsTime.fSecond = sec + nsec / 1000.0;
		int weak;
		double time;
		UTC2SEC(gpsTime, 0, weak, time);
		gpsTemp->timestamp = time;
		//printf("%lf \n", gpsTemp->timestamp);
		double longitude, latitude;
		gpsFile >> line;
		gpsFile >> line;

		// cout<< line<<endl;
		gpsFile >> line;
		// cout << line << endl;
		convertFromString(gpsTemp->z, line);
		gpsFile >> line;
		convertFromString(latitude, line);
		gpsFile >> line;
		convertFromString(longitude, line);
		UTM_coord coordTemp = from_latlon(longitude, latitude, 0, 0);

		gpsTemp->x = coordTemp.easting;
		gpsTemp->y = coordTemp.northing;
		gpsFile >> line;
		gpsFile >> line;
		gpsFile >> line;
		gpsFile >> line;
		gpsFile >> line;
		gpsFile >> line;
		gpsFile >> line;

		gpsTemp->state = line;
		gps_all.push_back(gpsTemp);
	}
	printf("%d \n" ,gps_all.size() );
	gpsFile.close();
}

bool saveGpsfile(std::string filename, vector<shared_ptr<gpsData>> &gps_all)
{

	ofstream outFile;

	outFile.open(filename.c_str());
	if (!outFile.is_open())
	{
		std::cout << "打开文件失败" << std::endl;
	};
	printf("%d \n", gps_all.size());
	for (int i = 0; i < gps_all.size(); i++)
	{
		outFile.precision(6);
		outFile.setf(ios::fixed, ios::floatfield);
		outFile << setprecision(6) << gps_all[i]->timestamp  << setprecision(6) << " " << gps_all[i]->x - gps_all[0]->x  << " "
				<< gps_all[i]->y - gps_all[0]->y << " " << gps_all[i]->z  - gps_all[0]->z << " " << gps_all[i]->state.c_str() << std::endl;
	}
	return true;
}

int UTC2GPS(KNGPSTIME stGPSTime, int iLeap, int &weekNo, double &secondOfweek)
{
	int year = stGPSTime.usYear;
	int month = stGPSTime.byMonth;
	int day = stGPSTime.byDay;
	int hour = stGPSTime.byHour;
	int minute = stGPSTime.byMinute;
	double second = stGPSTime.fSecond;

	/*****协调世界时转换为GPS的周秒表示*****/ //输入时间应为协调世界时，即当地时间-8，返回时间为GPS周和周秒
	int DayofYear = 0;
	int DayofMonth = 0;

	for (int i = 1970; i < year; i++) //从1980年到当前年的上一年经过的天数
	{
		if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
			DayofYear += 366;
		else
			DayofYear += 365;
	}
	for (int i = 1; i < month; i++) //从一月到当前月的上一月经历的天数
	{
		if (i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i == 12)
			DayofMonth += 31;
		else if (i == 4 || i == 6 || i == 9 || i == 11)
			DayofMonth += 30;
		else
		{
			if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
				DayofMonth += 29;
			else
				DayofMonth += 28;
		}
	}
	int Day;
	Day = DayofMonth + day + DayofYear - 6;
	weekNo = Day / 7;
	secondOfweek = Day % 7 * 86400 + hour * 3600 + minute * 60 + second + iLeap; // iLeap 为跳秒
	return 0;
}

int UTC2SEC(KNGPSTIME stGPSTime, int iLeap, int &weekNo, double &secondOfweek)
{
	int year = stGPSTime.usYear;
	int month = stGPSTime.byMonth;
	int day = stGPSTime.byDay;
	int hour = stGPSTime.byHour;
	int minute = stGPSTime.byMinute;
	double second = stGPSTime.fSecond;

	/*****协调世界时转换为GPS的周秒表示*****/ //输入时间应为协调世界时，即当地时间-8，返回时间为GPS周和周秒
	int DayofYear = 0;
	int DayofMonth = 0;

	for (int i = 1980; i < year; i++) //从1980年到当前年的上一年经过的天数
	{
		if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
			DayofYear += 366;
		else
			DayofYear += 365;
	}
	for (int i = 1; i < month; i++) //从一月到当前月的上一月经历的天数
	{
		if (i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i == 12)
			DayofMonth += 31;
		else if (i == 4 || i == 6 || i == 9 || i == 11)
			DayofMonth += 30;
		else
		{
			if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
				DayofMonth += 29;
			else
				DayofMonth += 28;
		}
	}
	int Day;
	Day = DayofMonth + day + DayofYear - 6;
	weekNo = Day / 7;
	secondOfweek = Day % 7 * 86400 + hour * 3600 + minute * 60 + second + iLeap; //iLeap表示跳秒
	secondOfweek = weekNo * 7 * 86400 + secondOfweek + 3657 * 86400;
	return 0;
}

template <class T>
void convertFromString(T &value, const std::string &s)
{
	std::stringstream ss(s);
	ss >> value;
}

int main(int argc, char **argv)
{

	if (argc != 3)
	{

		cerr << endl
			 << "Usage: ./app file in ,out " << endl;
		return 0;
	}
	ofstream gpsOut;
	vector<shared_ptr<gpsData>> gps_all;
	readGpsfile(argv[1], gps_all);
	saveGpsfile(argv[2], gps_all);

	for (int i = 0; i < gps_all.size(); i++)
	{
		//printf("%0.3lf %0.3lf %0.3lf %0.3lf %s \n", gps_all[i]->timestamp, gps_all[i]->x, gps_all[i]->y, gps_all[i]->z, gps_all[i]->state.c_str());
		//printf("%lf  \n  " ,time);
	}

	return 0;
}
