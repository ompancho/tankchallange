// TankChallange.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <iostream>

using namespace std;

class Pos
{
public:
	Pos() {}
	Pos(int x_, int y_) : x(x_), y(y_), bHighPriority(false) { }
	int x, y;
	bool bHighPriority;
};
inline bool operator==(const Pos& p1, const Pos& p2) { return p1.x == p2.x && p1.y == p2.y; }

//
class Map
{
public:
	Map()
		: angle(0)
		, currentPos(0, 0)
	{
		//initial size 1x1
		for (int i = 0; i < 1; i++)
		{
			vector<int> v = { 0 };
			mapData.push_back(v);
		}
	}


	enum { unknown = 0, scanned_empty = 1, scanned_unknown = 2, enemy = 3, wall = 4 };
	enum { front_ = 0, right_ = 1, back_ = 2, left_ = 3 };
	enum { moveForward = 1, moveBackward, turnRight, turnLeft };

public:
	/*
	updateMap handles :
	-resizeing of the map based on new data
	-updating the map data with new objects
	-detecting enemies
	-setting waypoints
	(how waypoints are added)
	-on resize set the corner of the new map as waypoint, try to add use opposite diagonal corners
	-if enemy was detected passing paralell to you, set it as waypoint
	-note! no waypoint will be added outside the "known world", there will be no unreachable waypoints
	(how waypoints are removed)
	-if a waypoint is put on a wall then it's removed from the list once it's deteced it's a wall
	-by reaching the waypoint
	-finding the shortest path to the waypoint, call function getNextStep() to get the next command
	*/
	void updateMap(int lidarFront, int lidarRight, int lidarBack, int lidarLeft, bool target)
	{
#define GETDISTANCE(a,b) (int)sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))
#define UPDATE_MAP_VALUE(d,v) if((d) != wall && (d) != enemy) (d) = (v)

		//resize memory behind the map
		resize(lidarFront, lidarRight, lidarBack, lidarLeft);

		Pos & p = currentPos;
		//update map with new lidar data
		vector<int> distance; distance.resize(4);
		vector<int> lidarData; lidarData.resize(4);

		distance[(front_ + angle) % 4] = lidarFront;
		distance[(right_ + angle) % 4] = lidarRight;
		distance[(back_ + angle) % 4] = lidarBack;
		distance[(left_ + angle) % 4] = lidarLeft;

		lidarData[(front_ + angle) % 4] = target ? enemy : wall;
		lidarData[(right_ + angle) % 4] = scanned_unknown;
		lidarData[(back_ + angle) % 4] = scanned_unknown;
		lidarData[(left_ + angle) % 4] = scanned_unknown;

		int dDistanceFront = distance[front_];
		int dDistanceRight = distance[right_];
		int dDistanceBack = distance[back_];
		int dDistanceLeft = distance[left_];

		for (int i = 1; i < dDistanceFront; i++)
		{
			mapData[p.x][p.y + i] = scanned_empty;
		}
		UPDATE_MAP_VALUE(mapData[p.x][p.y + dDistanceFront], lidarData[front_]);

		for (int i = 1; i < dDistanceRight; i++)
		{
			mapData[p.x + i][p.y] = scanned_empty;
		}
		UPDATE_MAP_VALUE(mapData[p.x + dDistanceRight][p.y], lidarData[right_]);

		for (int i = 1; i < dDistanceBack; i++)
		{
			mapData[p.x][p.y - i] = scanned_empty;
		}
		UPDATE_MAP_VALUE(mapData[p.x][p.y - dDistanceBack], lidarData[back_]);

		for (int i = 1; i < dDistanceLeft; i++)
		{
			mapData[p.x - i][p.y] = scanned_empty;
		}
		UPDATE_MAP_VALUE(mapData[p.x - dDistanceLeft][p.y], lidarData[left_]);

		//TODO - setting waypoints 
		//addWayPoint(p);

		//remove waypoint if we have deteced as wall, but only if the distance to the waypoint from current position is 1 step (that way we don't delete it prematurely)
		if (waypoints.size())
		{
			if (GETDISTANCE(waypoints[0], currentPos) < 2)
			{
				if (mapData[waypoints[0].x][waypoints[0].y] == wall)
				{
					waypoints.erase(waypoints.begin());
				}
			}
		}

		//update the next step to take
		updatePath();
	}

	void move(int n)
	{
		switch (angle)
		{
		case 0: currentPos.y += n; break;
		case 1: currentPos.x += n; break;
		case 2: currentPos.y += -n; break;
		case 3: currentPos.x += -n; break;
		}

		//remove waypoint if we have reached it
		if (waypoints.size())
		{
			if (currentPos == waypoints[0])
			{
				waypoints.erase(waypoints.begin());
			}
		}
	}

	void setangle(int a)
	{
		angle += a;
		angle = angle < 0 ? angle + 4 : angle;
		angle %= 4;
	}

	int getNextStep()
	{
		//decide what command to exec based on currentPos, nextPos and angle
		vector<int> vmove; vmove.resize(4);
		vmove[(front_ + angle) % 4] = moveForward;
		vmove[(right_ + angle) % 4] = turnRight;
		vmove[(back_ + angle) % 4] = moveBackward;
		vmove[(left_ + angle) % 4] = turnLeft;

		if ((currentPos.x - nextPos.x) < 0) return vmove[right_];
		if ((currentPos.x - nextPos.x) > 0) return vmove[left_];
		if ((currentPos.y - nextPos.y) < 0) return vmove[front_];
		if ((currentPos.y - nextPos.y) > 0) return vmove[back_];
		return 0;
	}

	void info()
	{
		cout << "-------------------------------------------------------" << endl;
		cout << "angle:" << angle << endl;
		cout << "pos:" << currentPos.x << "," << currentPos.y << endl;

		cout << "-------------------------------------------------------" << endl;
		cout << "printing map" << endl;
		cout << "-------------------------------------------------------" << endl;
		int x = 0;
		int y = 0;
		for (y = mapData[0].size(); y > 0; y--)
		{
			for (x = 0; x < (int)mapData.size(); x++)
			{
				if (currentPos.x == x && currentPos.y == y - 1)
				{
					switch (angle)
					{
					case front_: cout << "^"; break;
					case right_: cout << ">"; break;
					case back_: cout << "v"; break;
					case left_: cout << "<"; break;
					}
					continue;
				}
				int v = mapData[x][y - 1];
				switch (v)
				{
				case unknown: cout << " "; break;
				case scanned_empty: cout << "."; break;
				case wall: cout << "#"; break;
				case enemy: cout << "E"; break;
				case scanned_unknown: cout << "?"; break;
				default: cout << "'" << v << "'";
				}
			}
			cout << endl;
		}

		//compare to existing
		/*cout << "-------------------------------------------------------" << endl;
		for (y = mapData[0].size(); y > 0; y--)
		{
			for (x = 0; x < (int)mapData.size(); x++)
			{
				int v = mapData[x][y - 1];
				int v = mapData[x][y - 1];
			}
		}
		cout << "-------------------------------------------------------";*/
		cout << "nWidth:" << mapData.size() << ", nHeight:" << mapData[0].size() << endl;

	}

	bool DiscoveryDone()
	{
		return waypoints.size() ? false : true;
	}

private:
	void resize(int lidarFront, int lidarRight, int lidarBack, int lidarLeft)
	{
		int nWidth = mapData.size();
		int nHeight = mapData[0].size();

		int distanceFront = nHeight - currentPos.y;
		int distanceRight = nWidth - currentPos.x;
		int distanceBack = nHeight - distanceFront + 1;
		int distanceLeft = nWidth - distanceRight + 1;

		int distance[] = { distanceFront,distanceRight,distanceBack,distanceLeft };

		//cout << "" << f << "," << r << "," << b << "," << l << endl;
		//cout << "" << f_ << "," << r_ << "," << b_ << "," << l_ << endl;

		int dDistanceFront = lidarFront - distance[(front_ + angle) % 4] + 1;
		int dDistanceRight = lidarRight - distance[(right_ + angle) % 4] + 1;
		int dDistanceBack = lidarBack - distance[(back_ + angle) % 4] + 1;
		int dDistanceLeft = lidarLeft - distance[(left_ + angle) % 4] + 1;

		// Note! negative values indicate that your view is abstracted by a wall or an enemy

		distance[(front_ + angle) % 4] = dDistanceFront;
		distance[(right_ + angle) % 4] = dDistanceRight;
		distance[(back_ + angle) % 4] = dDistanceBack;
		distance[(left_ + angle) % 4] = dDistanceLeft;

		dDistanceFront = distance[front_];
		dDistanceRight = distance[right_];
		dDistanceBack = distance[back_];
		dDistanceLeft = distance[left_];
		//cout << "" << dDistanceFront << "," << dDistanceRight << "," << dDistanceBack << "," << dDistanceLeft << endl;

		//resize width
		for (int i = 0; i < dDistanceRight; i++)
		{
			vector<int> v;
			v.resize(mapData[0].size());
			mapData.push_back(v);
		}
		for (int i = 0; i < dDistanceLeft; i++)
		{
			vector<int> v;
			v.resize(mapData[0].size());
			mapData.insert(mapData.begin(), v);
		}
		//resize front back
		for (int j = 0; j < (int)mapData.size(); j++)
		{
			for (int i = 0; i < dDistanceFront; i++)
			{
				mapData[j].push_back(0);
			}
			for (int i = 0; i < dDistanceBack; i++)
			{
				mapData[j].insert(mapData[j].begin(), 0);
			}
		}

		//update current position and all waypoints
		if (dDistanceBack > 0)
		{
			currentPos.y += dDistanceBack;
			for (int i = 0; i < (int)waypoints.size(); i++) 
				waypoints[i].y += dDistanceBack;
		}
		if (dDistanceLeft > 0)
		{
			currentPos.x += dDistanceLeft;
			for (int i = 0; i < (int)waypoints.size(); i++) 
				waypoints[i].x += dDistanceLeft;
		}

		//add new corners as waypoints, add them diagonaly
		nWidth = mapData.size();
		nHeight = mapData[0].size();

		if (dDistanceFront > 0)
			addWayPoint(Pos(0, nHeight - 2), false);
		if (dDistanceBack > 0)
			addWayPoint(Pos(nWidth - 2, nHeight - 2), false);
		if (dDistanceRight > 0)
			addWayPoint(Pos(nWidth - 2, 0), false);
		if (dDistanceLeft > 0)
			addWayPoint(Pos(0, 0), false);

		// TODO - analyze new lidar data for enemy movment, do cost analyses,  set waypoints

	}

	void updatePath()
	{
#define FRONT(p) Pos(p.x,p.y+1)
#define RIGHT(p) Pos(p.x,p.y-1)
#define BACK(p) Pos(p.x+1,p.y)
#define LEFT(p) Pos(p.x-1,p.y)
#define GETVAL(p) ((0 <= p.x && p.x < nWidth) && (0 <= p.y && p.y < nHeight)) ?  vpathfinder[p.x][p.y] : wall
#define SETVAL(p,v) vpathfinder[p.x][p.y] = v

		if (!waypoints.size()) return;

		int nWidth = mapData.size();
		int nHeight = mapData[0].size();

		vector<vector<int>> vpathfinder = mapData;
		/*for (int i = 0; i < nWidth; i++)
		{
		vector<int> v = mapData[i];
		vpathfinder.push_back(v);
		}*/

		Pos start = waypoints[0];
		Pos end = currentPos;
		bool bFound = false;
		bool bDeadEnd = false;
		vector<Pos> vtemp;
		SETVAL(start, -1);
		Pos cur = start;
		while (!bFound)
		{
			int n = GETVAL(cur);
			if (cur == end)
			{
				bFound = true;
				//cout << "Path Found - step count:" << n << endl;
				continue;
			}

			Pos p0 = FRONT(cur);
			Pos p1 = RIGHT(cur);
			Pos p2 = BACK(cur);
			Pos p3 = LEFT(cur);

			int p0_ = GETVAL(p0);
			int p1_ = GETVAL(p1);
			int p2_ = GETVAL(p2);
			int p3_ = GETVAL(p3);

			if (p0_ == unknown || p0_ == scanned_empty) { vtemp.push_back(p0); SETVAL(p0, n - 1); }
			if (p1_ == unknown || p1_ == scanned_empty) { vtemp.push_back(p1); SETVAL(p1, n - 1); }
			if (p2_ == unknown || p2_ == scanned_empty) { vtemp.push_back(p2); SETVAL(p2, n - 1); }
			if (p3_ == unknown || p3_ == scanned_empty) { vtemp.push_back(p3); SETVAL(p3, n - 1); }

			if (vtemp.size())
			{
				cur = vtemp[0];
				vtemp.erase(vtemp.begin());
			}
			else
			{
				info();
				cout << "Dead end, can't get to it!!!" << endl;
				bFound = true;
				bDeadEnd = true;
			}
		}

		//debug
		/*cout << "***********************************" << endl;
		cout << "printing path map" << endl;
		int x = 0;
		int y = 0;
		for (y = vpathfinder[0].size(); y > 0; y--)
		{
		for (x = 0; x < (int)vpathfinder.size(); x++)
		{
		cout << vpathfinder[x][y - 1];
		}
		cout << endl;
		}*/

		//
		vector<Pos> vpath;
		vpath.push_back(end);
		vtemp.clear();
		cur = end;
		bFound = false;
		while (!bFound)
		{
			int n = GETVAL(cur);
			if (cur == start)
			{
				bFound = true;
				//cout << "Path Found - Step count:" << n << endl;
				continue;
			}

			Pos p0 = FRONT(cur);
			Pos p1 = RIGHT(cur);
			Pos p2 = BACK(cur);
			Pos p3 = LEFT(cur);

			int p0_ = GETVAL(p0); if (p0_ == n + 1) { n = p0_; vpath.push_back(p0); }
			int p1_ = GETVAL(p1); if (p1_ == n + 1) { n = p1_; vpath.push_back(p1); }
			int p2_ = GETVAL(p2); if (p2_ == n + 1) { n = p2_; vpath.push_back(p2); }
			int p3_ = GETVAL(p3); if (p3_ == n + 1) { n = p3_; vpath.push_back(p3); }

			if (vpath.size())
			{
				cur = vpath[vpath.size() - 1];
			}
		}

		nextPos = vpath[1];
		//debug
		/*for (int i = 0; i < (int)vpath.size(); i++)
		{
		cout << "step-" << i << " - " << vpath[i].x << "," << vpath[i].y << endl;
		}*/
	}

	void addWayPoint(Pos p, bool bHighPriority)
	{
		if (bHighPriority)
		{
			//this prevents overwriting of any other high priority waypoint
			int offset = 0;
			for (offset = 0; offset < (int)waypoints.size(); offset++)
			{
				if (!waypoints[offset].bHighPriority) break;
			}
			waypoints.insert(waypoints.begin() + offset, p);
		}
		else
		{
			waypoints.push_back(p);
		}
	}

	std::vector<std::vector<int>> mapData;
	int angle;
	Pos currentPos, nextPos; //x,y
	vector<Pos> waypoints;
};

struct simScanResults { int f; int b; int l; int r; bool enemy; int angle; };

vector<vector<int>> g_simMap;

void LoadMapData(string s)
{
	vector<int> v;
	for (int i = 0; i < (int)s.length(); i++)
	{
		char ch = s.at(i);
		switch (ch)
		{
		case '#': v.push_back(Map::wall); break;
		case '.':
		case 'S':v.push_back(Map::scanned_empty); break;
		case 'E': v.push_back(Map::enemy); break;
		}
	}
	g_simMap.push_back(v);
}

int simScan(Pos p, int angle, bool &bEnemy)
{
	int nStep = 0;
	int v = Map::unknown;
	while (v != Map::wall && v != Map::enemy)
	{
		nStep++;
		switch (angle)
		{
		case Map::front_: v = g_simMap[p.x][p.y + nStep]; break;
		case Map::right_: v = g_simMap[p.x + nStep][p.y]; break;
		case Map::back_: v = g_simMap[p.x][p.y - nStep]; break;
		case Map::left_: v = g_simMap[p.x - nStep][p.y]; break;
		}
	}
	bEnemy = (v == Map::enemy);
	return nStep;
}

void simGetScan(Pos currentPos, simScanResults & sr, int angle)
{
	bool bEnemyF = false, bEnemyR = false, bEnemyB = false, bEnemyL = false;
	int ascan[] = { simScan(currentPos,Map::front_,bEnemyF), simScan(currentPos,Map::right_,bEnemyR), simScan(currentPos,Map::back_,bEnemyB), simScan(currentPos,Map::left_,bEnemyL) };
	bool aenemy[] = { bEnemyF, bEnemyR, bEnemyB, bEnemyL };
	sr.f = ascan[(Map::front_ + angle) % 4];
	sr.r = ascan[(Map::right_ + angle) % 4];
	sr.b = ascan[(Map::back_ + angle) % 4];
	sr.l = ascan[(Map::left_ + angle) % 4];
	sr.angle = angle;
	sr.enemy = aenemy[(Map::front_ + angle) % 4];
}

void simMovePos(int n, Pos & p, int angle)
{
	switch (angle)
	{
	case Map::front_: p.y += n; break;
	case Map::right_: p.x += n; break;
	case Map::back_: p.y -= n; break;
	case Map::left_: p.x -= n; break;
	}
}

int main()
{
	/*
	Map to simpulate
	.	0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17	18	19	20	21
	0	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#
	1	#	E	#	.	#	E	.	.	E	.	.	.	.	.	.	#	.	.	.	.	.	#
	2	#	.	.	.	#	#	.	.	.	.	.	.	.	.	.	.	.	.	.	.	.	#
	3	#	.	#	.	.	.	.	E	#	.	#	#	#	#	.	#	#	.	#	.	E	#
	4	#	.	#	.	#	#	#	#	#	.	.	.	.	#	.	E	.	.	#	#	#	#
	5	#	.	#	.	.	.	.	.	#	.	.	.	.	#	#	#	#	E	.	.	.	#
	6	#	.	#	.	.	#	.	.	.	.	.	.	.	.	#	.	.	.	.	E	.	#
	7	#	.	#	#	#	#	.	.	.	#	#	#	E	.	#	.	.	#	#	#	.	#
	8	#	.	.	.	.	.	.	.	#	.	.	#	#	.	#	.	.	.	#	.	.	#
	9	#	.	.	.	#	.	.	.	#	E	.	.	#	.	#	.	.	.	.	.	.	#
	10	#	.	.	E	#	.	.	.	#	.	S	.	#	.	#	.	.	.	#	.	.	#
	11	#	.	.	#	#	.	.	#	#	.	.	.	.	.	.	.	E	.	#	.	.	#
	12	#	.	.	#	.	.	.	E	#	#	#	#	#	#	#	.	.	.	#	.	.	#
	13	#	#	#	#	.	.	.	.	#	.	.	.	.	.	.	.	.	.	#	.	.	#
	14	#	E	#	.	.	.	.	.	#	.	.	.	.	.	.	.	.	.	#	.	.	#
	15	#	.	#	.	.	.	.	.	E	.	.	.	.	.	.	.	.	.	#	E	.	#
	16	#	.	#	.	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	.	#
	17	#	.	#	.	.	.	.	E	#	E	.	#	.	.	.	.	.	#	E	.	.	#
	18	#	.	.	.	.	.	.	.	E	.	.	.	.	.	.	#	.	#	.	.	.	#
	19	#	.	.	.	.	.	.	E	#	E	.	.	.	.	.	#	.	.	.	.	.	#
	20	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#	#

	*/
	// NOTE! this map is loaded in on it's side. THIS DIRECTION IS UP -->
	LoadMapData("######################");
	LoadMapData("#E#.#E..E......#.....#");
	LoadMapData("#...##...............#");
	LoadMapData("#.#....E#.####.##.#.E#");
	LoadMapData("#.#.#####....#.E..####");
	LoadMapData("#.#.....#....####E...#");
	LoadMapData("#.#..#........#....E.#");
	LoadMapData("#.####..####E.#..###.#");
	LoadMapData("#.......#..##.#...#..#");
	LoadMapData("#...#...#E..#.#......#");
	LoadMapData("#..E#...#.S.#.#...#..#");
	LoadMapData("#..##..##.......E.#..#");
	LoadMapData("#..#...E#######...#..#");
	LoadMapData("####....#.........#..#");
	LoadMapData("#E#.....#.........#..#");
	LoadMapData("#.#.....E.........#E.#");
	LoadMapData("#.#.################.#");
	LoadMapData("#.#....E#E.#.....#E..#");
	LoadMapData("#.......E......#.#...#");
	LoadMapData("#......E#E.....#.....#");
	LoadMapData("######################");


	Pos scanPos(10, 10);
	int scanAngle = Map::front_;
	bool bDiscoveryDone = false;
	Map map;
	while (!bDiscoveryDone)
	{
		simScanResults sr;
		simGetScan(scanPos, sr, scanAngle);
		map.updateMap(sr.f, sr.r, sr.b, sr.l, sr.enemy);
		//map.info();
		switch (map.getNextStep())
		{
		case Map::moveForward: { map.move(1); simMovePos(1, scanPos, scanAngle); break; }
		case Map::moveBackward: { map.move(-1); simMovePos(-1, scanPos, scanAngle); break;}
		case Map::turnRight: { map.setangle(1);  scanAngle++; scanAngle %= 4; break; }
		case Map::turnLeft: { map.setangle(-1); scanAngle--; if (scanAngle < 0) scanAngle += 4; scanAngle %= 4; break; }
		}
		bDiscoveryDone = map.DiscoveryDone();
	}
	map.info();
	cout << "Map Discovery - Done!!!" << endl;

	//map.updateMap(1, 3, 1, 2, true);
	//map.info();

	/*
	2,3,1,2
	***********************************
	angle:0
	pos:2,1
	***********************************
	printing map
	000000
	003000
	120221
	001000
	nWidth:6, nHeight:4
	*/

	/* Map resize test*/
	//map.move(1);
	//map.setangle(1);
	//map.move(1);
	//map.setangle(-1);
	//map.resize(3, 3, 3, 3);
	//map.resize(2, 2, 1, 1);
	//map.resize(2, 2, 2, 1);
	//map.resize(2, 2, 2, 2);
	//map.resize(4, 4, 4, 4);


	/* Map navigation test */
	// counter clock turn
	/*map.setangle(1);
	map.move(1);
	map.info();
	map.setangle(-1);
	map.move(1);
	map.info();
	map.setangle(-1);
	map.move(1);
	map.move(1);
	map.info();
	map.setangle(-1);
	map.move(1);
	map.move(1);
	map.info();
	map.setangle(-1);
	map.move(1);
	map.move(1);
	map.info();
	map.setangle(-1);
	map.move(1);
	map.setangle(-1);
	map.move(1);
	map.info();*/

	return 0;
}
