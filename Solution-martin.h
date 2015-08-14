#ifndef Solution_h
#define Solution_h
#include "API.h"

class API;

#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>


#define GetValue(v,p)   ((0 <= p.x && p.x < (int)v.size()) && (0 <= p.y && p.y < (int)v[0].size())) ? (v)[p.x][p.y] : wall

using namespace std;

class Pos
{
public:
	Pos() {}
	Pos(int x_, int y_) : x(x_), y(y_), d(0), bHighPriority(false), bLookOnly(false) { }
	int x, y;
	bool bHighPriority;
	double d;
	bool bLookOnly;
};
inline bool operator==(const Pos& p1, const Pos& p2) { return p1.x == p2.x && p1.y == p2.y; }
struct cmpPos
{
	inline bool operator() (const Pos& p1, const Pos& p2)
	{
		return (p1.d < p2.d);
	}
};

struct scanResultsSim { int f; int b; int l; int r; bool enemy; int angle; };
struct scanResults { int d; int v; int a; int direction; Pos p; };
vector<vector<int>> mapDataSim;

struct compareKillCost
{
	inline bool operator() (const pair<int, vector<int>> & a, const pair<int, vector<int>> & b)
	{
		return (a.first < b.first);
	}
};
struct compareWaypoint
{
	inline bool operator() (const pair<int, Pos> & a, const pair<int, Pos> & b)
	{
		return ((a.first * a.second.d * (a.second.bHighPriority ? 1 : 100)) < (b.first * b.second.d) * (b.second.bHighPriority ? 1 : 100));
	}
};


#define FRONT(p) Pos(p.x,p.y+1)
#define BACK(p) Pos(p.x,p.y-1)
#define RIGHT(p) Pos(p.x+1,p.y)
#define LEFT(p) Pos(p.x-1,p.y)
//#define SETVAL(p,v) vpathfinder[p.x][p.y] = v
#define GETDISTANCE(a,b) (int)sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))


//
class Map
{
public:
	Map()
		: angle(0)
		, currentPos(0, 0)
		, bSortWayPoint(true)
		, lidarR(1)
		, lidarL(1)
	{
		//initial size 1x1
		for (int i = 0; i < 1; i++)
		{
			vector<int> v = { 0 };
			mapData.push_back(v);
		}
	}


	enum { unknown = 0, scanned_empty = 1, visited = 2, scanned_unknown = 3, enemy = 4, wall = 5 };
	enum { front_ = 0, right_ = 1, back_ = 2, left_ = 3 };
	enum { moveForward = 1, moveBackward, turnRight, turnLeft, fire };
	enum { cost_idle = 1, cost_turn = 1, cost_move = 1, cost_fire = 5, cost_hit = 50 };
	enum { enemy_inbound = 1, enemy_outgoing, enemy_parallele, enemy_static };


public:

	//updateMap handles :
	//-resizeing of the map based on new data
	//-updating the map data with new objects
	//-detecting enemies
	//-setting waypoints
	//(how waypoints are added)
	//-on resize set the corner of the new map as waypoint, try to add use opposite diagonal corners
	//-if enemy was detected passing paralell to you, set it as waypoint
	//-note! no waypoint will be added outside the "known world", there will be no unreachable waypoints
	//(how waypoints are removed)
	//-if a waypoint is put on a wall then it's removed from the list once it's deteced it's a wall
	//-by reaching the waypoint
	//-finding the shortest path to the waypoint, call function getNextStep() to get the next command

	void updateMap(int lidarFront, int lidarRight, int lidarBack, int lidarLeft, bool target)
	{
		lidarR = lidarRight;
		lidarL = lidarLeft;

		//resize memory behind the map
		resize(lidarFront, lidarRight, lidarBack, lidarLeft);

		Pos & p = currentPos;
		//update map with new lidar data
		vector<int> distance(4);
		vector<int> lidarData(4);

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
		UpdateMapValue(Pos(p.x, p.y + dDistanceFront), lidarData[front_], front_);

		for (int i = 1; i < dDistanceRight; i++)
		{
			mapData[p.x + i][p.y] = scanned_empty;
		}
		UpdateMapValue(Pos(p.x + dDistanceRight, p.y), lidarData[right_], right_);

		for (int i = 1; i < dDistanceBack; i++)
		{
			mapData[p.x][p.y - i] = scanned_empty;
		}
		UpdateMapValue(Pos(p.x, p.y - dDistanceBack), lidarData[back_], back_);

		for (int i = 1; i < dDistanceLeft; i++)
		{
			mapData[p.x - i][p.y] = scanned_empty;
		}
		UpdateMapValue(Pos(p.x - dDistanceLeft, p.y), lidarData[left_], left_);

		//
		mapData[currentPos.x][currentPos.y] = visited;

		//remove waypoint if we have deteced as wall, but only if the distance to the waypoint from current position is 1 step (that way we don't delete it prematurely)
		if (waypoints.size())
		{
			if (GETDISTANCE(waypoints[0].second, currentPos) < 2)
			{
				if (mapData[waypoints[0].second.x][waypoints[0].second.y] == wall)
				{
					waypoints.erase(waypoints.begin());
					bSortWayPoint = true;
				}
			}
		}
		threatAssessment();

		//update the next step to take
		updatePath();
	}

	void UpdateMapValue(Pos p, int v, int angle)
	{
		int & d = mapData[p.x][p.y];

		int nFreeSideCount = 0;
		int vf = GetValue(mapData, FRONT(p));
		int vr = GetValue(mapData, RIGHT(p));
		int vb = GetValue(mapData, BACK(p));
		int vl = GetValue(mapData, LEFT(p));
		if (vf == scanned_empty) nFreeSideCount++;
		if (vr == scanned_empty) nFreeSideCount++;
		if (vb == scanned_empty) nFreeSideCount++;
		if (vl == scanned_empty) nFreeSideCount++;

		// look at some corners
		if (d == unknown && v == scanned_unknown && nFreeSideCount >= 2)
		{
			//dont add outer edges
			if( ( 0 < p.x && p.x < (int)mapData.size() - 1) && (0 < p.y && p.y < (int)mapData[0].size() - 1) )
			{
				addWayPoint(p, true, true);
			}
		}
		//moving enemy on the side
		if ((d == scanned_empty || v == visited) && (v == scanned_unknown) )
		{
		//	if(GETDISTANCE(p, currentPos) <= 3)
				d = enemy;
		//	else
				//TODO - Impruve this!!!
		//		addWayPoint(p, true, true); //add a way point,
		}
		//moving enemy on the front
		/*if ((d == scanned_empty || v == visited) && (v == enemy))
		{
			if(GETDISTANCE(p, currentPos) <= 2)
				d = enemy;
			else
				//TODO - Impruve this!!!
				addWayPoint(p, true, true); //add a way point,
		}*/
		//something moved, enemy - add waypoint
		if ((d == scanned_unknown) && (v == scanned_empty))
		{
			d = v;
			//TODO - Impruve this!!!
			addWayPoint(p, true, true); //add a way point,
		}
		if (d != wall && d != enemy)
		{
			d = v;
		}	/**/
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
			if (currentPos == waypoints[0].second)
			{
				cout << "Remove WayPoint" << endl;
				waypoints.erase(waypoints.begin());
				bSortWayPoint = true;
			}
			if (!waypoints.size())
			{
				AddAllUnknownsAsWaypoints();
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
		//do we have any enemies in sight, deal with them, 
		if (vNextEnemy.size())
		{
			//we have two options here, fire or turn
			nextPos = vNextEnemy[0];

			//decide what command to exec based on currentPos, nextPos and angle
			//TODO - optimize witch way to turn if the enemy is on your back
			vector<int> vmove(4);
			vmove[(front_ + angle) % 4] = fire;
			vmove[(right_ + angle) % 4] = turnRight;
			vmove[(back_ + angle) % 4] = lidarR < lidarL ? turnRight : turnLeft; //moveBackward
			vmove[(left_ + angle) % 4] = turnLeft;

			int nNextStep = turnLeft;
			if ((currentPos.x - nextPos.x) < 0) nNextStep = vmove[right_];
			if ((currentPos.x - nextPos.x) > 0) nNextStep = vmove[left_];
			if ((currentPos.y - nextPos.y) < 0) nNextStep = vmove[front_];
			if ((currentPos.y - nextPos.y) > 0) nNextStep = vmove[back_];

			//sim
			if (nNextStep == fire)
			{
				mapData[nextPos.x][nextPos.y] = scanned_empty;
			}

			return nNextStep;
		}
		else
		{
			//decide what command to exec based on currentPos, nextPos and angle
			vector<int> vmove(4);
			vmove[(front_ + angle) % 4] = moveForward;
			vmove[(right_ + angle) % 4] = turnRight;
			vmove[(back_ + angle) % 4] = lidarR < lidarL ? turnRight : turnLeft; //moveBackward
			vmove[(left_ + angle) % 4] = turnLeft;

			int nNextStep = turnLeft;
			if ((currentPos.x - nextPos.x) < 0) nNextStep = vmove[right_];
			if ((currentPos.x - nextPos.x) > 0) nNextStep = vmove[left_];
			if ((currentPos.y - nextPos.y) < 0) nNextStep = vmove[front_];
			if ((currentPos.y - nextPos.y) > 0) nNextStep = vmove[back_];

			//check that we don't reverse into anything unknown
			int v = mapData[nextPos.x][nextPos.y];
			if (nNextStep == moveForward && (v == wall))
			{
				nNextStep = turnLeft;
			}
			if (nNextStep == moveBackward && (v == scanned_unknown || v == unknown || v == enemy))
			{
				nNextStep = turnLeft;
			}
			return nNextStep;
		}
	}

	void info()
	{
		cout << "*******************************************************>" << endl;
		cout << "angle:" << angle << endl;
		cout << "currentPos:" << currentPos.x << "," << currentPos.y << endl;
		cout << "   nextPos:" << nextPos.x << "," << nextPos.y << endl;

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
		//cout << "-------------------------------------------------------" << endl;
		//for (y = mapData[0].size(); y > 0; y--)
		//{
		//for (x = 0; x < (int)mapData.size(); x++)
		//{
		//int v = mapData[x][y - 1];
		//int v = mapData[x][y - 1];
		//}
		//}
		//cout << "-------------------------------------------------------";
		cout << "nWidth:" << mapData.size() << ", nHeight:" << mapData[0].size() << endl;

	}

	//bool DiscoveryDone()
	//{
	//	if(!waypoints.size()) 
	//		AddAllUnknownsAsWaypoints();
	//	return waypoints.size() ? false : true;
	//}

private:
	void resize(int lidarFront, int lidarRight, int lidarBack, int lidarLeft)
	{
		nWidth = mapData.size();
		nHeight = mapData[0].size();

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
				waypoints[i].second.y += dDistanceBack;
		}
		if (dDistanceLeft > 0)
		{
			currentPos.x += dDistanceLeft;
			for (int i = 0; i < (int)waypoints.size(); i++)
				waypoints[i].second.x += dDistanceLeft;
		}

		//add new corners as waypoints
		nWidth = mapData.size();
		nHeight = mapData[0].size();

		if (dDistanceFront > 0)
		{
			Pos p(1, nHeight - 2);
			//if (mapData[p.x][p.y] == unknown)
				addWayPoint(p, false, false);
		}
		if (dDistanceBack > 0)
		{
			Pos p(nWidth - 2, nHeight - 2);
			//if (mapData[p.x][p.y] == unknown)
				addWayPoint(p, false, false);
		}
		if (dDistanceLeft > 0)
		{
			Pos p(1, 1);
			//if (mapData[p.x][p.y] == unknown)
				addWayPoint(p, false, false);
		}
		if (dDistanceRight > 0)
		{
			Pos p(nWidth - 2, 1);
			//if (mapData[p.x][p.y] == unknown)
				addWayPoint(p, false, false);
		}

		// TODO - analyze new lidar data for enemy movment, do cost analyses,  set waypoints

	}


	//scan the area for enemies
	//calculate cost to kill all enemies
	//sort by lowest cost first, use lowest cost operations

	void threatAssessment()
	{
		vector<scanResults> vsr(4);
		ScanMap(currentPos, vsr, angle);
		//debug
		//
		//vsr[0].v = wall;
		//vsr[1].v = enemy;
		//vsr[2].v = enemy;
		//vsr[3].v = enemy;

		//remove non enemies
		for (int i = 0; i < (int)vsr.size(); i++)
		{
			if (vsr[i].v != enemy)
			{
				vsr.erase(vsr.begin() + i);
				i--;
			}
		}

		//build a list of possible combinations
		int nCount = vsr.size();
		vector<vector<int>> orderToKill;
		for (int i = 0; i < nCount; i++)
		{
			if (nCount == 1)
			{
				orderToKill.push_back({ i });
			}
			else
			{
				for (int j = 0; j < nCount; j++)
				{
					if (nCount == 2)
					{
						if (i != j)
							orderToKill.push_back({ i, j });
					}
					else
					{
						for (int k = 0; k < nCount; k++)
						{
							if (nCount == 3)
							{
								if (i != j && i != k && j != k)
									orderToKill.push_back({ i, j, k });
							}
							else
							{
								for (int n = 0; n < nCount; n++)
								{
									if (i != j && i != k && i != n && j != k && j != n && k != n)
									{
										orderToKill.push_back({ i,j,k,n });
									}
								}
							}
						}
					}
				}
			}
		}

		//debug
		//orderToKill.clear();
		//orderToKill.push_back({ 0,1,2,3 });
		//orderToKill.push_back({ 0,3,2,1 });
		//orderToKill.push_back({ 3,2,1,0 });
		//orderToKill.push_back({ 2,0,3,1 });

		vector < pair< int, vector<int> > > vCostToKill;
		for (int i = 0; i < (int)orderToKill.size(); i++)
		{
			vector<int> & vkill = orderToKill[i];
			//
			int nStep = 0;
			int nAngle = angle;
			int nCostAll = 0;
			int nCost = 0;
			const int nStepToFire = 1;

			for (int j = 0; j < (int)vkill.size(); j++)
			{
				nCost = 0;
				int n = vkill[j];
				scanResults & sr = vsr[n];

				//NOTE! every turn is counted, even if you turn to objects that are are not enemy

				//TODO - impruve it with enemy classification, inbound, outbound, static...
				bool bShoot = false;
				if (sr.v == enemy) //&& sr.direction == enemy_inbound
				{
					bShoot = true;
				}
				// calculate cost in turn(sortest) and fire
				int nStepToGetInLine = min(abs((nAngle - sr.a) % 4), abs((nAngle - sr.a + 4) % 4));
				nCost += cost_turn * nStepToGetInLine;
				if (bShoot)
					nCost += cost_fire;

				//Fire and turns has to be counted before calculating damage
				nStep += nStepToGetInLine;
				if (bShoot)
					nStep += nStepToFire;

				// calculate damage
				if (bShoot)
					if ((nStep - sr.d) > 0)
						nCost += (nStep - sr.d) * 50;
				nAngle = sr.a;

				nCostAll += nCost;
			}
			vCostToKill.push_back(std::make_pair(nCostAll, vkill));
		}
		sort(vCostToKill.begin(), vCostToKill.end(), compareKillCost());

		//assign the enemy to be your next target
		vNextEnemy.clear();
		for (int i = 0; i < (int)vCostToKill.size(); i++)
		{
			for (int j = 0; j < (int)vCostToKill[i].second.size(); j++)
			{
				vNextEnemy.push_back(vsr[vCostToKill[i].second[j]].p);
			}
		}
	}

	int ScanMapDirection(Pos p, int angle, scanResults & sr)
	{
		int nStep = 0;
		sr.a = angle;
		sr.v = Map::unknown;
		while (sr.v != Map::wall && sr.v != Map::enemy && sr.v != Map::scanned_unknown)
		{
			nStep++;
			switch (angle)
			{
			case front_: sr.v = mapData[p.x][p.y + nStep]; sr.p = Pos(p.x, p.y + nStep); break;
			case right_: sr.v = mapData[p.x + nStep][p.y]; sr.p = Pos(p.x + nStep, p.y); break;
			case back_: sr.v = mapData[p.x][p.y - nStep]; sr.p = Pos(p.x, p.y - nStep); break;
			case left_: sr.v = mapData[p.x - nStep][p.y]; sr.p = Pos(p.x - nStep, p.y); break;
			}
		}
		sr.d = nStep;
		return nStep;
	}

	void ScanMap(Pos currentPos, vector<scanResults> & sr_, int angle)
	{
		vector<scanResults> vsr(4);
		ScanMapDirection(currentPos, front_, vsr[0]);
		ScanMapDirection(currentPos, right_, vsr[1]);
		ScanMapDirection(currentPos, back_, vsr[2]);
		ScanMapDirection(currentPos, left_, vsr[3]);
		sr_[0] = vsr[(front_ + angle) % 4];
		sr_[1] = vsr[(right_ + angle) % 4];
		sr_[2] = vsr[(back_ + angle) % 4];
		sr_[3] = vsr[(left_ + angle) % 4];
	}


	void AddAllUnknownsAsWaypoints()
	{
		vector<int> type = { unknown, scanned_unknown };
		for (size_t t = 0; t < 2; t++)
		{
			for (int x = 1; x < (int)mapData.size() - 1; x++)
			{
				for (int y = 1; y < (int)mapData[0].size() - 1; y++)
				{
					int v = mapData[x][y];
					if (v == type[t])
					{
						Pos p(x, y);
						addWayPoint(p, false, false);
					}
				}
				cout << endl;
			}
			if (waypoints.size())
				break;
		}
	}

	void updatePath()
	{
		bool bDeadEnd = false;
		bool bWalkOnScanedUnknown = false;
		bool bWalkOnVisited = false;
		int nDeadEndCount = 0;
		do
		{
			//we have an issue where waypoints are not been able to reached, we gone try walking on scanned _unkowns
			//bWalkOnScanedUnknown = false;
			bWalkOnVisited = false;
			if (bDeadEnd)
			{
				bWalkOnVisited = true;
			}
			bWalkOnScanedUnknown = true;
			bDeadEnd = false;
			if (!waypoints.size()) return;

			nWidth = mapData.size();
			nHeight = mapData[0].size();

			vpathfinder.clear();
			vpathfinder = mapData;

			if (nDeadEndCount < (int)waypoints.size())
			{
				// sort the waypoints get unexplored and closest one
				if (bSortWayPoint)
				{
					updateWayPointsWeight();
					sort(waypoints.begin(), waypoints.end() /*- nDeadEndCount*/, compareWaypoint());
					bSortWayPoint = false;
				}
			}
			else
			{
				nDeadEndCount = 0;
				bWalkOnScanedUnknown = false;
				bDeadEnd = false;
				waypoints.clear();
				//add all known unknowns as waypoints, all except on the fringes
				AddAllUnknownsAsWaypoints();
			}

			Pos start = waypoints[0].second;
			Pos end = currentPos;
			while (start == end)
			{
				waypoints.erase(waypoints.begin());
				bSortWayPoint = true;
				start = waypoints[0].second;
				if (!waypoints.size()) continue;
			}
			bool bFound = false;

			//if the point is close by try to probe unknown objects
			//if (GETDISTANCE(start, end) < 4)
			//	bWalkOnScanedUnknown = true;

			vector<Pos> vtemp;
			vpathfinder[start.x][start.y] = -1;
			Pos cur = start;
			while (!bFound)
			{
				int n = vpathfinder[cur.x][cur.y];
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

				if (CanWalkON(p0, bWalkOnScanedUnknown, bWalkOnVisited)) { vtemp.push_back(p0); vpathfinder[p0.x][p0.y] = n - 1; }
				if (CanWalkON(p1, bWalkOnScanedUnknown, bWalkOnVisited)) { vtemp.push_back(p1); vpathfinder[p1.x][p1.y] = n - 1; }
				if (CanWalkON(p2, bWalkOnScanedUnknown, bWalkOnVisited)) { vtemp.push_back(p2); vpathfinder[p2.x][p2.y] = n - 1; }
				if (CanWalkON(p3, bWalkOnScanedUnknown, bWalkOnVisited)) { vtemp.push_back(p3); vpathfinder[p3.x][p3.y] = n - 1; }

				if (vtemp.size())
				{
					cur = vtemp[0];
					vtemp.erase(vtemp.begin());
				}
				else
				{
					//cout << "Dead end, can't get to it!!!" << endl;
					info_pathfinder();
					bFound = true;
					bDeadEnd = true;
				}
			}

			//debug
			//info_pathfinder();

			//
			vector<Pos> vpath;
			vpath.push_back(end);
			vtemp.clear();
			cur = end;
			bFound = false;
			while (!bFound && !bDeadEnd)
			{
				int n = vpathfinder[cur.x][cur.y];
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

				int p0_ = vpathfinder[p0.x][p0.y]; if (p0_ == n + 1) { n = p0_; vpath.push_back(p0); }
				int p1_ = vpathfinder[p1.x][p1.y]; if (p1_ == n + 1) { n = p1_; vpath.push_back(p1); }
				int p2_ = vpathfinder[p2.x][p2.y]; if (p2_ == n + 1) { n = p2_; vpath.push_back(p2); }
				int p3_ = vpathfinder[p3.x][p3.y]; if (p3_ == n + 1) { n = p3_; vpath.push_back(p3); }

				if (vpath.size())
				{
					cur = vpath[vpath.size() - 1];
				}
				else
				{
					//cout << "Error... find path";
					break;
				}
			}
			if (bDeadEnd && bWalkOnScanedUnknown && bWalkOnScanedUnknown)
			{
				//remove only when it has tried walking ob scaned unknownes
				waypoints.erase(waypoints.begin());
				bSortWayPoint = true;
				//addWayPoint(start, false, false);
				nDeadEndCount++;
			}
			else if (!bDeadEnd)
			{
				nextPos = vpath[vpath.size() == 1 ? 0 : 1];
			}
		} while (bDeadEnd);

		//debug
		//for (int i = 0; i < (int)vpath.size(); i++)
		//{
		//cout << "step-" << i << " - " << vpath[i].x << "," << vpath[i].y << endl;
		//}
	}

	bool CanWalkON(Pos &p, bool bWalkOnScanedUnknown, bool bWalkOnVisited)
	{
		int v = GetValue(vpathfinder, p);

		if (p == currentPos) bWalkOnVisited = true;
		if (v == unknown || v == scanned_empty || v == enemy || (bWalkOnScanedUnknown && v == scanned_unknown) || (bWalkOnVisited && v == visited))
		{
			return true;
		}
		return false;
	}


	void addWayPoint(Pos p, bool bHighPriority, bool bLookOnly)
	{
		p.bLookOnly = bLookOnly;
		p.bHighPriority = bHighPriority;

		int weight = CalculateWayPointWeight(p);
		cout << "WayPoint(" << weight << "):" << p.x << "," << p.y << endl;
		/*if (bHighPriority)
		{
			//this prevents overwriting of any other high priority waypoint
			int offset = 0;
			for (offset = 0; offset < (int)waypoints.size(); offset++)
			{
				if (!waypoints[offset].second.bHighPriority) break;
			}
			waypoints.insert(waypoints.begin() + offset, make_pair(weight,p));
		}
		else
		{*/
			waypoints.push_back(make_pair(weight, p));
		//}
		bSortWayPoint = true;
	}

	void updateWayPointsWeight()
	{
		for (size_t i = 0; i < waypoints.size(); i++)
		{
			int weight = CalculateWayPointWeight(waypoints[i].second);
			waypoints[i].first = weight;
			waypoints[i].second.d = GETDISTANCE(waypoints[i].second, currentPos);
		}
	}
	
	//how attractive it is to discover, the lower the value the more ther is to discover
	int CalculateWayPointWeight(Pos & p)
	{
		int n = GetValue(mapData, p);
		int nf = GetValue(mapData, FRONT(p));
		int nb = GetValue(mapData, BACK(p));
		int nl = GetValue(mapData, LEFT(p));
		int nr = GetValue(mapData, RIGHT(p));
		return n + nf + nb + nr + nl;
	}



	void info_pathfinder()
	{
		//debug
		cout << "***********************************" << endl;
		cout << "printing path map" << endl;
		int x = 0;
		int y = 0;
		for (y = vpathfinder[0].size(); y > 0; y--)
		{
			for (x = 0; x < (int)vpathfinder.size(); x++)
			{
				cout << setw(3) << setfill(' ') << vpathfinder[x][y - 1];
			}
			cout << endl;
		}
	}

public:
	int nWidth;
	int nHeight;

	int lidarR, lidarL;

	std::vector<std::vector<int>> mapData;
	int angle;
	Pos currentPos, nextPos, nextEnemy; //x,y
	vector< pair<int,Pos>> waypoints;
	vector<Pos> vNextEnemy;
	bool bSortWayPoint;
	vector<vector<int>> vpathfinder;
};




class Solution {
 public:
  Solution() 
  {
    nStep = 0;
  }

  Map map;

  /**
   * Executes a single step of the tank's programming. The tank can only move,
   * turn, or fire its cannon once per turn. Between each update, the tank's
   * engine remains running and consumes 1 fuel. This function will be called
   * repeatedly until there are no more targets left on the grid, or the tank
   * runs out of fuel.
   */
  int nStep;
  void update() 
  {
    
    int f = API::lidarFront();
    int r = API::lidarRight();
    int b = API::lidarBack();
    int l = API::lidarLeft();
    bool enemy = API::identifyTarget();
    
	/*	cout << f << "," << r << "," << b << "," << l << "," << enemy << endl;

    //if(nStep == 1) API::moveForward();
    if(nStep == 0) API::turnLeft();
    if(nStep == 1) API::moveForward();
    if(nStep == 2) API::turnRight();
    if(nStep == 3) API::moveForward();
    if(nStep == 4) API::turnLeft();*/

		map.updateMap(f, r, b, l, enemy);
		//map.info();
		cout << "Fuel: " << API::currentFuel() << endl;
		cout << "Lidar: " << f << "," << r << "," << b << "," << l << "," << enemy << endl;

		switch (map.getNextStep())
		{
		case Map::moveForward: { map.move(1); API::moveForward(); break; }
		case Map::moveBackward: { map.move(-1); API::moveBackward(); break; }
		case Map::turnRight: { map.setangle(1);  API::turnRight(); break; }
		case Map::turnLeft: { map.setangle(-1); API::turnLeft(); break; }
		case Map::fire: { cout << "Fire" << endl; API::fireCannon(); break; }
		}
		cout << "*******************************************************" << endl;
		nStep++;
  }
};

#endif
