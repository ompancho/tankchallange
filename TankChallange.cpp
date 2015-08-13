// TankChallange.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

class Pos
{
public:
	Pos() {}
	Pos(int x_, int y_) : x(x_), y(y_), d(0), bHighPriority(false) { }
	int x, y;
	bool bHighPriority;
	double d;
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

/*
bool pairIntVectorCompare(pair<int, vector<int>> & a, pair<int, vector<int>> & b)
{
	return a.first < b.first;
}*/
struct cmpIntVector
{
	inline bool operator() (const pair<int, vector<int>> & a, const pair<int, vector<int>> & b)
	{
		return a.first < b.first;
	}
};


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
	enum { moveForward = 1, moveBackward, turnRight, turnLeft, fire };
	enum { cost_idle = 1, cost_turn = 1, cost_move = 1, cost_fire = 5, cost_hit = 50 };
	enum { enemy_inbound = 1, enemy_outgoing, enemy_parallele, enemy_static };

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
//TODO - write a more advanced function to handle history of enemy posistions, for threat analysis 
#define UPDATE_MAP_VALUE(d,v) if((d) != wall && (d) != enemy) (d) = (v)

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
		threatAssessment();

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
		//do we have any enemies in sight, deal with them, 
		if(vNextEnemy.size())
		{
			//we have two options here, fire or turn
			nextPos = vNextEnemy[0];

			//decide what command to exec based on currentPos, nextPos and angle
			//TODO - optimize witch way to turn if the enemy is on your back, currently it just turns right
			vector<int> vmove(4);
			vmove[(front_ + angle) % 4] = fire;
			vmove[(right_ + angle) % 4] = turnRight;
			vmove[(back_ + angle) % 4] = turnRight;
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
			vmove[(back_ + angle) % 4] = moveBackward;
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
		if(!waypoints.size()) 
			AddAllUnknownsAsWaypoints();
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
		{
			addWayPoint(Pos(1, nHeight - 1), false, false);
		}
		if (dDistanceBack > 0)
		{
			addWayPoint(Pos(nWidth - 1, nHeight - 1), false, false);
		}
		if (dDistanceRight > 0)
		{
			addWayPoint(Pos(nWidth - 1, 1), false, false);
		}
		if (dDistanceLeft > 0)
		{
			addWayPoint(Pos(1, 1), false, false);
		}

		// TODO - analyze new lidar data for enemy movment, do cost analyses,  set waypoints

	}

	/*
	*	scan the area for enemies
	*	calculate cost to kill all enemies
	*	sort by lowest cost first, use lowest cost operations
	*/
	void threatAssessment()
	{
		vector<scanResults> vsr(4); 
		ScanMap(currentPos, vsr, angle);
		//debug
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
		/*orderToKill.clear();
		orderToKill.push_back({ 0,1,2,3 });
		orderToKill.push_back({ 0,3,2,1 });
		orderToKill.push_back({ 3,2,1,0 });*/
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
			
			//TODO - find out if enemy can both fire and turn at the same time
			//cost_idle = 1, cost_turn = 1, cost_move = 1, cost_fire = 5, cost_hit
			for (int j = 0; j < (int)vkill.size(); j++)
			{
				nCost = 0;
				int n = vkill[j];
				scanResults & sr = vsr[n];
			
				//NOTE! every turn is counted, even if you turn to objects that are are not enemy

				//TODO - impruve it with enemy classification, inbound, outbound, static...
				bool bShoot = false;
				if (sr.v == enemy /*&& sr.direction == enemy_inbound*/)
				{
					bShoot = true;
				}
				// calculate cost in turn(sortest) and fire
				int nStepToGetInLine = min(abs((nAngle - sr.a) % 4), abs((nAngle - sr.a + 4) % 4));
				nCost += cost_turn * nStepToGetInLine;
				if(bShoot)
					nCost += cost_fire; 
					
				//Fire and turns has to be counted before calculating damage
				nStep += nStepToGetInLine;
				if(bShoot)
					nStep += nStepToFire;
					
				// calculate damage
				if(bShoot)
					if ((nStep - sr.d) > 0)
						nCost += (nStep - sr.d) * 50;
				nAngle = sr.a;
				
				nCostAll += nCost;
			}
			vCostToKill.push_back(std::make_pair(nCostAll, vkill));
		}
		sort(vCostToKill.begin(), vCostToKill.end(), pairIntVectorCompare);
	
		//assign the enemy to be your next target
		vNextEnemy.clear();
		for (int i = 0; i < vCostToKill.size(); i++)
		{
			for (int j = 0; j < vCostToKill[i].second.size(); j++)
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
			case front_: sr.v = mapData[p.x][p.y + nStep]; sr.p = Pos(p.x,p.y + nStep); break;
			case right_: sr.v = mapData[p.x + nStep][p.y]; sr.p = Pos(p.x + nStep,p.y); break;
			case back_: sr.v = mapData[p.x][p.y - nStep]; sr.p = Pos(p.x,p.y - nStep); break;
			case left_: sr.v = mapData[p.x - nStep][p.y]; sr.p = Pos(p.x - nStep,p.y); break;
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
		for (int x = 1; x < (int)mapData.size()-1; x++)
		{
			for (int y = 1; y < (int)mapData[0].size()-1; y++)		
			{
				int v = mapData[x][y];
				if (v == scanned_unknown || v == unknown)
				{
					addWayPoint(Pos(x, y), false, false);
				}
			}
			cout << endl;
		}
	}

	void updatePath()
	{
#define FRONT(p) Pos(p.x,p.y+1)
#define RIGHT(p) Pos(p.x,p.y-1)
#define BACK(p) Pos(p.x+1,p.y)
#define LEFT(p) Pos(p.x-1,p.y)
#define GETVAL(p) ((0 <= p.x && p.x < nWidth) && (0 <= p.y && p.y < nHeight)) ?  vpathfinder[p.x][p.y] : wall
#define SETVAL(p,v) vpathfinder[p.x][p.y] = v
#define CAN_WALK_ON(v) v == unknown || v == scanned_empty || v == enemy || (bWalkOnScanedUnknown && v == scanned_unknown)

		bool bDeadEnd = false;
		bool bWalkOnScanedUnknown = false;
		int nDeadEndCount = 0;
		do
		{
			//we have an issue where waypoints are not been able to reached, we gone try walking on scanned _unkowns
			bWalkOnScanedUnknown = false;
			if (bDeadEnd) 
				bWalkOnScanedUnknown = true;
			bDeadEnd = false;
			if (!waypoints.size()) return;

			int nWidth = mapData.size();
			int nHeight = mapData[0].size();

			vector<vector<int>> vpathfinder = mapData;
			
			if (nDeadEndCount < (int)waypoints.size())
			{
				//update dist to current pos
				for (int i = 0; i < (int)waypoints.size(); i++)
				{
					waypoints[i].d = GETDISTANCE(waypoints[i], currentPos);
				}
				// sort the waypoints get closest one
				sort(waypoints.begin(), waypoints.end() - nDeadEndCount, cmpPos());
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

			Pos start = waypoints[0];
			Pos end = currentPos;
			while(start == end)
			{
				waypoints.erase(waypoints.begin());
				start = waypoints[0];
				if(!waypoints.size()) continue;
			}
			bool bFound = false;

			//if the point is close by try to probe unknown objects
			/*if (GETDISTANCE(start, end) < 4)
				bWalkOnScanedUnknown = true;*/

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

				int v0_ = GETVAL(p0);
				int v1_ = GETVAL(p1);
				int v2_ = GETVAL(p2);
				int v3_ = GETVAL(p3);

				if (CAN_WALK_ON(v0_)) { vtemp.push_back(p0); SETVAL(p0, n - 1); }
				if (CAN_WALK_ON(v1_)) { vtemp.push_back(p1); SETVAL(p1, n - 1); }
				if (CAN_WALK_ON(v2_)) { vtemp.push_back(p2); SETVAL(p2, n - 1); }
				if (CAN_WALK_ON(v3_)) { vtemp.push_back(p3); SETVAL(p3, n - 1); }

				if (vtemp.size())
				{
					cur = vtemp[0];
					vtemp.erase(vtemp.begin());
				}
				else
				{
					//info();
					//cout << "Dead end, can't get to it!!!" << endl;
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

					bFound = true;
					bDeadEnd = true;
				}
			}

			//debug
			/*
			cout << "***********************************" << endl;
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
			}
			*/
			//
			vector<Pos> vpath;
			vpath.push_back(end);
			vtemp.clear();
			cur = end;
			bFound = false;
			while (!bFound && !bDeadEnd)
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
				else
				{
					//cout << "Error... find path";
					break;
				}
			}
			if (bDeadEnd && bWalkOnScanedUnknown)
			{ 
				//remove only when it has tried walking ob scaned unknownes
				waypoints.erase(waypoints.begin());
				waypoints.push_back(start);
				nDeadEndCount++;
			}
			else if(!bDeadEnd)
			{
				nextPos = vpath[vpath.size() == 1 ? 0 : 1];
			}
		}
		while(bDeadEnd);

		//debug
		/*for (int i = 0; i < (int)vpath.size(); i++)
		{
		cout << "step-" << i << " - " << vpath[i].x << "," << vpath[i].y << endl;
		}*/
	}

	void addWayPoint(Pos p, bool bHighPriority, bool bAddAfterCurrent)
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
public:
	std::vector<std::vector<int>> mapData;
	int angle;
	Pos currentPos, nextPos, nextEnemy; //x,y
	vector<Pos> waypoints;
	vector<Pos> vNextEnemy;
};


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
	mapDataSim.push_back(v);
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
		case Map::front_: v = mapDataSim[p.x][p.y + nStep]; break;
		case Map::right_: v = mapDataSim[p.x + nStep][p.y]; break;
		case Map::back_: v = mapDataSim[p.x][p.y - nStep]; break;
		case Map::left_: v = mapDataSim[p.x - nStep][p.y]; break;
		}
	}
	bEnemy = (v == Map::enemy);
	return nStep;
}

void simGetScan(Pos currentPos, scanResultsSim & sr, int angle)
{
	bool bEnemyF = false, bEnemyR = false, bEnemyB = false, bEnemyL = false;
	int ascan[] = { 
		simScan(currentPos,Map::front_,bEnemyF), 
		simScan(currentPos,Map::right_,bEnemyR), 
		simScan(currentPos,Map::back_,bEnemyB), 
		simScan(currentPos,Map::left_,bEnemyL) };
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
	/*LoadMapData("######################");
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
	LoadMapData("######################");*/
	LoadMapData("###########");
	LoadMapData("#.E.......#");
	LoadMapData("#.........#");
	LoadMapData("#ES......E#");
	LoadMapData("#.........#");
	LoadMapData("#.........#");
	LoadMapData("#.........#");
	LoadMapData("#.........#");
	LoadMapData("#.E.......#");
	LoadMapData("#.........#");
	LoadMapData("###########");


	Pos scanPos(3, 2);
	int scanAngle = Map::front_;
	bool bDiscoveryDone = false;
	Map map;
	while (!bDiscoveryDone)
	{
		scanResultsSim sr;
		simGetScan(scanPos, sr, scanAngle);
		map.updateMap(sr.f, sr.r, sr.b, sr.l, sr.enemy);
		map.info();
		switch (map.getNextStep())
		{
		case Map::moveForward: { map.move(1); simMovePos(1, scanPos, scanAngle); break; }
		case Map::moveBackward: { map.move(-1); simMovePos(-1, scanPos, scanAngle); break; }
		case Map::turnRight: { map.setangle(1);  scanAngle++; scanAngle %= 4; break; }
		case Map::turnLeft: { map.setangle(-1); scanAngle--; if (scanAngle < 0) scanAngle += 4; scanAngle %= 4; break; }
		//TODO - if you fire, erase enemy, from both mapDataSim and mapData
		case Map::fire: { cout << "Fire" << endl; /*mapDataSim[nextPos.x + (scanPos.x-map.currentPos.x)][nextPos.y] = Map::scanned_empty;*/ break; }
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
