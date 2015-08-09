/*Ed added a writeMap() function that writes map-info to what lidar sees
 * added more printout for dumping map, in map::info()
 * renamed some params for readability, changed enum type objects
*/

// TankChallange.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <iostream>

using namespace std;

struct Pos { int x, y; };

class Map
{
public:
    Map()
        : angle(0)
    {
        towall.resize(4);

        //initial size   
        for (int i = 0; i < 1; i++)
        {
            vector<int> v = { 0 };
            mapData.push_back(v);
        }
        pos.x = 0;
        pos.y = 0;
        //updatePos();
    }

    enum { unknown = 0, scanned_unknown = 1, scanned_empty = 2, enemy = 3, wall = 4 };
    enum { front_ = 0, right_ = 1, back_ = 2, left_ = 3 };

    void resize(int lidarForward, int lidarRight, int lidarBack, int lidarLeft)
    {
        int nWidth = mapData.size();
        int nHeight = mapData[0].size();

        int distanceForward = nHeight - pos.y;
        int distanceRight = nWidth - pos.x;
        int distanceBack = nHeight - distanceForward + 1;
        int distanceLeft = nWidth - distanceRight + 1;

        int distance[] = { distanceForward,distanceRight,distanceBack,distanceLeft };

        //cout << "" << f << "," << r << "," << b << "," << l << endl;
        //cout << "" << f_ << "," << r_ << "," << b_ << "," << l_ << endl;

        int dDistanceForward = lidarForward - distance[(front_ + angle) % 4] + 1;
        int dDistanceRight = lidarRight - distance[(right_ + angle) % 4] + 1;
        int dDistanceBack = lidarBack - distance[(back_ + angle) % 4] + 1;
        int dDistanceLeft = lidarLeft - distance[(left_ + angle) % 4] + 1;

        //If any of the above results produce negative values, that means a alien have moved to that square, or we cant se the full distance
        //(lidar detects nearest object)

        distance[(front_ + angle) % 4] = dDistanceForward;
        distance[(right_ + angle) % 4] = dDistanceRight;
        distance[(back_ + angle) % 4] = dDistanceBack;
        distance[(left_ + angle) % 4] = dDistanceLeft;

        dDistanceForward = distance[front_];
        dDistanceRight = distance[right_];
        dDistanceBack = distance[back_];
        dDistanceLeft = distance[left_];
        cout << "" << dDistanceForward << "," << dDistanceRight << "," << dDistanceBack << "," << dDistanceLeft << endl;

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
        for (int j = 0; j < mapData.size(); j++)
        {
            for (int i = 0; i < dDistanceForward; i++)
            {
                mapData[j].push_back(0);
            }
            for (int i = 0; i < dDistanceBack; i++)
            {
                mapData[j].insert(mapData[j].begin(), 0);
            }
        }

        //update pos
        if (dDistanceBack > 0)
            pos.y += dDistanceBack;
        if (dDistanceLeft > 0)
            pos.x += dDistanceLeft;
    }

    void writeMap(int lidarForward, int lidarRight, int lidarBack, int lidarLeft, bool target)
    {
        int distance[] = { lidarForward,lidarRight,lidarBack,lidarLeft };

        int dDistanceForward = distance[(front_ + angle) % 4];
        int dDistanceRight = distance[(right_ + angle) % 4];
        int dDistanceBack = distance[(back_ + angle) % 4];
        int dDistanceLeft = distance[(left_ + angle) % 4];

        //Clear line of sight. set 'scanned_empty'
        for (int i = 1; i < dDistanceForward; i++)
        {
            mapData[pos.x][pos.y + i] = scanned_empty;
        }
        angle == 0 ?
            (target ? mapData[pos.x][pos.y + dDistanceForward] = enemy :
                mapData[pos.x][pos.y + dDistanceForward] = wall) :
            mapData[pos.x][pos.y + dDistanceForward] = scanned_unknown;
        for (int i = 1; i < dDistanceRight; i++)
        {
            mapData[pos.x + i][pos.y] = scanned_empty;
        }
        angle == 1 ?
            (target ? mapData[pos.x + dDistanceRight][pos.y] = enemy :
                mapData[pos.x + dDistanceRight][pos.y] = wall) :
            mapData[pos.x + dDistanceRight][pos.y] = scanned_unknown;
        for (int i = 1; i < dDistanceBack; i++)
        {
            mapData[pos.x][pos.y - i] = scanned_empty;
        }
        angle == 2 ?
            (target ? mapData[pos.x][pos.y - dDistanceBack] = enemy :
                mapData[pos.x][pos.y - dDistanceBack] = wall) :
            mapData[pos.x][pos.y - dDistanceBack] = scanned_unknown;
        for (int i = 1; i < dDistanceLeft; i++)
        {
            mapData[pos.x - i][pos.y] = scanned_empty;
        }
        angle == 3 ?
            (target ? mapData[pos.x - dDistanceLeft][pos.y] = enemy :
                mapData[pos.x - dDistanceLeft][pos.y] = wall) :
            mapData[pos.x - dDistanceLeft][pos.y] = scanned_unknown;
        cout << "***********************************" << endl;
        cout << "printing map" << endl ;
        int x = 0;
        int y = 0;
        for (y = mapData[0].size(); y > 0; y--)
        {
            for (x = 0; x < mapData.size(); x++)
            {
                cout << mapData[x][y-1];
            }
            cout << endl;
        }

        cout << "nWidth:" << mapData.size() << ", nHeight:" << mapData[0].size() << endl;

        /*cout << "-----------" << endl;
        cout << "  " << towall[front_] << endl;
        cout << towall[left_] << "    " << towall[right_] << endl;
        cout << "  " << towall[back_] << endl;
        cout << "----------" << endl << endl;*/
    }

    std::vector<std::vector<int>> mapData;
    int angle;
    Pos pos; //x,y
    vector<int> towall;
};

struct scanresults { int f; int b; int l; int r; };

int main()
{
    Map map;

    map.resize(2, 3, 1, 2);
    map.writeMap(1, 3, 1, 2, true);
    map.info();



    //map.move(1);
    //map.setangle(1);
    //map.move(1);
    //map.setangle(-1);
    //map.resize(3, 3, 3, 3);
    //map.resize(2, 2, 1, 1);
    //map.resize(2, 2, 2, 1);
    //map.resize(2, 2, 2, 2);
    //map.resize(4, 4, 4, 4);


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
