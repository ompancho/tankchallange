#ifndef Solution_h
#define Solution_h
#include "API.h"

class API;

class Solution {
 public:
  Solution() 
  {
    nFuel_prev = 0;
    nFuel = 0;
    nTrack = -1;
    vAction.resize(6);
    
    //Level 1 - 842
    vAction[0].push_back(turnLeft);
    for(int i = 0; i < 9; i++)
      vAction[0].push_back(moveForward);
    vAction[0].push_back(turnRight);
    for(int i = 0; i < 4; i++)
    vAction[0].push_back(moveForward);
    vAction[0].push_back(turnRight);
    for(int i = 0; i < 4; i++)
      vAction[0].push_back(wait);
    vAction[0].push_back(turnLeft);
    for(int i = 0; i < 5; i++)
      vAction[0].push_back(moveForward);
    vAction[0].push_back(turnRight);
  
  
    // Level 2 - 1309
    vAction[1].push_back(turnRight);
    vAction[1].push_back(moveForward);
    vAction[1].push_back(turnRight);
    vAction[1].push_back(wait);
    vAction[1].push_back(turnLeft);
    vAction[1].push_back(turnLeft);
    for(int i = 0; i < 9; i++)
      vAction[1].push_back(wait);
    for(int i = 0; i < 7; i++)
      vAction[1].push_back(moveForward);
    vAction[1].push_back(turnRight);
    for(int i = 0; i < 3; i++)
      vAction[1].push_back(moveForward);
    vAction[1].push_back(turnRight);
    for(int i = 0; i < 2; i++)
      vAction[1].push_back(moveForward);
    vAction[1].push_back(turnRight);
    for(int i = 0; i < 3; i++)
      vAction[1].push_back(wait);
    vAction[1].push_back(turnRight);
    for(int i = 0; i < 2; i++)
      vAction[1].push_back(moveForward);
    vAction[1].push_back(turnLeft);
    for(int i = 0; i < 9; i++)
      vAction[1].push_back(moveForward);
    vAction[1].push_back(turnLeft);
    for(int i = 0; i < 2; i++)
      vAction[1].push_back(moveForward);
    vAction[1].push_back(turnLeft);
    vAction[1].push_back(moveForward);
    vAction[1].push_back(turnRight);
    
    //level 3 - 1875
    vAction[2].push_back(moveForward);
    vAction[2].push_back(turnLeft);
    for(int i = 0; i < 4; i++)
      vAction[2].push_back(moveForward);
    vAction[2].push_back(turnRight);
    for(int i = 0; i < 4; i++)
      vAction[2].push_back(wait);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(wait);
    vAction[2].push_back(wait);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(moveForward);
    
    vAction[2].push_back(turnRight);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(turnRight);
    vAction[2].push_back(moveBackward);
    //for(int i = 0; i < 4; i++)
    //  vAction[2].push_back(wait);

    for(int i = 0; i < 24; i++)
      vAction[2].push_back(fireCannon);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(turnLeft);
    //vAction[2].push_back(wait);
    for(int i = 0; i < 6; i++)
      vAction[2].push_back(moveForward);
    vAction[2].push_back(wait);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(turnRight);
    vAction[2].push_back(wait);
    vAction[2].push_back(turnLeft);
    vAction[2].push_back(turnLeft);
    for(int i = 0; i < 3; i++)
      vAction[2].push_back(moveForward);
    vAction[2].push_back(turnLeft);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(turnRight);
    vAction[2].push_back(wait);
    vAction[2].push_back(turnLeft);
    vAction[2].push_back(moveForward);
    vAction[2].push_back(turnRight);
    vAction[2].push_back(wait);
    vAction[2].push_back(turnLeft);
    
    // level 4 - 
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 6; i++)//delay
     vAction[3].push_back(wait);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnRight);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
     vAction[3].push_back(moveForward);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    
    //for(int i = 0; i < 4; i++)//delay
    vAction[3].push_back(turnLeft);
    
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    //corner
    //delay
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
    
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 3; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    //corner
    for(int i = 0; i < 3; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
    for(int i = 0; i < 5; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);

     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);

     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnLeft);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    //corner

    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 4; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);

    vAction[3].push_back(turnRight);
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    for(int i = 0; i < 4; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnLeft);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 2; i++)
     vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnRight);





     /*
    vAction[3].push_back(turnRight);
    vAction[3].push_back(wait);
    vAction[3].push_back(turnRight);
    for(int i = 0; i < 4; i++)
      vAction[3].push_back(moveBackward);*/
/*
    for(int i = 0; i < 10; i++)
    {
      vAction[3].push_back(turnRight);
      vAction[3].push_back(wait);
      vAction[3].push_back(wait);
      vAction[3].push_back(turnRight);
      vAction[3].push_back(turnRight);
      vAction[3].push_back(wait);
      vAction[3].push_back(wait);
      vAction[3].push_back(turnRight);
    
      vAction[3].push_back(moveForward);
    }
    vAction[3].push_back(turnRight);
    vAction[3].push_back(turnRight);
    vAction[3].push_back(moveForward);
    vAction[3].push_back(turnRight);

    for(int i = 0; i < 10; i++)
    {
      vAction[3].push_back(turnRight);
      vAction[3].push_back(wait);
      vAction[3].push_back(wait);
      vAction[3].push_back(turnRight);
      vAction[3].push_back(wait);
      vAction[3].push_back(turnRight);
      vAction[3].push_back(wait);
      vAction[3].push_back(wait);
      vAction[3].push_back(turnRight);
      vAction[3].push_back(wait);

      vAction[3].push_back(moveForward);
    }*/
    
    
    // level 5 - 2949
    vAction[4].push_back(turnRight);
    vAction[4].push_back(moveForward);
    vAction[4].push_back(turnRight);
    for(int i = 0; i < 3; i++)
      vAction[4].push_back(moveForward);
    for(int i = 0; i < 6; i++)
      vAction[4].push_back(wait);
    vAction[4].push_back(fireCannon);
    vAction[4].push_back(fireCannon);
    vAction[4].push_back(wait);
    vAction[4].push_back(wait);
    for(int i = 0; i < 22; i++)
      vAction[4].push_back(fireCannon);
    vAction[4].push_back(moveBackward);
      vAction[4].push_back(fireCannon);
    vAction[4].push_back(turnRight);
    vAction[4].push_back(moveBackward);
      vAction[4].push_back(fireCannon);
    vAction[4].push_back(moveBackward);
      vAction[4].push_back(fireCannon);
    vAction[4].push_back(moveBackward);
      vAction[4].push_back(fireCannon);
      vAction[4].push_back(fireCannon);
    for(int i = 0; i < 12; i++)
      vAction[4].push_back(fireCannon);
    vAction[4].push_back(turnRight);
    vAction[4].push_back(fireCannon);
    vAction[4].push_back(turnLeft);
    for(int i = 0; i < 10; i++)
      vAction[4].push_back(fireCannon);
    vAction[4].push_back(turnRight);
    vAction[4].push_back(fireCannon);
    vAction[4].push_back(turnLeft);
    /*for(int i = 0; i < 70; i++)
      vAction[4].push_back(fireCannon);*/  

    //7406 (1,2,3,5)
    vAction[5].push_back(turnRight);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    for(int i = 0; i < 120; i++)
      vAction[5].push_back(wait);
    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    for(int i = 0; i < 4; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(wait);
    vAction[5].push_back(wait);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    for(int i = 0; i < 12; i++)
      vAction[5].push_back(wait);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(turnRight);
    for(int i = 0; i < 12; i++)
      vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(moveBackward);
    vAction[5].push_back(moveBackward);
    vAction[5].push_back(turnRight);
    for(int i = 0; i < 4; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(wait);
    vAction[5].push_back(wait);
    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);
    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);
    
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);

    for(int i = 0; i < 11; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(wait);
    vAction[5].push_back(wait);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    for(int i = 0; i < 12; i++)
      vAction[5].push_back(wait);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(turnRight);
    for(int i = 0; i < 15; i++)
      vAction[5].push_back(wait);
    for(int i = 0; i < 4; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);
    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(wait);
    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);

    vAction[5].push_back(turnLeft);
    vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);

    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnRight);
    vAction[5].push_back(wait);
    vAction[5].push_back(wait);
  
    
    /*
    vAction[5].push_back(turnLeft);
    vAction[5].push_back(wait);
    vAction[5].push_back(turnLeft);     
    vAction[5].push_back(turnLeft); 
    vAction[5].push_back(wait);
    for(int i = 0; i < 3; i++)
      vAction[5].push_back(moveForward);
    vAction[5].push_back(turnLeft);  
      */
    nStep = 0;
  }
  
  enum 
  {
    wait=0, currentFuel,moveForward,moveBackward,turnLeft,turnRight,identifyTarget,fireCannon,lidarFront,lidarBack,lidarLeft,lidarRight
  };
  

  int nStep;
  vector<vector<int>> vAction;
  int nTrack;
  int nFuel_prev;
  int nFuel;

  /**
   * Executes a single step of the tank's programming. The tank can only move,
   * turn, or fire its cannon once per turn. Between each update, the tank's
   * engine remains running and consumes 1 fuel. This function will be called
   * repeatedly until there are no more targets left on the grid, or the tank
   * runs out of fuel.
   */
  void update() 
  {
    int f = API::lidarFront();
    int b = API::lidarBack();
    int l = API::lidarLeft();
    int r = API::lidarRight();
    bool bTarget = API::identifyTarget();

    //track selection
    if(nTrack == -1)
    {
      cout << f << "," << b << "," << l << "," << r << endl;
      
      if((f==5) && (b==1) && (l==10) && (r==11))
      {
        nTrack = 0;
        cout << nStep << " - Track set to: " << nTrack << endl;
      }
      else if((f==10) && (b==11) && (l==3) && (r==2))
      {
        nTrack = 1;
        cout << nStep << " - Track set to: " << nTrack << endl;
      }
      else if((f==2) && (b==1) && (l==2) && (r==3))
      {
        nTrack = 2;
        cout << nStep << " - Track set to: " << nTrack << endl;
      }
      else if((f==5) && (b==4) && (l==2) && (r==1))
      {
        nTrack = 3;
        cout << nStep << " - Track set to: " << nTrack << endl;
      }
      else if((f==2) && (b==3) && (l==6) && (r==5))
      {
        nTrack = 4;
        cout << nStep << " - Track set to: " << nTrack << endl;
      }
      else if((f==3) && (b==1) && (l==1) && (r==2))
      {
        nTrack = 5;
        cout << nStep << " - Track set to: " << nTrack << endl;
      }

      
      else
       cout << nStep << " - Track not found!!!" << endl;
    }
    //else
    //  cout << "nTrack: " << nTrack << endl;
    
    if(nTrack == 3 && bTarget)
    {
      cout << nStep << " - Fire" << endl;
      API::fireCannon();
    }
    else
    if(vAction[nTrack].size() && nTrack >= 0)
    {
      int nAction = vAction[nTrack][0];
      vAction[nTrack].erase(vAction[nTrack].begin());
      cout << nStep << " - Action: " << nAction << endl;
      switch(nAction)
      {
        case moveForward: API::moveForward(); break;
        case moveBackward: API::moveBackward(); break;
        case turnLeft: API::turnLeft(); break;
        case turnRight: API::turnRight(); break;
        case fireCannon: API::fireCannon(); break;
        case wait: break;
      }
    }
    
    if(bTarget)
    {
      cout << nStep << " - Fire" << endl;
      API::fireCannon();
    }
      //case currentFuel: API::currentFuel(); break;
      //case identifyTarget: API::identifyTarget(); break;
      //case lidarFront: API::lidarFront(); break;
      //case lidarBack: API::lidarBack(); break;
      //case lidarLeft: API::lidarLeft(); break;
      //case lidarRight: API::lidarRight(); break;
    nFuel_prev = nFuel;
    nFuel = API::currentFuel();
    cout << nStep << " - Fuel: " << nFuel << ( (nFuel_prev - nFuel > 50) ? " *** HIT ***" : "" ) << endl;
  
    nStep++;  
  }
};

#endif
