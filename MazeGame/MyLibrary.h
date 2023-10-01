class Vector2D {
  public:
    float x;
    float y;
};

class Player {
  public:
    int ItemNum;
    int centerAngleIndex;
    Vector2D pos;
    
    Player(float nx, float ny){
      ItemNum = 0;
      centerAngleIndex = 0;
      pos.x = nx; pos.y = ny;
    }
};

class Item {
  public:
    Vector2D pos;
    bool taken;
};
