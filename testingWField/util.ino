// Gives the angle relative to the horizontal from OSV to target.
float angleTo(float x, float y) {
    float delX = x-getX();
    float delY = y-getY();
    float angle= atan2(delY, delX);
    return angle;
}

// This function computes the distance from the OSV to the coordinate passed in
float distanceTo(float x, float y) {
    float delX = getX()-x;
    float delY = getY()-y;
    return sqrt(sq(delX)+sq(delY));
}

// Returns OSV X
float getX(){
  return locX;
}

// Returns OSV Y
float getY(){
  return locY;
}

// Returns OSV Theta
float getTheta(){
  return locT;
}