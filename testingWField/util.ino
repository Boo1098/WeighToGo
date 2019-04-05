// Gives the angle relative to the horizontal from OSV to target.
float angleTo(float x, float y) {
  float delX = x - locX;
  float delY = y - locY;
  float angle = atan2(delY, delX);
  return angle;
}

// This function computes the distance from the OSV to the coordinate passed in
float distanceTo(float x, float y) {
  float delX = locX - x;
  float delY = locY - y;
  return sqrt(sq(delX) + sq(delY));
}

// Anything that needs to be ran every tick goes here
void updateEverything() {
  while (!updateLocation())
    ;
  while (!((locY < 2 && locY > 0) && (locX < 4 && locX > 0) && (locT > -3.2 && locT < 3.2))) {
    Enes100.println("Bad location");
    updateLocation();
  }
  printStats();
  scale.set_scale(calibration_factor);
}