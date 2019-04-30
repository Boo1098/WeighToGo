// Gives the angle relative to the horizontal from OSV to target.
double angleTo(double x, double y) {
  double delX = x - locX;
  double delY = y - locY;
  double angle = atan2(delY, delX);
  return angle;
}

// This function computes the distance from the OSV to the coordinate passed in
double distanceTo(double x, double y) {
  double delX = locX - x;
  double delY = locY - y;
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

int getColumn(double y) {
  if (y < 0.666) {
    return 1;
  } else if (y < 1.333) {
    return 2;
  } else {
    return 3;
  }
}