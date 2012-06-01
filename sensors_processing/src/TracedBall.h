#ifndef TRACED_BALL
#define TRACED_BALL

#include <string>
#include <math.h>
#include <stdio.h>

using namespace std;

class TracedBall {

private:
	float x_; //	wspolrzedna x w odom
	float y_; //	wspolrzedne y w odom
	int hits_; //	liczba pojawien pilki
	bool selected_;	//	czy do tej pilki jedzie robot

public:

	TracedBall(float x, float y);
	bool isTheSameBall(TracedBall & ball);
	void hitsUp();
	void hitsDown();
	int getHits();

	float getX();
	void setX(float x);
	float getY();
	void setY(float y);
	void updateXY(float x, float y);
	string toString();
	bool isBall();
	bool isMistake();

	bool isSelected();
	void select();
	void unselect();
};

#endif
