#include "TracedBall.h"

TracedBall::TracedBall(float x, float y) {
	x_ = x;
	y_ = y;
	hits_ = 0;
	selected_ = false;
}

bool TracedBall::isTheSameBall(TracedBall & ball) {

	float dist = sqrt(pow(ball.getX() - x_, 2) + pow(ball.getY() - y_, 2));
	if (dist < 0.1) {
		return true;
	}
	return false;
}

void TracedBall::hitsUp() {
	hits_ += 2;
	if(hits_ > 3){
		hits_ = 3;
	}
}
void TracedBall::hitsDown() {
	hits_ -= 1;
	if(hits_ < -3){
		hits_ = -3;
	}
}
int TracedBall::getHits() {
	return hits_;
}

float TracedBall::getX() {
	return x_;
}
void TracedBall::setX(float x) {
	x_ = x;
}

float TracedBall::getY() {
	return y_;
}

void TracedBall::setY(float y) {
	y_ = y;
}

void TracedBall::updateXY(float x, float y) {
	x_ = x;
	y_ = y;
}


bool TracedBall::isSelected(){
	return selected_;
}
void TracedBall::select(){
	selected_ = true;
}
void TracedBall::unselect(){
	selected_ = false;
}

std::string TracedBall::toString(){
	std::string retString = "Traced ball - ";

	char* str = new char[30];

	retString.append(" x = ");
	sprintf(str, "%.4g", x_ );
	retString.append(str);

	retString.append(", y = ");
	sprintf(str, "%.4g", y_ );
	retString.append(str);

	retString.append(", hits = ");
	sprintf(str, "%d", hits_ );
	retString.append(str);

	if (selected_ == true) {
		retString.append(", SELECTED ");
	}

	return retString;
}

bool TracedBall::isBall(){
	if(hits_ >=2){
		return true;
	}
	return false;
}
bool TracedBall::isMistake(){
	if(hits_ <= -2){
		return true;
	}
	return false;
}

