#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <glm/glm.hpp>

class Rectangle{
public:
    Rectangle(float _x=0, float _y=0, float _width=0, float _height=0){
	x = _x;
	y = _y;
	width = _width;
	height = _height;
    }

    bool contains(glm::vec2 p) const{
	if(p.x<x)
	    return false;
	if(p.y<y)
	    return false;
	if(p.x>x+width)
	    return false;
	if(p.y>y+height)
	    return false;
	return true;
    }

    float left() const{
	return x+width;
    }

    float bottom() const{
	return y+height;
    }

    Rectangle scaled(float scaleX, float scaleY) const{
	return Rectangle(x*scaleX, y*scaleY, width*scaleX, height*scaleY);
    }

    float x, y, width, height;
};


class TrackerSettings{
public:
    TrackerSettings():minAge(0), interpolation(.001), maxDistance(30){}

    int minAge;
    float interpolation;
    float maxDistance;
};

#endif // TYPES_H
