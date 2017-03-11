#include "depthTracker.h"
#include "object_type_db.h"



void DepthTracker::_bind_methods(){
	ObjectTypeDB::bind_method(_MD("newPoints"), &DepthTracker::newPoints);

	ObjectTypeDB::bind_method(_MD("setMaxDistance", "float"), &DepthTracker::setMaxDistance);
	ObjectTypeDB::bind_method(_MD("getMaxDistance"), &DepthTracker::getMaxDistance);
	ADD_PROPERTY(PropertyInfo(Variant::REAL,"maxDistance"), _SCS("setMaxDistance"), _SCS("getMaxDistance"));

	ObjectTypeDB::bind_method(_MD("setMinAge", "float"), &DepthTracker::setMinAge);
	ObjectTypeDB::bind_method(_MD("getMinAge"), &DepthTracker::getMinAge);
	ADD_PROPERTY(PropertyInfo(Variant::REAL,"minAge"), _SCS("setMinAge"), _SCS("getMinAge"));

	ObjectTypeDB::bind_method(_MD("setInterpolation", "float"), &DepthTracker::setInterpolation);
	ObjectTypeDB::bind_method(_MD("getInterpolation"), &DepthTracker::getInterpolation);
	ADD_PROPERTY(PropertyInfo(Variant::REAL,"interpolation"), _SCS("setInterpolation"), _SCS("getInterpolation"));

	ObjectTypeDB::bind_method(_MD("setDebugRender", "bool"), &DepthTracker::setDebugRender);
	ObjectTypeDB::bind_method(_MD("getDebugRender"), &DepthTracker::getDebugRender);
	ADD_PROPERTY( PropertyInfo(Variant::BOOL,"debugRender"), _SCS("setDebugRender"), _SCS("getDebugRender"));

}

DepthTracker::DepthTracker():
	maxDistance(.5f),
	minAge(1),
	interpolation(0),
	bDebugRender(true),
	maxPointsIn(12),
	curId(0),
	clusters(nullptr)
{}


void DepthTracker::_notification(int what){
	switch(what){
	case NOTIFICATION_PARENTED:
		clusters = get_parent()->cast_to<DepthClusters>();
		if(hasDepthClusters())
			clusters->connect("newPoints", this, "newPoints");
		break;

	case NOTIFICATION_ENTER_TREE:
		for(auto& t: testCubes){
			memdelete_notnull(t);
		}

		for(auto i=0; i<15; i++){
			testCubes.push_back(memnew(TestCube));
			testCubes.back()->hide();
			add_child(testCubes.back(), false);
		}
	}
}

float DepthTracker::getMaxDistance() const{
	return maxDistance;
}

void DepthTracker::setMaxDistance(float value){
	maxDistance = value;
}

float DepthTracker::getMinAge() const{
	return minAge;
}

void DepthTracker::setMinAge(float value){
	minAge = value;
}

float DepthTracker::getInterpolation() const{
	return interpolation;
}

void DepthTracker::setInterpolation(float value){
	interpolation = value;
}

bool DepthTracker::getDebugRender() const{
	return bDebugRender;
}

void DepthTracker::setDebugRender(bool value){
	bDebugRender = value;
}

bool DepthTracker::hasDepthClusters(){
	return clusters;
}

DepthClusters *DepthTracker::getDepthClusters(){
	return clusters;
}

void DepthTracker::newPoints(){
	if(hasDepthClusters())
		setInput(getDepthClusters()->getPoints());
}

void DepthTracker::setInput(std::vector<Vector3> ps){
	float maxDS = maxDistance; // * maxDistance;

	//high values can result in a crash!
	if(ps.size()>maxPointsIn)
		ps.resize(maxPointsIn);

	PointList pointsNew;
	for(auto p: ps){
		pointsNew.push_back(Point(Vector2(p.x, p.z)));
	}

	// remove points that are older than 1 second
	pointsActive.erase(std::remove_if(pointsActive.begin(), pointsActive.end(),
									  [](const Point& p){
		return p.getAge() < 1.f;
	}), pointsActive.end());

	//
	using MatchPair = std::pair<size_t, size_t>;
	using MatchDistance = std::pair<MatchPair, float>;

	std::vector<MatchDistance> distances;

	//find closest matches
	for(size_t i=0; i<points.size(); i++){
		for(size_t j=0; j<pointsNew.size(); j++){
			auto dist = points[i].position.distance_to(pointsNew[j].position);
			distances.push_back(MatchDistance(MatchPair(i, j), dist));
		}
	}

	//sort by distances
	std::sort(distances.begin(), distances.end(), [](const MatchDistance& a, const MatchDistance& b){
		return a.second < b.second;
	});

	std::vector<bool> matchedCur(points.size(), false);
	std::vector<bool> matchedNew(points.size(), false);

	std::vector<MatchPair> matches;
	for(auto d: distances){
		if(d.second < maxDS && !matchedCur[d.first.first] && !matchedNew[d.first.second]){
			//LogInfo() << d.second << " - " << maxDS;
			matches.push_back(d.first);
			matchedCur[d.first.first] = true;
			matchedNew[d.first.second] = true;
		}
	}

	//Copy old point data to new matches
	for(auto m: matches){
		pointsNew[m.second].created = points[m.first].created;
		pointsNew[m.second].id = points[m.first].id;
		if(interpolation > 0.f){
			pointsNew[m.second].position = points[m.first].position.linear_interpolate(pointsNew[m.second].position, interpolation);
		}
	}

	//assign IDs to points, also a way to detect new points
	for(auto& p: pointsNew){
		if(p.id == 0){
			if(p.getAge() >= minAge){
				p.id = newId();
				//signalAdded.emit(p);
			}
		}else{
			//signalMoved.emit(p);
		}
	}

	//trigger remove events
	for(size_t i=0; i<points.size(); i++){
		if(points[i].id == 0)
			continue;

		bool matched = false;
		for(auto m: matches){
			if(m.first == i){
				matched = true;
				continue;
			}
		}
		if(!matched){
			//signalRemoved.emit(points[i]);
		}
	}

	points = pointsNew;

	//copy only points with valid ids to the active set
	pointsActive = points;
	pointsActive.erase(std::remove_if(pointsActive.begin(), pointsActive.end(),
									  [](const Point& p){
		return p.id==0;
	}), pointsActive.end());


	// update render
	size_t i = 0;
	if(bDebugRender){
		for(; i<pointsActive.size(); i++){
			if(i<testCubes.size()){
				auto cube = testCubes[i];
				cube->show();
				cube->set_translation(Vector3(points[i].position.x, 0, points[i].position.y));
				cube->set_scale({.1, .1, .1});
			}
		}
	}

	for(; i<testCubes.size(); i++){
		testCubes[i]->hide();
	}
}

unsigned DepthTracker::newId(){
	curId++;
	if(curId == 0){
		curId = 1;
	}

	return curId;
}
