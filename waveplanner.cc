/*
 *
 * Trey Amador
 * ID: 012187370
 * CS 521 Robotics
 * Project 4 Wavefront Planner
 * 14 March 2017
 * 
 * 
 * 
 */
#include <libplayerc++/playerc++.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>


namespace {

	const std::string OUTPUT_FILEPATH = "output_wavefront.pnm";
	const std::string ENVIRONMENT = "hospital_section.pnm";
	const double RAD_TO_DEG = 180.0/M_PI;
	const double DEG_TO_RAD = M_PI/180.0;	
	const float MAP_X_COORDINATES = 44.0;
	const float MAP_Y_COORDINATES = 18.0;
	const double MIN_THRESHOLD = 0.4;
	const int OBSTACLE_GROWTH = 18;
	const double THRESHOLD = 1.0;
	const double LASER_MAX = 8.0;
	const int COLOR_FACTOR = 200;
	const int MIDDLE_LASER = 90;
	const double V_SLOW = 0.05;
	const double V_MAX = 10.0;
	const double FACTOR = 0.10;
	const int DIRECTIONS = 8;
	const int GOAL_RANGE = 2;
	const int OCCUPIED = 1;
	const int BERTH = 10;
	const int INDEX = 0;
	const int FREE = 0;
	const int ADJ = 3;

};


// simple geometric data structures
struct Point2D {
	Point2D(double x, double y) : 
		x_(x), y_(y) 
	{}
	double x_, y_;
};


struct PointXY {
	PointXY() :
		x_(0),y_(0)
	{}
	PointXY(int x, int y) :
		x_(x),y_(y)
	{}
	int x_,y_;
};
typedef std::vector<PointXY>::iterator pIterXY;
typedef std::vector<Point2D>::iterator pIter2D;


struct Vector2D {

	Vector2D(double x, double y) :
		x_(x),y_(y)
	{}
	Vector2D(const Vector2D& vec) :
		x_(vec.x_),y_(vec.y_)
	{}
	void operator+=(const Vector2D& vec) {
		this->x_ += vec.x_;
		this->y_ += vec.y_;
	}
	void operator*=(double factor) {
		this->x_ *= factor;
		this->y_ *= factor;
	}
	double magnitude() {
		return std::sqrt(
			std::pow(this->x_,2.0)+
			std::pow(this->y_,2.0));
	}
	double theta() {
		if (this->x_ == 0.0)
			return 0.0;
		return std::atan(this->y_ / this->x_);
	}
	
	double x_, y_;
};


struct BoundingBox {

	BoundingBox(double x, double y) : 
		x_(x), y_(y), range_(0.15)
	{}
	void reposition(double x, double y) {
		x_ = x; 
		y_ = y;
	}
	// only works for axis aligned
	bool collides(Point2D* p) {
		if ((p->x_ > x_ - range_/2 && p->x_ < x_ + range_/2) &&
			(p->y_ > y_ - range_/2 && p->y_ < y_ + range_/2))
			return true;
		return false;
	}

	double x_, y_, range_;
};


struct BoudingCircle {

	BoudingCircle(double cx, double cy, double r) :
		c_x_(cx), c_y_(cy), rad_(r)
	{}
	void reposition(double x, double y) {
		c_x_ = x;
		c_y_ = y;
	}
	bool collides(Point2D& p) {
		double distance = std::sqrt(
			std::pow(c_x_-p.x_,2.0)+ 
			std::pow(c_y_-p.y_,2.0));
		return distance < rad_;
	}
	
	double c_x_, c_y_, rad_;
};


class Map {


public:
	Map(const std::string& filepath);
	~Map();
	
	int wavefront(
		PointXY& pos,
		PointXY& destination);
	std::vector<PointXY> create_path(
		PointXY& init,PointXY& dest,
		int gradient);
	
	int height_pixel();
	int width_pixel();
	
		
private:
	PointXY* propagate_wave(
		PointXY*& perimeter,
		int& size, int gradient);
	
	void gradient_descent(PointXY& point);
	bool query_adjacent(
		PointXY& loc, int grd,
		int off_x, int off_y);
	void adjust_point(PointXY& loc, int off_x, int off_y);
	
	void prune_path();
	bool removable_node(PointXY& beg, PointXY& end);
	
	void grow_obstacles(int size_growth);
	void grow_pixels(
		std::vector<std::vector<double> >& map,
		int row, int col);
	void swap_waves(
		PointXY*& perimeter,
		PointXY*& frontier,
		int size);
	void clear_perimeter(PointXY*& perimeter);
	
	bool goal_reached(
		PointXY*& perimeter,
		int size,
		PointXY& goal);
	bool goal_unreachable(int perimeter_size);
	
	void read(const std::string& filepath);
	void output_wavefront(
		const std::string& filepath,
		const std::string& output_path,
		int gradient);
	
	PointXY cast_point(Point2D& point);
	
	
private:
	std::vector<std::vector<double> > map_;
	std::vector<std::vector<int> > wave_;
	std::vector<PointXY> path_;
	
	
};


Map::Map(const std::string& filepath) {
	this->read(filepath);
	this->grow_obstacles(OBSTACLE_GROWTH);
}


Map::~Map() {
	
}


int Map::wavefront(PointXY& plyr, PointXY& dest) {
	int gradient = 1, size = 1;
	this->wave_[dest.y_][dest.x_] = gradient;
	PointXY* perim = new PointXY[size];
	perim[size-1] = PointXY(dest.x_,dest.y_);
	while (!this->goal_reached(perim,size,plyr)) {
		int perim_s = size;
		PointXY* frontier = this->propagate_wave(
			perim,size,++gradient);
		if (this->goal_unreachable(perim_s))
			return -1;
		this->swap_waves(perim,frontier,size);
	}
	return gradient;
}


std::vector<PointXY> Map::create_path(
	PointXY& init, PointXY& dest, int gradient)
{
	path_ = { PointXY(init.x_,init.y_) };
	while (wave_[path_.back().y_][path_.back().x_] != 1)
		this->gradient_descent(path_.back());
	this->output_wavefront(ENVIRONMENT,OUTPUT_FILEPATH,gradient);
	this->prune_path();
	return path_;
}


PointXY* Map::propagate_wave(
	PointXY*& perimeter,
	int& size,
	int gradient)
{
	int iter = 0;
	PointXY* frontier = new PointXY[DIRECTIONS*size];
	for (int p = 0; p < size; ++p) {
		for (int i = 0; i < ADJ*ADJ; ++i) {
			int r = (perimeter[p].y_-1)+i/ADJ,
				c = (perimeter[p].x_-1)+i%ADJ;
			if (map_[r][c] == 0 && wave_[r][c] == 0) {
				frontier[iter++] = PointXY(c,r);
				wave_[r][c] = gradient;
			}
		}
	}
	size = iter;
	return frontier;
}


void Map::gradient_descent(PointXY& p) {
	int grd = wave_[p.y_][p.x_];
	if (this->query_adjacent(p,grd,1,0))
		this->adjust_point(p,1,0);
	else if (this->query_adjacent(p,grd,-1,0))
		this->adjust_point(p,-1,0);
	else if (this->query_adjacent(p,grd,0,1))
		this->adjust_point(p,0,1);
	else if (this->query_adjacent(p,grd,0,-1))
		this->adjust_point(p,0,-1);
	else if (this->query_adjacent(p,grd,1,1))
		this->adjust_point(p,1,1);
	else if (this->query_adjacent(p,grd,-1,1))
		this->adjust_point(p,-1,1);
	else if (this->query_adjacent(p,grd,1,-1))
		this->adjust_point(p,1,-1);
	else if (this->query_adjacent(p,grd,-1,-1))
		this->adjust_point(p,-1,-1);
	else
		this->adjust_point(p,-1,-1);
}


bool Map::query_adjacent(
	PointXY& loc, int grd,
	int off_x, int off_y)
{
	return
		wave_[loc.y_+off_y][loc.x_+off_x] < grd &&
		wave_[loc.y_+off_y][loc.x_+off_x] > 0 &&
		map_[loc.y_+off_y][loc.x_+off_x] != 1;
}


void Map::adjust_point(PointXY& loc, int off_x, int off_y) {
	path_.push_back(PointXY(
		loc.x_+off_x,
		loc.y_+off_y));
}


void Map::prune_path() {
	pIterXY iter = path_.begin()+1;
	while (iter != path_.end()-1)
		if(this->removable_node(*(iter-1),*(iter+1)))
			iter = path_.erase(iter);
		else
			++iter;
}


bool Map::removable_node(PointXY& init, PointXY& end) {
	double delta_x = static_cast<double>(end.x_-init.x_),
		delta_y = static_cast<double>(end.y_-init.y_);
	if (delta_y == 0.0 || delta_x == 0.0) 
		return true;
	double nrml_x = std::copysign(delta_x/delta_y,delta_x),
		nrml_y = std::copysign(delta_y/delta_y,delta_y);
	int iter = static_cast<int>(std::ceil(std::fabs(delta_y)));
	for (int i = 0; i < iter; ++i) {
		int r = static_cast<int>(init.y_+i*nrml_y),
			c_f = static_cast<int>(init.x_+std::floor(i*nrml_x)),
			c_c = static_cast<int>(init.x_+std::ceil(i*nrml_x));
		if (wave_[r][c_f] == 0 || wave_[r][c_c] == 0)
			return false;
	}
	return true;
}


void Map::grow_obstacles(int span) {
	std::vector<std::vector<double> > map = map_;
	for (size_t r = 0; r < map_.size(); ++r)
		for (size_t c = 0; c < map_[r].size(); ++c)
			if (map_[r][c] == 1)
				this->grow_pixels(map,r,c);
	map_ = map;
}


void Map::grow_pixels(
	std::vector<std::vector<double> >& map,
	int row, int col)
{
	int offset = OBSTACLE_GROWTH/2;
	for (int r = row-offset; r < row+offset; ++r) {
		for (int c = col-offset; c < col+offset; ++c) {
			try {
				map.at(r).at(c) = 1;
			} catch (const std::out_of_range& err) {
				// not much to catch here
			}
		}
	}
}


void Map::swap_waves(
	PointXY*& perimeter, 
	PointXY*& frontier, 
	int size)
{
	this->clear_perimeter(perimeter);
	perimeter = new PointXY[size];
	for (int i = 0; i < size; ++i)
		perimeter[i] = frontier[i];
	this->clear_perimeter(frontier);
}


void Map::clear_perimeter(PointXY*& perimeter) {
	if (perimeter != nullptr) {
		delete [] perimeter;
		perimeter = nullptr;
	}
}


bool Map::goal_reached(PointXY*& perimeter, int size, PointXY& goal) {
	for (int i = 0; i < size; ++i)
		if (perimeter[i].x_ == goal.x_ && perimeter[i].y_ == goal.y_)
			return true;
	return false;
}


bool Map::goal_unreachable(int perimeter_size) {
	if (perimeter_size == 0)
		return true;
	return false;
}


void Map::read(const std::string& filepath) {
	std::string line, header;
	std::ifstream in_file(filepath);
	std::getline(in_file,header);
	int width,height,max_val;
	in_file >> width >> height >> max_val;
	while (std::getline(in_file,line)) {
		map_= std::vector<std::vector<double> >(
			height,std::vector<double>(width));
		wave_ = std::vector<std::vector<int> >(
			height,std::vector<int>(width,0));
		for (size_t i = 0; i < line.size(); ++i)
			map_[i/width][i%width] = static_cast<double>(line[i]+1);
	}
}


void Map::output_wavefront(
	const std::string& filepath,
	const std::string& output_path,
	int gradient)
{
	std::string line, header;
	std::ifstream in_file(filepath);
	std::getline(in_file,header);
	int width,height,max_val;
	in_file >> width >> height >> max_val;
	float max_grd = static_cast<float>(gradient);
	std::ofstream out_file(output_path);
	out_file << header << std::endl;
	out_file << width << " " << height << " " << max_val << std::endl;
	for (size_t r = 0; r < wave_.size(); ++r) {
		for (size_t c = 0; c < wave_[r].size(); ++c) {
			bool path_marked = false;
			for (pIterXY iter = path_.begin(); 
				iter != path_.end() && !path_marked; ++iter) {
				if (iter->x_ == c && iter->y_ == r) {
					out_file << static_cast<char>(-1);
					path_marked = true;
				}
				
			}
			if (!path_marked)
				out_file << (map_[r][c] == 1 || wave_[r][c] == 0 ? 
					static_cast<char>(map_[r][c]-1) : 
					static_cast<char>(
						wave_[r][c]*COLOR_FACTOR/max_grd));
		}
	}
}


int Map::width_pixel() {
	return static_cast<int>(map_[0].size());
}


int Map::height_pixel() {
	return static_cast<int>(map_.size());
}


PointXY Map::cast_point(Point2D& point) {
	return PointXY(
		static_cast<int>(point.x_),
		static_cast<int>(point.y_));
}




class Robot {

public:
	Robot(int num, char* points[]);
	~Robot();
	int loop();


private:
	void navigator(
		PlayerCc::Position2dProxy& pp);
	void pilot(PlayerCc::Position2dProxy& pp);
	void go_to_waypoint(
		PlayerCc::Position2dProxy& pp,
		double& distance, double& angle);
	void avoid_obstacle(
		PlayerCc::LaserProxy& lp,
		double& distance,
		double& angle);
	void traverse(
		PlayerCc::Position2dProxy& pp,
		double velocity, double angle);
	
	void slow_approach(
		double& distance, double& angle,
		double sign);
	bool unobstructed(
		PlayerCc::LaserProxy& lp,
		double velocity,
		double angle);
	bool goal_behind(double angle);
	
	void tangent_field(
		Vector2D& tangent,
		double& distance, double& angle);
	
	Vector2D calculate_vector(
		double magnitude, double radians);
	
	Vector2D perpendicular(
		double magnitude, double radians);
	
	std::vector<Point2D> parse_coordinates(
		int num, char* points[]);
	void set_bound(PlayerCc::Position2dProxy& pp);
	
	PointXY coordinate_to_pixel(Point2D& node);
	Point2D pixel_to_coordinate(PointXY& node);
	
	bool reached_coordinate(
		PlayerCc::Position2dProxy& pp,
		Point2D& destination);	
	bool path_complete();
	void path_unreachable();
	int mission_complete();
	

private:
	std::vector<Point2D> coordinates_;
	BoudingCircle bounding_;
	pIter2D coordinate_;
	Map map_;


};


Robot::Robot(int num, char* points[]) :
	coordinates_(this->parse_coordinates(num,points)),
	bounding_(0.0,0.0,0.0),
	map_(ENVIRONMENT)
{}


Robot::~Robot() {
	coordinates_.clear();
	coordinates_.shrink_to_fit();
}


void Robot::navigator(
	PlayerCc::Position2dProxy& pp)
{
	Point2D init_robot = Point2D(pp.GetXPos(),pp.GetYPos());
	PointXY pos = this->coordinate_to_pixel(init_robot),
			goal = this->coordinate_to_pixel(coordinates_.back());
	coordinates_.clear();
	int gradient = map_.wavefront(pos,goal);
	if (gradient != -1) {
		std::vector<PointXY> path = 
			map_.create_path(pos,goal,gradient);
		for (pIterXY iter = path.begin(); 
			iter != path.end(); ++iter)
			coordinates_.push_back(
				this->pixel_to_coordinate(*iter));
		coordinate_ = coordinates_.begin();
	} else {
		this->path_unreachable();
	}
}


void Robot::pilot(PlayerCc::Position2dProxy& pp) {
	if (!this->path_complete() && 
		this->reached_coordinate(pp,*coordinate_))
		++coordinate_;
}


void Robot::go_to_waypoint(
	PlayerCc::Position2dProxy& pp,
	double& distance, double& angle)
{
	Point2D goal = *coordinate_;
	double distance_y = goal.y_-pp.GetYPos();
	double distance_x = goal.x_-pp.GetXPos();
	distance = std::sqrt(
		std::pow(distance_x,2.0) + 
		std::pow(distance_y,2.0));
	double coordinate_angle = 
		std::copysign(1.0,distance_y) * 
		std::acos(distance_x/distance);
	double yaw_angle = pp.GetYaw();
	angle = coordinate_angle - yaw_angle;
}


void Robot::avoid_obstacle(
	PlayerCc::LaserProxy& lp,
	double& distance,
	double& angle)
{
	if (!this->unobstructed(lp,distance,angle)){
		Vector2D tangential = Vector2D(0.0,0.0);
		int count = lp.GetCount();
		bool margin = true;
		for (int i = 0; i < count && margin; ++i) {
			double magnitude = lp[i];
			double radians = (i-MIDDLE_LASER)*DEG_TO_RAD;
			if (magnitude < MIN_THRESHOLD)
				margin = false;
			else if (magnitude < THRESHOLD)
				tangential += this->perpendicular(magnitude,radians);
		}
		if (this->goal_behind(angle))
			this->slow_approach(distance,angle,-1.0);
		else if (margin || distance < MIN_THRESHOLD)
			this->tangent_field(tangential,distance,angle);
		else
			this->slow_approach(distance,angle,1.0);
	}
}


void Robot::traverse(
	PlayerCc::Position2dProxy& pp,
	double velocity, double angle)
{
	pp.SetSpeed(velocity,angle);
}


void Robot::slow_approach(
	double& distance, double& angle, double sign)
{
	angle = std::copysign(1.0,sign*angle);
	distance = V_SLOW;
}


bool Robot::unobstructed(
	PlayerCc::LaserProxy& lp,
	double velocity,
	double angle)
{
	int degree = static_cast<int>(
		angle*RAD_TO_DEG)+MIDDLE_LASER;
	return 
		degree > MIDDLE_LASER-GOAL_RANGE && 
		degree < MIDDLE_LASER+GOAL_RANGE && 
		velocity < lp[degree];
}


bool Robot::goal_behind(double angle) {
	int degrees = static_cast<int>(std::fabs(angle)*RAD_TO_DEG);
	int perpendicular = static_cast<int>(M_PI*3*RAD_TO_DEG/4);
	return degrees >= perpendicular;
}


void Robot::tangent_field(
	Vector2D& tangential,
	double& distance, double& angle)
{
	Vector2D velocity = this->calculate_vector(
		std::max(V_MAX/distance,V_MAX),angle);
	velocity += tangential;
	distance = std::min(V_MAX,distance);
	angle = velocity.theta();
}


Vector2D Robot::calculate_vector(
	double magnitude,
	double radians) 
{
	return Vector2D(
		magnitude * std::cos(radians),
		magnitude * std::sin(radians));
}


Vector2D Robot::perpendicular(double magnitude, double radians) {
	double weighted = (magnitude != 0.0) ? 
		FACTOR * (THRESHOLD / magnitude) : 0.0;
	return Vector2D(
		weighted*std::cos(radians),
		-weighted*std::sin(radians));
}


std::vector<Point2D> Robot::parse_coordinates(
	int num, char* points[])
{
	std::vector<Point2D> coordinates;
	for (int i = 1; i < num; ++i) {
		std::vector<std::string> tokens;
		std::stringstream ss(points[i]);
		std::string item;
		char x = 'x';
		while(getline(ss,item,x))
			tokens.push_back(item);
		if (tokens.size() >= 2)
			coordinates.push_back(Point2D(
				std::atof(tokens[0].c_str()),
				std::atof(tokens[1].c_str())));
	}
	return coordinates;
}


void Robot::set_bound(PlayerCc::Position2dProxy& pp) {
	pp.RequestGeom();
	bounding_ = BoudingCircle(
		pp.GetXPos(),pp.GetYPos(),
		pp.GetSize().sw/2);
}


Point2D Robot::pixel_to_coordinate(PointXY& node) {
	return Point2D(
		(MAP_X_COORDINATES/map_.width_pixel())*node.x_ -
			MAP_X_COORDINATES/2.0,
		MAP_Y_COORDINATES/2.0 - 
			(MAP_Y_COORDINATES/map_.height_pixel())*node.y_);
}


PointXY Robot::coordinate_to_pixel(Point2D& node) {
	return PointXY(
		static_cast<int>(std::round(map_.width_pixel()/2.0 +
			(map_.width_pixel()/MAP_X_COORDINATES)*node.x_)),
		static_cast<int>(std::round(map_.height_pixel()/2.0 -
			(map_.height_pixel()/MAP_Y_COORDINATES)*node.y_)));
}


bool Robot::reached_coordinate(
	PlayerCc::Position2dProxy& pp,
	Point2D& destination)
{
	bounding_.reposition(pp.GetXPos(),pp.GetYPos());
	if (bounding_.collides(destination))
		return true;
	return false;
}


bool Robot::path_complete() {
	return coordinate_ == coordinates_.end();
}


void Robot::path_unreachable() {
	std::cout << "The goal cannot be reached.\n" << std::endl;
	coordinate_ = coordinates_.end();
}


int Robot::mission_complete() {
	std::cout << "Mission Accomplished.\n" << std::endl;
	return 0;
}


int Robot::loop() {
	try {
		PlayerCc::PlayerClient robot(
			PlayerCc::PLAYER_HOSTNAME,
			PlayerCc::PLAYER_PORTNUM);
		PlayerCc::Position2dProxy pp(&robot,INDEX);
		PlayerCc::LaserProxy lp(&robot,INDEX);
		robot.Read();
		this->set_bound(pp);
		this->navigator(pp);
		while (!this->path_complete()) {
			robot.Read();
			double velocity = 0.0, angle = 0.0;
			this->go_to_waypoint(pp,velocity,angle);
			this->avoid_obstacle(lp,velocity,angle);
			this->traverse(pp,velocity,angle);
			this->pilot(pp);
		}
	} catch (PlayerCc::PlayerError& e) {
		std::cerr << e << std::endl;
		return -1;
	}
	return this->mission_complete();
}


int main(int argc, char* argv[]) {
	if (argc >= 2) {
		Robot robot(argc,argv);
		return robot.loop();
	} else {
		return -1;
	}
}



