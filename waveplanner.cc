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
	const std::string ENVIRONMENT = "hospital_section.pnm";
	const std::string OUTPUT_FILEPATH = "output_wavefront.pnm";
	const int COLOR_FACTOR = 200;
	const int DIRECTIONS = 8;
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
typedef std::vector<PointXY>::iterator pIter;


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
		x_(x), y_(y), range_(0.1)
	{}
	void reposition(double x, double y) {
		x_ = x; 
		y_ = y;
	}
	bool collides(Point2D* p) {
		if ((p->x_ > x_ - range_/2 && p->x_ < x_ + range_/2) &&
			(p->y_ > y_ - range_/2 && p->y_ < y_ + range_/2))
			return true;
		return false;
	}

	double x_, y_, range_;
};


class Map {

	
public:
	Map(const std::string& filepath);
	~Map();
	
	int wavefront(
		Point2D& pos,
		Point2D& destination);
	
	PointXY* propagate_wave(
		PointXY*& perimeter,
		int& size,
		int gradient);
	void swap_waves(
		PointXY*& perimeter,
		PointXY*& frontier,
		int size);
		
	bool goal_reached(
		PointXY*& perimeter,
		int size,
		PointXY& goal);
	bool goal_unreachable(int perimeter_size);
	
	void create_path(
		PointXY& init,PointXY& dest,
		int gradient);
	bool query_adjacent(
		PointXY& loc, int grd,
		int off_x, int off_y);
	void adjust_point(PointXY& loc, int off_x, int off_y);
	
	void prune_path();
	bool removable_node(PointXY& beg, PointXY& end);
	
	PointXY cast_point(Point2D& point);
	void clear_perimeter(PointXY*& perimeter);
	
	bool contains_wall(int span, int row, int col);
	void fill_grid(int span, int row, int col);
	
	void read(const std::string& filepath);
	void output_wavefront(
		const std::string& filepath,
		const std::string& output_path,
		int gradient);
	void fill_map();
	
	
private:
	std::vector<std::vector<double> > map_;
	std::vector<std::vector<int> > wave_;
	std::vector<PointXY> path_;
	

};


Map::Map(const std::string& filepath) {
	this->read(filepath);
}


Map::~Map() {
	
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
	for (size_t r = 0; r < wave_.size(); ++r)
		for (size_t c = 0; c < wave_[r].size(); ++c)
			out_file << (map_[r][c] == 1 || wave_[r][c] == 0 ? 
				static_cast<char>(map_[r][c]-1) : 
				static_cast<char>(wave_[r][c]*COLOR_FACTOR/max_grd));
}


bool Map::contains_wall(int span, int row, int col) {
	for (int r = row; r < row+span; ++r)
		for (int c = col; c < col+span; ++c)
			if (map_[r][c] == 1)
				return true;
	return false;
}


void Map::fill_grid(int span, int row, int col) {
	for (int r = row; r < row+span; ++r)
		for (int c = col; c < col+span; ++c)
			map_[r][c] = 1;
}


void Map::fill_map() {
	
	const int span = 4;
	for (size_t r = 0; r < map_.size()-span+1; r+=span) 
		for (size_t c = 0; c < map_[r].size()-span+1; c+=span)
			if (this->contains_wall(span,r,c))
				this->fill_grid(span,r,c);
	
	for (size_t r = 0; r < map_.size()-1; ++r)
		for (size_t c = 0; c < map_[r].size()-1; ++c)
			if ((map_[r][c] == 1 && map_[r+1][c+1] == 1 &&
				map_[r+1][c] == 0 && map_[r][c+1] == 0) || 
				(map_[r][c] == 0 && map_[r+1][c+1] == 0 &&
				map_[r+1][c] == 1 && map_[r][c+1] == 1))
				map_[r][c] = map_[r][c+1] = 1;
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


void Map::swap_waves(PointXY*& perimeter, PointXY*& frontier, int size) {
	this->clear_perimeter(perimeter);
	perimeter = new PointXY[size];
	for (int i = 0; i < size; ++i)
		perimeter[i] = frontier[i];
	this->clear_perimeter(frontier);
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


void Map::create_path(
	PointXY& init, PointXY& dest, int gradient)
{
	path_ = { PointXY(init.x_,init.y_) };
	while (wave_[path_.back().y_][path_.back().x_] != 1) {
		PointXY p = path_.back();
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
			this->adjust_point(p,1,1);
	}
	this->prune_path();
	
	for (int i = 0; i < path_.size(); ++i)
		std::cout << 
			path_[i].x_ << " " << 
			path_[i].y_ << " " << 
			wave_[path_[i].y_][path_[i].x_] << "\n";
	
}


bool Map::query_adjacent(
	PointXY& loc, int grd,
	int off_x, int off_y)
{
	return 
		wave_[loc.y_+off_y][loc.x_+off_x] < grd && 
		map_[loc.y_+off_y][loc.x_+off_x] != 1;
}


void Map::adjust_point(PointXY& loc, int off_x, int off_y) {
	path_.push_back(PointXY(
		loc.x_+off_x,
		loc.y_+off_y));
}


bool Map::removable_node(PointXY& init, PointXY& end) {
	float delta_x = end.x_-init.x_,
		delta_y = end.y_-init.y_;
	int iter = static_cast<int>(std::round(std::fabs(delta_y)));
	delta_x /= delta_y;
	int row = init.y_, col = init.x_;
	for (int i = 0; i < iter && row < end.y_; ++i) {
		col = static_cast<int>(std::round(col + delta_x));
		if (wave_[++row][col] == 0)
			return false;
	}
	return true;
}


void Map::prune_path() {
	pIter iter = path_.begin()+1;
	while (iter != path_.end()-1)
		if(this->removable_node(*(iter-1),*(iter+1)))
			iter = path_.erase(iter);
		else
			++iter;
}


int Map::wavefront(Point2D& player, Point2D& destination) {
	int gradient = 1, size = 1;
	PointXY dest = this->cast_point(destination),
			plyr = this->cast_point(player);
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


PointXY Map::cast_point(Point2D& point) {
	return PointXY(
		static_cast<int>(point.x_),
		static_cast<int>(point.y_));
}


void Map::clear_perimeter(PointXY*& perimeter) {
	if (perimeter != nullptr) {
		delete [] perimeter;
		perimeter = nullptr;
	}
}



class Robot {


public:
	Robot(int num, char* points[]);
	~Robot();
	int loop();


private:
	void navigator(
		PlayerCc::Position2dProxy& pp);
	
	void smooth_path();
	
	
	std::vector<Point2D> parse_coordinates(
		int num, char* points[]);
	int mission_complete();
	

private:
	std::vector<Point2D> coordinates_;
	BoundingBox* bound_;
	Map map_;

};


Robot::Robot(int num, char* points[]) :
	coordinates_(this->parse_coordinates(num,points)),
	bound_(new BoundingBox(0.0,0.0)),
	map_(ENVIRONMENT)
{}


Robot::~Robot() {
	delete bound_;
	coordinates_.clear();
	coordinates_.shrink_to_fit();
}


void Robot::navigator(
	PlayerCc::Position2dProxy& pp)
{
	// coordinates_;
	this->smooth_path();
	
}


void Robot::smooth_path() {
	
	//map_.smooth_path();
	
	
	
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
				std::atoi(tokens[0].c_str()),
				std::atoi(tokens[1].c_str())));
	}
	return coordinates;
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
		pp.RequestGeom();
		this->navigator(pp);
		while (true) {
			robot.Read();
			double velocity = 0.0, angle = 0.0;
			//this->go_to(pp,goal_,velocity,angle);
			//this->avoid_obstacle(lp,velocity,angle);
			//this->pilot(pp,velocity,angle);
		}
	} catch (PlayerCc::PlayerError& e) {
		std::cerr << e << std::endl;
		return -1;
	}
	return this->mission_complete();
}



int main(int argc, char* argv[]) {
	
	Point2D dest = Point2D(700.0,400.0);
	Point2D player = Point2D(50.0,50.0);
	Map map = Map(ENVIRONMENT);
	map.fill_map();
	int gradient = map.wavefront(player,dest);
	if (gradient != -1)
		std::cout << "Gradient complete" << std::endl;
	map.output_wavefront(ENVIRONMENT,OUTPUT_FILEPATH,gradient);
	PointXY dst = map.cast_point(dest),
			plyr = map.cast_point(player);
	map.create_path(plyr,dst,gradient);
	
	return 0;
	
	
	
	//Robot robot(argc,argv);
	//return robot.loop();
	
}






