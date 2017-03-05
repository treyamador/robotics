#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>


namespace {
	const std::string ENVIRONMENT = "hospital_copy.pnm";
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
	PointXY(int x, int y) :
		x_(x),y_(y)
	{}
	int x_,y_;
};

typedef std::vector<PointXY>::iterator pIter;


class Map {
	
	
public:
	Map();
	~Map();
	
	void read(const std::string& filepath);
	void read_txt(const std::string& filepath);
	
	void wavefront(Point2D& pos, Point2D& destination);
	void print_wavefront();
	void print_map();
	
	void output_wavefront(const std::string& filepath);
	
	void propagate_wave(
		std::vector<PointXY>& perimeter,
		std::vector<PointXY>& frontier);
	void fill_gradient(
		std::vector<PointXY>& frontier,
		int gradient);
	void replace_perimeter(
		std::vector<PointXY>& perimeter,
		std::vector<PointXY>& frontier);
	
	bool frontier_completed(
		std::vector<PointXY>& frontier, 
		bool completed);
	
	bool goal_reached(
		std::vector<PointXY>& perim,
		PointXY& goal);
	
	void clear_points(std::vector<PointXY>& perim);
	
private:
	std::vector<std::vector<double> > map_;
	std::vector<std::vector<int> > wave_;
	
};


Map::Map() {
	
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
	std::ofstream out_file("output.pnm");
	out_file << width << " " << height << " " << max_val << std::endl;
	for (size_t r = 0; r < map_.size(); ++r)
		for (size_t c = 0; c < map_[r].size(); ++c)
			out_file << (char)(map_[r][c]-1);
}


void Map::read_txt(const std::string& filepath) {
	std::ifstream in_file(filepath); 
	std::string line;
	while (std::getline(in_file,line)) {
		map_.push_back(std::vector<double>(line.size()));
		wave_.push_back(std::vector<int>(line.size(),0));
		for (size_t i = 0; i < map_.back().size(); ++i) {
			map_.back()[i] = line[i]-'0';
		}
	}
}



void Map::output_wavefront(const std::string& filepath) {

	std::string line, header;
	std::ifstream in_file(filepath);
	std::getline(in_file,header);
	int width,height,max_val;
	in_file >> width >> height >> max_val;

	std::ofstream out_file("wavefront.pnm");
	out_file << header << std::endl;
	out_file << width << " " << height << " " << max_val << std::endl;
	for (size_t r = 0; r < wave_.size(); ++r)
		for (size_t c = 0; c < wave_[r].size(); ++c)
			out_file << (map_[r][c] == 1 ? 
				(char)(map_[r][c]) : (char)(wave_[r][c]-1));
}


void Map::clear_points(std::vector<PointXY>& perim) {
	perim.clear();
	perim.shrink_to_fit();
}


void Map::propagate_wave(
	std::vector<PointXY>& perim,
	std::vector<PointXY>& frontier)
{
	for (pIter p = perim.begin(); p != perim.end(); ++p) {
		for (int i = 0; i < ADJ*ADJ; ++i) {
			int r = (p->y_-1)+i/ADJ,
				c = (p->x_-1)+i%ADJ;
			if (map_[r][c] == 0 && wave_[r][c] == 0) {
				frontier.push_back(PointXY(c,r));
			}
		}
	}
}


void Map::fill_gradient(
	std::vector<PointXY>& frnt,
	int gradient)
{
	for (pIter p = frnt.begin(); p != frnt.end(); ++p)
		wave_[p->y_][p->x_] = gradient;
}


void Map::replace_perimeter(
	std::vector<PointXY>& perim,
	std::vector<PointXY>& frontier)
{
	this->clear_points(perim);
	for (size_t i = 0; i < frontier.size(); ++i)
		perim.push_back(frontier[i]);
}

bool Map::frontier_completed(
	std::vector<PointXY>& frontier, 
	bool completed)
{
	return frontier.empty() || completed;
}


bool Map::goal_reached(
	std::vector<PointXY>& perim,
	PointXY& goal)
{
	for (pIter p = perim.begin(); p != perim.end(); ++p)
		if (p->x_ == goal.x_ && p->y_ == goal.y_)
			return true;
	return false;
}


void Map::wavefront(Point2D& player, Point2D& destination) {
	int gradient = 1;
	PointXY dest = PointXY(
		static_cast<int>(destination.x_),
		static_cast<int>(destination.y_));
	PointXY plyr = PointXY(
		static_cast<int>(player.x_),
		static_cast<int>(player.y_));
	this->wave_[dest.y_][dest.x_] = gradient;
	std::vector<PointXY> perimeter = { dest };
	while (!this->goal_reached(perimeter,plyr)) {
		++gradient;
		std::vector<PointXY> frontier;
		this->propagate_wave(perimeter,frontier);
		this->fill_gradient(frontier,gradient);
		this->replace_perimeter(perimeter,frontier);
		this->print_wavefront();
		// check if frontier is empty and goal unreached
	}
}


void Map::print_map() {
	for (size_t r = 0; r < map_.size(); ++r) {
		for (size_t c = 0; c < map_[r].size(); ++c) {
			std::cout << map_[r][c] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
	
}


void Map::print_wavefront() {
	for (size_t r = 0; r < wave_.size(); ++r) {
		for (size_t c = 0; c < wave_[r].size(); ++c) {
			std::cout << std::setw(3) << std::right << 
				(map_[r][c] == 1 ? map_[r][c] : wave_[r][c]);
		}
		std::cout << "\n";
	}
	std::cout << std::endl;
}


int main(int argc, char* argv[]) {	
	Point2D dest = Point2D(28.0,12.0);
	Point2D player = Point2D(8.0,6.0);
	Map map;
	map.read_txt("map2.txt");
	map.wavefront(player,dest);
	map.print_wavefront();
	
	std::cout << "\n\nCompleted\n\n" << std::endl;
	
	return 0;
}



