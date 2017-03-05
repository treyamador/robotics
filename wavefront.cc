#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>


namespace {
	
	const std::string ENVIRONMENT = "hospital_section.pnm";
	
	
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


typedef std::vector<PointXY>::iterator PointIter;


class Map {
	
	
public:
	Map();
	~Map();
	
	void wavefront(Point2D& pos, Point2D& destination);
	
	std::vector<PointXY> create_compass(
		PointXY& wf,
		PointXY& prev);
	PointXY assess_movement(
		std::vector<PointXY>& adj,
		PointXY& prev);
	PointXY greatest_gradient(
		std::vector<PointXY>& adj,
		PointXY& prev);
	PointXY follow_wall(
		std::vector<PointXY>& adj,
		PointXY& prev);
	
	bool wall_adjacent(PointXY& pos);
	bool destination(PointXY& wf,Point2D& pos, bool cmplt);
	bool frontier_completed(PointXY& wave, PointXY& end);
	
	void read(const std::string& filepath);
	void read_txt(const std::string& filepath);
	
	void output_wavefront(const std::string& filepath);
	//void output_txt(const std::string& filepath);
	void print_wavefront();
	void print_map();


private:
	std::vector<std::vector<int> > wavefront_;
	std::vector<std::vector<double> > map_;
	
	
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
		wavefront_ = std::vector<std::vector<int> >(
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
		wavefront_.push_back(std::vector<int>(line.size(),0));
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
	for (size_t r = 0; r < wavefront_.size(); ++r)
		for (size_t c = 0; c < wavefront_[r].size(); ++c)
			out_file << (char)(wavefront_[r][c]-1);
}


std::vector<PointXY> Map::create_compass(PointXY& wf, PointXY& prev) {
	std::vector<PointXY> compass;
	PointXY dspl = PointXY(prev.x_-wf.x_,prev.y_-wf.y_);
	if (dspl.x_ == 0 && dspl.y_ == 1)
		compass = {
			PointXY(wf.x_+1,wf.y_),
			PointXY(wf.x_,wf.y_-1),
			PointXY(wf.x_-1,wf.y_) 
		};
	else if (dspl.x_ == -1 && dspl.y_ == 0)
		compass = {
			PointXY(wf.x_,wf.y_+1),
			PointXY(wf.x_+1,wf.y_),
			PointXY(wf.x_,wf.y_-1) 
		};
	else if (dspl.x_ == 0 && dspl.y_ == -1)
		compass = {
			PointXY(wf.x_-1,wf.y_),
			PointXY(wf.x_,wf.y_+1),
			PointXY(wf.x_+1,wf.y_) 
		};
	else if (dspl.x_ == 1 && dspl.y_ == 0)
		compass = {
			PointXY(wf.x_,wf.y_-1),
			PointXY(wf.x_-1,wf.y_),
			PointXY(wf.x_,wf.y_+1) 
		};
	else
		compass = {
			PointXY(prev.x_,prev.y_)
		};
	return compass;
}


PointXY Map::assess_movement(
	std::vector<PointXY>& adj,
	PointXY& prev)
{
	PointIter p = adj.begin();
	while (p != adj.end())
		if (map_[p->y_][p->x_] == 1)
			p = adj.erase(p);
		else if (wavefront_[p->y_][p->x_] == 0)
			return PointXY(p->x_,p->y_);
		else
			++p;
	return this->follow_wall(adj,prev);
}


PointXY Map::follow_wall(
	std::vector<PointXY>& adj,
	PointXY& prev)
{
	for (PointIter p = adj.begin(); p != adj.end(); ++p)
		if (this->wall_adjacent(*p))
			return PointXY(p->x_,p->y_);
	return PointXY(prev.x_,prev.y_);
}


bool Map::wall_adjacent(PointXY& pos) {
	int V = 3;
	for (int i = 0; i < V*V; ++i) {
		int r = (pos.y_-1)+i/V,
			c = (pos.x_-1)+i%V;
		if (map_[r][c] == 1 && i != V*V/2)
			return true;
	}
	return false;
}


PointXY Map::greatest_gradient(
	std::vector<PointXY>& adj,
	PointXY& prev)
{
	PointXY max_pos = PointXY(prev.x_,prev.y_);
	int max_grd = 0;
	for (PointIter p = adj.begin(); p != adj.end(); ++p) {
		int grd = wavefront_[p->y_][p->x_];
		if (grd > max_grd) {
			max_pos = PointXY(p->x_,p->y_);
			max_grd = grd;
		}
	}
	return max_pos;
}


void Map::wavefront(Point2D& player, Point2D& dest) {
	int gradient = 1;
	bool completed = false;
	int dest_x = static_cast<int>(dest.x_),
		dest_y = static_cast<int>(dest.y_);
	wavefront_[dest_y][dest_x] = gradient;
	while (!completed) {
		++gradient;
		int init_y = dest_y;
		// revise this while loop?
		while (wavefront_[init_y][dest_x] != 0 &&
			map_[init_y-1][dest_x] != 1)
			--init_y;
		//std::cout << dest_x << " " << init_y << std::endl;
		PointXY wf = PointXY(dest_x,init_y),
				prev = PointXY(dest_x,init_y+1),
				end_wave = PointXY(dest_x,init_y);
		do {
			std::cout << "\n\n" << wf.x_ << " " << wf.y_ << "\n\n";
			std::vector<PointXY> compass = 
				this->create_compass(wf,prev);
			PointXY mvmt = this->assess_movement(compass,prev);
			prev = PointXY(wf.x_,wf.y_);
			wf = PointXY(mvmt.x_,mvmt.y_);
			if (wavefront_[wf.y_][wf.x_] == 0 && 
				map_[wf.y_][wf.x_] == 0)
				wavefront_[wf.y_][wf.x_] = gradient;
			completed = this->destination(wf,player,completed);
			this->print_wavefront();
		} while (!this->frontier_completed(wf,end_wave));
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
	for (size_t r = 0; r < wavefront_.size(); ++r) {
		for (size_t c = 0; c < wavefront_[r].size(); ++c) {
			std::cout << std::setw(3) << std::right << 
				(map_[r][c] == 1 ? map_[r][c] : wavefront_[r][c]);
		}
		std::cout << "\n";
	}
	std::cout << std::endl;
}


bool Map::destination(PointXY& wf, Point2D& pos, bool cmplt) {
	return
		(wf.x_ == static_cast<int>(pos.x_) && 
		wf.y_ == static_cast<int>(pos.y_)) || 
		cmplt;
}


bool Map::frontier_completed(PointXY& wave, PointXY& end) {
	return 
		wave.x_ == end.x_ &&
		wave.y_ == end.y_;
}


int main(int argc, char* argv[]) {
	
	//Point2D dest = Point2D(500.0,200.0);
	//Point2D player = Point2D(100.0,100.0);
	//Map map;
	//map.read(ENVIRONMENT);
	//map.wavefront(player,dest);
	//std::cout << "\n\nCompleted\n\n" << std::endl;
	//map.output_wavefront(ENVIRONMENT);
	//return 0;
	
	
	//Map map;
	//map.read_txt("test.txt");
	
	//PointXY wf = PointXY(3,5),
	//		prev = PointXY(5,4);
	//std::vector<PointXY> compass = map.create_compass(wf,prev);
	
	//std::cout << wf.x_ << " " << wf.y_ << "\n";
	//std::cout << prev.x_ << " " << prev.y_ << "\n";
	//for (size_t i = 0; i < compass.size(); ++i)
	//	std::cout << compass[i].x_ << " " << compass[i].y_ << "\n";
	
	//if (map.wall_adjacent(wf))
	//	std::cout << "wall adjacent\n";
	//else
	//	std::cout << "not adjacent\n";
	
	
	Point2D dest = Point2D(13.0,8.0);
	Point2D player = Point2D(8.0,6.0);
	Map map;
	map.read_txt("test.txt");
	map.wavefront(player,dest);
	//std::cout << "\n\nCompleted\n\n" << std::endl;
	//map.output_wavefront(ENVIRONMENT);
	map.print_wavefront();
	return 0;
	
	
	
	
}




