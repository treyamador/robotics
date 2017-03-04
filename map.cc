#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>


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


class Map {
	
	
public:
	Map();
	~Map();
	
	void read(const std::string& filepath);
	void wavefront(Point2D& pos, Point2D& destination);
	void print_wavefront();
	void print_map();
	
	bool destination_reached(PointXY& wf,Point2D& pos);
	bool frontier_completed(PointXY& begin, PointXY& end);
	
	bool gradient_left(PointXY& wf, int gradient);
	bool gradient_right(PointXY& wf, int gradient);
	bool gradient_up(PointXY& wf, int gradient);
	bool gradient_down(PointXY& wf, int gradient);
	
	bool gradient_down_right(PointXY& wf, int gradient);
	bool gradient_down_left(PointXY& wf, int gradient);
	bool gradient_up_right(PointXY& wf, int gradient);
	bool gradient_up_left(PointXY& wf, int gradient);
	
	bool barrier_left(PointXY& wf, int gradient);
	bool barrier_right(PointXY& wf, int gradient);
	bool barrier_up(PointXY& wf, int gradient);
	bool barrier_down(PointXY& wf, int gradient);
	
	
	
	
	
private:
	std::vector<std::vector<int> > wavefront_;
	std::vector<std::vector<double> > map_;
	
	
};


Map::Map() {
	
}


Map::~Map() {
	
}


void Map::read(const std::string& filepath) {
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


bool Map::gradient_left(PointXY& wf, int gradient) {
	return wavefront_[wf.y_][wf.x_-1] == gradient-1;
}


bool Map::gradient_right(PointXY& wf, int gradient) {
	return wavefront_[wf.y_][wf.x_+1] == gradient-1;
}


bool Map::gradient_up(PointXY& wf, int gradient) {
	return wavefront_[wf.y_-1][wf.x_] == gradient-1;
}


bool Map::gradient_down(PointXY& wf, int gradient) {
	return wavefront_[wf.y_+1][wf.x_] == gradient-1;
}


bool Map::gradient_down_right(PointXY& wf, int gradient) {
	return
		wavefront_[wf.y_+1][wf.x_+1] == gradient-1 &&
		(wavefront_[wf.y_][wf.x_+1] <= 0 ||
		wavefront_[wf.y_][wf.x_+1] == gradient);
}


bool Map::gradient_down_left(PointXY& wf, int gradient) {
	return
		wavefront_[wf.y_+1][wf.x_-1] == gradient-1 &&
		(wavefront_[wf.y_+1][wf.x_] <= 0 ||
		wavefront_[wf.y_+1][wf.x_] == gradient);
}


bool Map::gradient_up_right(PointXY& wf, int gradient) {
	return
		wavefront_[wf.y_-1][wf.x_+1] == gradient-1 &&
		(wavefront_[wf.y_-1][wf.x_] <= 0 ||
		wavefront_[wf.y_-1][wf.x_] == gradient);
}


bool Map::gradient_up_left(PointXY& wf, int gradient) {
	return
		wavefront_[wf.y_-1][wf.x_-1] == gradient-1 &&
		(wavefront_[wf.y_][wf.x_-1] <= 0 ||
		wavefront_[wf.y_][wf.x_-1] == gradient);
}


void Map::wavefront(Point2D& player, Point2D& dest) {	
	int gradient = 1;
	int dest_x = static_cast<int>(dest.x_),
		dest_y = static_cast<int>(dest.y_);
	wavefront_[dest_y][dest_x] = gradient;
	while (gradient < 6) {
		++gradient;
		int init_y = dest_y;
		while (wavefront_[init_y][dest_x] > 0)
			--init_y;
		PointXY wf = PointXY(dest_x,init_y);
		PointXY end_wave = PointXY(wf.x_,wf.y_);
		do {
			if (map_[wf.y_][wf.x_] == 0) {
				wavefront_[wf.y_][wf.x_] = gradient;
				if (this->gradient_down(wf,gradient) || 
					this->gradient_down_right(wf,gradient))
					++wf.x_;
				else if (this->gradient_up(wf,gradient) ||
					this->gradient_up_left(wf,gradient))
					--wf.x_;
				else if (this->gradient_left(wf,gradient) ||
					this->gradient_down_left(wf,gradient))
					++wf.y_;
				else if (this->gradient_right(wf,gradient) ||
					this->gradient_up_right(wf,gradient))
					--wf.y_;
			} else {
				wavefront_[wf.y_][wf.x_] = -gradient;
				
				
				
			}
			this->print_wavefront();
			
		} while (!this->frontier_completed(wf,end_wave));
		
		
	}
	
}


void Map::print_map() {
	for (size_t r = 0; r < map_.size(); ++r) {
		for (size_t c = 0; c < map_[r].size(); ++c) {
			std::cout << map_[r][c];
		}
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
	
}


void Map::print_wavefront() {
	for (size_t r = 0; r < wavefront_.size(); ++r) {
		for (size_t c = 0; c < wavefront_[r].size(); ++c) {
			std::cout << wavefront_[r][c] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
}


bool Map::destination_reached(PointXY& wf, Point2D& pos) {
	return
		wf.x_ == static_cast<int>(pos.x_) && 
		wf.y_ == static_cast<int>(pos.y_);
}


bool Map::frontier_completed(PointXY& begin, PointXY& end) {
	return
		begin.x_ == end.x_ &&
		begin.y_ == end.y_;
}


int main(int argc, char* argv[]) {
	Point2D dest = Point2D(24.0,15.0);
	Point2D player = Point2D(0.0,0.0);
	Map map;
	map.read("map.txt");
	map.wavefront(player,dest);
	
	
	return 0;
}


