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


void Map::wavefront(Point2D& player, Point2D& dest) {	
	int gradient = 1;
	int dest_x = static_cast<int>(dest.x_),
		dest_y = static_cast<int>(dest.y_);
	wavefront_[dest_x][dest_y] = gradient;
	this->print_wavefront();
	while (true) {
		++gradient;
		
		//std::cout << "\n\n\n" << gradient << "\n\n\n" << std::endl;
		
		int init_y = dest_y;
		while (wavefront_[dest_x][init_y] > 0)
			--init_y;
		PointXY wf = PointXY(dest_x,init_y);
		PointXY end_wave = PointXY(wf.x_,wf.y_);
		do {
			if (map_[wf.x_][wf.y_] == 0) {
				wavefront_[wf.x_][wf.y_] = gradient;
				if (wavefront_[wf.x_][wf.y_+1] > 0 || 
					wavefront_[wf.x_+1][wf.y_+1] > 0)
					++wf.x_;
				else if (wavefront_[wf.x_][wf.y_-1] > 0 ||
					wavefront_[wf.x_-1][wf.y_-1] > 0)
					--wf.x_;
				else if (wavefront_[wf.x_-1][wf.y_] > 0 ||
					wavefront_[wf.x_-1][wf.y_+1] > 0)
					++wf.y_;
				else if (wavefront_[wf.x_+1][wf.y_] > 0 ||
					wavefront_[wf.x_-1][wf.y_-1] > 0)
					--wf.y_;
			} else {
				
			}
			
			
			
		} while (!this->frontier_completed(wf,end_wave));
		
		//this->print_wavefront();
	}
	
}


/*

void Map::wavefront(Point2D& player, Point2D& dest) {
	
	PointXY wf = PointXY(						// the wavefront point
		static_cast<int>(dest.x_),
		static_cast<int>(dest.y_));
	PointXY last_open = PointXY(wf.x_,wf.y_);	// set each time a pixel is open
	int gradient = 1;
	wavefront_[wf.y_][wf.x_] = gradient;
	
	int iter = 0;
	
	while (!destination_reached(wf,player)) {
		
		wf = PointXY(last_open.x_,last_open.y_+1);
		PointXY init = PointXY(wf.x_++,wf.y_);
		++gradient;
		int hrz = 0, vrt = 0;
		
		while (!frontier_completed(wf,init)) {
			
			// fill in
			if (map_[wf.x_][wf.y_] == 0) {
				// in open space
				wavefront_[wf.x_][wf.y_] = gradient;
				last_open = PointXY(wf.x_,wf.y_);
				
				// decide horizontal movement
				if (wavefront_[wf.x_-1][wf.y_] == gradient)
					hrz = 1;
				else if (wavefront_[wf.x_+1][wf.y_] == gradient)
					hrz = -1;
				else
					hrz = 0;
				
				// decide vertical movement
				if (wavefront_[wf.x_][wf.y_+1] == gradient)
					vrt = -1;
				else if (wavefront_[wf.x_][wf.y_-1] == gradient)
					vrt = 1;
				else
					vrt = 0;
				
			} else {
				// on wall
				wavefront_[wf.x_][wf.y_] = -gradient;
				
				if (wavefront_[wf.x_-1][wf.y_] == -gradient)
					hrz = 1;
				else if (wavefront_[wf.x_+1][wf.y_] == -gradient)
					hrz = -1;
				else
					hrz = 0;
				
				if (wavefront_[wf.x_][wf.y_+1] == -gradient)
					vrt = -1;
				else if (wavefront_[wf.x_][wf.y_-1] == -gradient)
					vrt = 1;
				else
					vrt = 0;
				
			}
			
			// move
			wf.x_ += hrz;
			wf.y_ += vrt;
			
			this->print();
			
		}
		
		
		// fill in initial pixel
		if (map_[init.x_][init.y_] == 0) {
			wavefront_[init.x_][init.y_] = gradient;
			last_open = PointXY(init.x_,init.y_);
		} else {
			wavefront_[init.x_][init.y_] = -gradient;
		}
		
		this->print();
		
		
			
	}
}

*/



/*
void Map::wavefront(Point2D& player, Point2D& dest) {
	PointXY wf = PointXY(
		static_cast<int>(dest.x_),
		static_cast<int>(dest.y_));
	int gradient = 1;
	wavefront_[wf.y_][wf.x_] = gradient;
	while (!destination_reached(wf,player)) {
		++gradient;
		PointXY init = PointXY(wf.x_,wf.y_++);
		while (!frontier_completed(wf,init)) {
			
		}
		
		
		
		// 2,762.09
		
	}
	
	
}
*/


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
	
	// 24 x 15
	Point2D dest = Point2D(20.0,10.0);
	Point2D player = Point2D(0.0,0.0);
	Map map;
	map.read("map.txt");
	//map.print_wavefront();
	map.wavefront(player,dest);
	
	
	return 0;
}


