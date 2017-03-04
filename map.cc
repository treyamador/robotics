#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>


namespace {
	
	
	
	
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
	PointXY adjacent_greatest(PointXY& wf);
	
	bool right_open(PointXY& wf, PointXY& adj);
	bool up_open(PointXY& wf, PointXY& adj);
	bool left_open(PointXY& wf, PointXY& adj);
	
	PointXY right_element(PointXY& wf, PointXY& adj);
	PointXY up_element(PointXY& wf, PointXY& adj);
	PointXY left_element(PointXY& wf, PointXY& adj);

	
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


PointXY Map::adjacent_greatest(PointXY& wf) {
	int max_grad = 0, V = 3;
	PointXY adjacent = PointXY(0,0);
	for (int i = 1; i < V*V; i+=2) {
		int r = (wf.y_-1)+i/V,
			c = (wf.x_-1)+i%V;
		if (std::fabs(wavefront_[r][c]) > std::fabs(max_grad)) {
			max_grad = wavefront_[r][c];
			adjacent = PointXY(c,r);
		}
	}
	return adjacent;
}


bool Map::right_open(PointXY& wf, PointXY& adj) {
	//PointXY right = this->right_element(wf,adj);
	//return wavefront_[right.y_][right.x_] <= 0;
	PointXY r = this->right_element(wf,adj);
	return wavefront_[r.y_][r.x_] <= 0;
}


bool Map::up_open(PointXY& wf, PointXY& adj) {
	PointXY up = this->up_element(wf,adj);
	return wavefront_[up.y_][up.x_] <= 0;
}


bool Map::left_open(PointXY& wf, PointXY& adj) {
	PointXY left = this->left_element(wf,adj);
	return wavefront_[left.y_][left.x_] <= 0;
}


PointXY Map::right_element(PointXY& wf, PointXY& adj) {
	//int col = adj.x_-wf.x_ == 1 ? 0 : adj.x_-wf.x_+1,
	//	row = adj.y_-wf.y_ == 1 ? 0 : adj.y_-wf.y_+1;
	//return PointXY(wf.x_+col,wf.y_+row);
	
	PointXY dspl = PointXY(adj.x_-wf.x_,adj.y_-wf.y_);
	if (dspl.x_ == 0 && dspl.y_ == 1)
		dspl = PointXY(1,0);
	else if (dspl.x_ == 1 && dspl.y_ == 0)
		dspl = PointXY(0,-1);
	else if (dspl.x_ == 0 && dspl.y_ == -1)
		dspl = PointXY(-1,0);
	else if (dspl.x_ == -1 && dspl.y_ == 0)
		dspl = PointXY(0,1);
	return PointXY(wf.x_+dspl.x_,wf.y_+dspl.y_);
}


PointXY Map::up_element(PointXY& wf, PointXY& adj) {
	PointXY diff = PointXY(-(adj.x_-wf.x_),-(adj.y_-wf.y_));
	return PointXY(wf.x_+diff.x_,wf.y_+diff.y_);
}


PointXY Map::left_element(PointXY& wf, PointXY& adj) {
	int col = adj.x_-wf.x_ == -1 ? 0 : adj.x_-wf.x_-1,
		row = adj.y_-wf.y_ == -1 ? 0 : adj.y_-wf.y_-1;
	return PointXY(wf.x_+col,wf.y_+row);
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
		PointXY wf = PointXY(dest_x,init_y),
				end_wave = PointXY(wf.x_,wf.y_);
		do {
			if (map_[wf.y_][wf.x_] == 0)
				wavefront_[wf.y_][wf.x_] = gradient;
			else
				wavefront_[wf.y_][wf.x_] = -gradient;
			PointXY adj_wave = this->adjacent_greatest(wf);
			
			//std::cout << "\n( " << 
			//	wf.x_ << " x " << wf.y_ << " ), " << " ( " << 
			//	adj_wave.x_ << " x " << adj_wave.y_ << " )\n\n" << std::endl;
			
			if (this->right_open(wf,adj_wave))
				wf = this->right_element(wf,adj_wave);
			else if (this->up_open(wf,adj_wave))
				wf = this->up_element(wf,adj_wave);
			else if (this->left_open(wf,adj_wave))
				wf = this->left_element(wf,adj_wave);
			//else move back
			
			
			//this->print_wavefront();
			
			
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
	//for (size_t r = 0; r < wavefront_.size(); ++r) {
	//	for (size_t c = 0; c < wavefront_[r].size(); ++c) {
	//		std::cout << wavefront_[r][c] << " ";
	//	}
	//	std::cout << std::endl;
	//}
	//std::cout << std::endl << std::endl;
	
	for (size_t r = 0; r < wavefront_.size(); ++r) {
		for (size_t c = 0; c < wavefront_[r].size(); ++c) {
			std::cout << std::setw(3) << std::right << wavefront_[r][c];
		}
		std::cout << "\n";
	}
	std::cout << std::endl;
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
	//map.read("map.txt");
	map.read("empty_map.txt");
	map.wavefront(player,dest);
	return 0;
}


