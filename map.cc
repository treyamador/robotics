#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
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


class Map {
	
	
public:
	Map();
	~Map();
	
	void read(const std::string& filepath);
	void wavefront(Point2D& pos, Point2D& destination);
	void print_wavefront();
	void print_map();
	
	void output_wavefront(const std::string& filepath);
	
	bool destination(PointXY& wf,Point2D& pos, bool cmplt);
	bool frontier_completed(PointXY& wave, PointXY& end);
	PointXY adjacent_greatest(PointXY& wf);
	
	bool right_open(PointXY& wf, PointXY& adj, int grd);
	bool up_open(PointXY& wf, PointXY& adj, int grd);
	bool left_open(PointXY& wf, PointXY& adj, int grd);
	
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


PointXY Map::adjacent_greatest(PointXY& wf) {
	std::vector<PointXY> compass = {
		PointXY(wf.x_,wf.y_-1),
		PointXY(wf.x_,wf.y_+1),
		PointXY(wf.x_-1,wf.y_),
		PointXY(wf.x_+1,wf.y_)
	};
	int max_grd = 0;
	PointXY adjacent = PointXY(0,0);
	for (std::vector<PointXY>::iterator iter = compass.begin(); 
		iter != compass.end(); ++iter)
	{
		int curr_grd = wavefront_[iter->y_][iter->x_];
		if (std::abs(curr_grd) > std::abs(max_grd)) {
			adjacent = PointXY(iter->x_,iter->y_);
			max_grd = curr_grd;
		}
	}
	return adjacent;
}


bool Map::right_open(PointXY& wf, PointXY& adj, int grd) {
	PointXY right = this->right_element(wf,adj);
	return
		wavefront_[right.y_][right.x_] <= 0 ||
		std::abs(wavefront_[right.y_][right.x_]) == grd;
}


bool Map::up_open(PointXY& wf, PointXY& adj, int grd) {
	PointXY up = this->up_element(wf,adj);
	return
		wavefront_[up.y_][up.x_] <= 0 ||
		std::abs(wavefront_[up.y_][up.x_]) == grd;
}


bool Map::left_open(PointXY& wf, PointXY& adj, int grd) {
	PointXY left = this->left_element(wf,adj);
	return
		wavefront_[left.y_][left.x_] <= 0 ||
		std::abs(wavefront_[left.y_][left.x_]) == grd;
}


PointXY Map::right_element(PointXY& wf, PointXY& adj) {
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
	return PointXY(
		-(adj.x_-wf.x_)+wf.x_,
		-(adj.y_-wf.y_)+wf.y_);
}


PointXY Map::left_element(PointXY& wf, PointXY& adj) {
	PointXY dspl = PointXY(adj.x_-wf.x_,adj.y_-wf.y_);
	if (dspl.x_ == 0 && dspl.y_ == 1)
		dspl = PointXY(-1,0);
	else if (dspl.x_ == 1 && dspl.y_ == 0)
		dspl = PointXY(0,1);
	else if (dspl.x_ == 0 && dspl.y_ == -1)
		dspl = PointXY(1,0);
	else if (dspl.x_ == -1 && dspl.y_ == 0)
		dspl = PointXY(0,-1);
	return PointXY(wf.x_+dspl.x_,wf.y_+dspl.y_);
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
		while (wavefront_[init_y][dest_x] > 0)
			--init_y;
		PointXY wf = PointXY(dest_x,init_y),
				end_wave = PointXY(dest_x,init_y);
		do {
			wavefront_[wf.y_][wf.x_] = 
				map_[wf.y_][wf.x_] == 0 ? gradient : -gradient;
			PointXY adj_wave = this->adjacent_greatest(wf);
			if (this->right_open(wf,adj_wave,gradient))
				wf = this->right_element(wf,adj_wave);
			else if (this->up_open(wf,adj_wave,gradient))
				wf = this->up_element(wf,adj_wave);
			else if (this->left_open(wf,adj_wave,gradient))
				wf = this->left_element(wf,adj_wave);
			completed = this->destination(wf,player,completed);	
			
			std::cout << wf.x_ << " " << wf.y_ << std::endl;
					
		} while (!this->frontier_completed(wf,end_wave));
		this->output_wavefront(ENVIRONMENT);
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
	Point2D dest = Point2D(500.0,200.0);
	Point2D player = Point2D(100.0,100.0);
	Map map;
	map.read(ENVIRONMENT);
	map.wavefront(player,dest);
	std::cout << "\n\nCompleted\n\n" << std::endl;
	map.output_wavefront(ENVIRONMENT);
	return 0;
}


