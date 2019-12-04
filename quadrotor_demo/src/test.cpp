#include <iostream>
#include <vector>
#include <math.h>
#include "spline.h"

using namespace std;

double calc_s (vector<double>& x, vector<double>& y, int x_index, int y_index) {
	return fabs(x[x_index] - x[0]) + fabs(y[y_index] - y[0]);
}

int main() {
	vector<double> x_vec {0.0, 1.0, 2.0, 3.0, 3.0, 4.0, 5.0};
	vector<double> y_vec {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
	vector<double> new_x = x_vec;
	vector<double> new_y = y_vec;
	for(int i=0; i<x_vec.size(); i++) {
		if (x_vec[i] == x_vec[i+1]) {
			new_x[i+1] = new_x[i] + 0.00001;
		}
		if (y_vec[i] == y_vec[i+1]) {
			new_y[i+1] = new_y[i] + 0.00001;
		}
	}
	double total_dis = calc_s(x_vec, y_vec, x_vec.size()-1, y_vec.size()-1);

 	vector<double> s;
 	for (int i=0; i<x_vec.size(); i++) {
 		int cur_s = calc_s(x_vec, y_vec, i, i);
 		s.push_back(cur_s);
 	}

	for (auto n : new_y) {
		cout << "x: " << n << endl;
	}
	tk::spline sx;
	sx.set_points(s, new_x);
	tk::spline sy;
	sy.set_points(s, new_y);
	for (double i=0; i<=total_dis; i+=0.5) {
		cout << "\t" << i << "\t" << sx(i) << "\t" << sy(i) << endl;
	}

	return 0;
}