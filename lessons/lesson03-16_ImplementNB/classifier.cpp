#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {
	
}

GNB::~GNB() {}

double gaussian_prob(double obs, double mu, double sig) {
    double num = pow((obs - mu), 2.0);
    double denum = 2 * pow(sig, 2);
    double norm = 1 / sqrt(2 * 3.1416 * pow(sig, 2));
    return norm * exp(-num / denum);
}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{
	/*	
		INPUTS
		data - array of N obs: [3.5, 0.1, 5.9, -0.02], [8.0, -0.3, 3.0, 2.2],...
		labels - "left", "keep", or "right".  
	*/
	
    double mean_left[4], mean_right[4], mean_keep[4];
    double var_left[4], var_right[4], var_keep[4];
    double sum_left[4], sum_right[4], sum_keep[4];	
    for (int i = 0; i < 4; i++) {
        mean_left[i] = 0.0;
        mean_right[i] = 0.0;
        mean_keep[i] = 0.0;
        var_left[i] = 0.0;
        var_right[i] = 0.0;
        var_keep[i] = 0.0;
        sum_left[i] = 0.0;
        sum_right[i] = 0.0;
        sum_keep[i] = 0.0;
    }	
	
	int n_left = 0, n_keep = 0, n_right = 0;
	// for each label: get sum (of each state variable) and count
	for (int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[0].size(); j++){
			if (labels[i] == "left"){
				sum_left[j] += data[i][j];
				n_left++;
			}
			if (labels[i] == "keep"){
				sum_keep[j] += data[i][j];
				n_keep++;
			}				
			if (labels[i] == "right"){
				sum_right[j] += data[i][j];
				n_right++;
			}
		}
	}
	
	n_left = n_left / 4; 
	n_keep = n_keep/ 4; 
	n_right = n_right / 4;
	
	for (int i = 0; i < data[0].size(); i++){
		mean_left[i] = sum_left[i] / n_left;
		mean_keep[i] = sum_keep[i] / n_keep;
		mean_right[i] = sum_right[i] / n_right;		
	}

	for (int i = 0; i < data.size(); i++){
		for (int j = 0; j < data[0].size(); j++){
			if (labels[i] == "left"){
				var_left[j] += pow(data[i][j] - mean_left[j], 2);
			}
			if (labels[i] == "keep"){
				var_keep[j] += pow(data[i][j] - mean_keep[j], 2);
			}				
			if (labels[i] == "right"){
				var_right[j] += pow(data[i][j] - mean_right[j], 2);
			}			
		}
	}

	for (int j = 0; j < data[0].size(); j++){
		var_left[j] = sqrt(var_left[j] / n_left);
		var_keep[j] = sqrt(var_keep[j] / n_keep);
		var_right[j] = sqrt(var_right[j] / n_right);
	}
	

    for (int j = 0; j < data[0].size(); j++) {
        means[0][j] = mean_left[j];
        means[1][j] = mean_keep[j];
        means[2][j] = mean_right[j];
        vars[0][j] = var_left[j];
        vars[1][j] = var_keep[j];
        vars[2][j] = var_right[j];
    }	
	
	cout.precision(4);
	cout << "Left  means=" << means[0][0] << "," << means[0][1] << "," << means[0][2] << "," << means[0][3] << "," << endl;
	cout << "Keep  means=" << means[1][0] << "," << means[1][1] << "," << means[1][2] << "," << means[1][3] << "," << endl;
	cout << "Right means=" << means[2][0] << "," << means[2][1] << "," << means[2][2] << "," << means[2][3] << "," << endl;	
	cout << "Left  vars=" << vars[0][0] << "," << vars[0][1] << "," << vars[0][2] << "," << vars[0][3] << "," << endl;
	cout << "Keep  vars=" << vars[1][0] << "," << vars[1][1] << "," << vars[1][2] << "," << vars[1][3] << "," << endl;
	cout << "Right vars=" << vars[2][0] << "," << vars[2][1] << "," << vars[2][2] << "," << vars[2][3] << "," << endl;		
}

string GNB::predict(vector<double> sample)
{
	/*
	INPUTS observation - a 4 tuple with s, d, s_dot, d_dot. Ex [3.5, 0.1, 8.5, -0.2]
	OUTPUT A label representing the best guess: "left", "keep" or "right".
	*/
	double p_s, p_d, p_sdot, p_ddot;
	double p[3] = {1.0, 1.0, 1.0};
	// for left, keep, right:
	for (int i = 0; i < 3; i++){
		p_s    = gaussian_prob(sample[0], means[i][0], vars[i][0]);
		p_d    = gaussian_prob(sample[1], means[i][1], vars[i][1]);		
		p_sdot = gaussian_prob(sample[2], means[i][2], vars[i][2]);		
		p_ddot = gaussian_prob(sample[3], means[i][3], vars[i][3]);	
		p[i] = p_s * p_d * p_sdot * p_ddot;
	}
	int pmax_id = 0;
	if (p[0] < p[1] and p[1] > p[2]) pmax_id = 1;
	if (p[0] < p[2] and p[2] > p[1]) pmax_id = 2;	
	cout << "sample=" << sample[0] << "," << sample[1] << "," <<sample[2] << "," <<sample[3] << " PROBS=" << p[0] << "," << p[1] << "," << p[2] << " pmax_id=" << pmax_id << endl;
	
	return this->possible_labels[pmax_id];

}