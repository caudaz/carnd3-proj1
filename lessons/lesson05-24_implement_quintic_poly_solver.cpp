#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// function JerkMinimizingTrajectory
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /* INPUTS
    start - the vehicles start location [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Also length three array.
    T     - The duration, in seconds, over which this maneuver should occur.
    
    OUTPUT 
    an array of length 6 for polynomial coeffs:
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    
    EXAMPLE > JMT( [0, 10, 0], [10, 10, 0], 1) = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]   */
    
    MatrixXd A(3, 3);   
    A <<   pow(T,3),    pow(T,4),    pow(T,5),
         3*pow(T,2),  4*pow(T,3),  5*pow(T,4),
         6*pow(T,1), 12*pow(T,2), 20*pow(T,3);

    VectorXd b(3); // or MatrixXd b(3,1)
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T,2)),
         end[1] - (start[1] + start[2] * T),
         end[2]-   start[2];
   //NOTE: 1/2 will not work on: b << end[0] - (start[0] + start[1]*T + 1/2*start[2]*pow(T,2)),

    // A * x = b (solve for a3, a4, a5)    
    Eigen::MatrixXd Ai = A.inverse();
    Eigen::MatrixXd x = Ai * b;

    return {start[0], start[1], .5 * start[2], x.data()[0], x.data()[1], x.data()[2]}; 
}

bool close_enough(vector< double > poly, vector<double> target_poly, double eps=0.01) {


    if(poly.size() != target_poly.size())
    {
        cout << "your solution didn't have the correct number of terms" << endl;
        return false;
    }
    for(int i = 0; i < poly.size(); i++)
    {
        double diff = poly[i]-target_poly[i];
        if(abs(diff) > eps)
        {
            cout << "at least one of your terms differed from target by more than " << eps << endl;
            return false;
        }

    }
    return true;
}
    
struct test_case {
    
        vector<double> start;
        vector<double> end;
        double T;
};

vector< vector<double> > answers = {{0.0, 10.0, 0.0, 0.0, 0.0, 0.0},
                                    {0.0,10.0,0.0,0.0,-0.625,0.3125},
                                    {5.0,10.0,1.0,-3.0,0.64,-0.0432}};

int main() {

    //create test cases

    vector< test_case > tc;

    test_case tc1;
    tc1.start = {0,10,0};
    tc1.end = {10,10,0};
    tc1.T = 1;
    tc.push_back(tc1);

    test_case tc2;
    tc2.start = {0,10,0};
    tc2.end = {20,15,20};
    tc2.T = 2;
    tc.push_back(tc2);

    test_case tc3;
    tc3.start = {5,10,2};
    tc3.end = {-30,-20,-4};
    tc3.T = 5;
    tc.push_back(tc3);

    bool total_correct = true;
    for(int i = 0; i < tc.size(); i++)
    {
        vector< double > jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
        bool correct = close_enough(jmt,answers[i]);
        total_correct &= correct;

    }
    if(!total_correct)
    {
        cout << "Try again!" << endl;
    }
    else
    {
        cout << "Nice work!" << endl;
    }

    return 0;
}