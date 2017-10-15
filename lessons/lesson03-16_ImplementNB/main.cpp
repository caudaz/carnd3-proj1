#include "classifier.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

// function to load CSV text file - into 2D vector
vector<vector<double> > Load_State(string file_name)
{
    // Input File Stream (ifstream) - reads file into "in_state"
    ifstream in_state_(file_name.c_str(), ifstream::in);
    // define 2D vector
    vector< vector<double >> state_out;
    // define line
    string line;
 
    // read file "in_state" line by line
    while (getline(in_state_, line)) 
    {
        // returns a string object with a copy of the current contents of the stream
        istringstream iss(line);
        // empty vector for state - 4 values
        vector<double> x_coord;
        // emtpy string
        string token;
        // token = values in iss istream, delimited by ','
        while( getline(iss,token,','))
        {
            // get 4 values one at a time into 1D vector
            x_coord.push_back(stod(token));
        }
        // put 1D vector into 2D vector
        state_out.push_back(x_coord);
    }
    return state_out;
}

// function to load CSV text file - into STRING vector
vector<string> Load_Label(string file_name)
{
    ifstream in_label_(file_name.c_str(), ifstream::in);
    vector< string > label_out;
    string line;
    while (getline(in_label_, line)) 
    {
        istringstream iss(line);
        string label;
        iss >> label;
    
        label_out.push_back(label);
    }
    return label_out;
    
}

int main() {
    
    vector< vector<double> > X_train = Load_State("./train_states.txt");
    vector< vector<double> > X_test  = Load_State("./test_states.txt");
    vector< string > Y_train  = Load_Label("./train_labels.txt");
    vector< string > Y_test   = Load_Label("./test_labels.txt");
    
    cout << "X_train size= " << X_train.size() << " X " << X_train[0].size() << endl;
    cout << "Y_train size= " << Y_train.size() << " X " << Y_train[0].size() << endl;

    cout << "X_test size= " << X_test.size() << endl;
    cout << "Y_test size= " << Y_test.size() << endl;

    // create Gaussian Naive Bayes classifier and train it
    GNB gnb = GNB();
    gnb.train(X_train, Y_train);

    // compute accuracy of GNB classifier using X_test and Y_test
    int score = 0;
    for(int i = 0; i < X_test.size(); i++)
    {
        // state (S,d,Sdot,ddot)
        vector<double> coords = X_test[i];
        // predict label
        string predicted = gnb.predict(coords);
		cout << "pred=" << predicted << " label=" << Y_test[i] << endl;
        // compare to label
        if(predicted.compare(Y_test[i]) == 0)
        {
            score += 1;
        }
    }
    // accuracy
    float fraction_correct = float(score) / Y_test.size();
    cout << "You got " << (100*fraction_correct) << " correct" << endl;
    
    return 0;
}

