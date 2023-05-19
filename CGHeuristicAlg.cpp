#include <iostream>
#include <iterator>
#include <cmath>
#include <vector>
#include <list>
#include <tuple>
#include <map>
#include <fstream>
#include <string>
#include <sstream>
#include <gurobi_c++.h>

#include "Tabusearch.h"
#include "Geneticalgorithm.h"
#include "BranchAndBound.h"
#include "OperatorTheory.h"
#include "HungarianAlg.h"
#include "CGUtilities.h"

const int DEPOT_NODE = 0;
const int CAPACITY_LIMIT = 10;   //10 for truck, 3 for uav
const double FLYING_RANGE = 15000.0;
const double TRAVELING_RANGE = 75000.0;


int main() {
    //set of customer nodes
    std::set<int> C;
    for (int i = 0; i < 45; i++) {
        C.insert(i);
    }
    //set of truck customers
    std::set<int> Ct;
    for (int i = 1; i < 25; i++) {
        Ct.insert(i);
    }
    //set of drone customers
    std::set<int> Cd;
    for (int i = 25; i < 45; i++) {
        Cd.insert(i);
    }
    //set of rendezvous nodes
    std::set<int> S;
    for (int i = 13; i < 25; i++) {
        S.insert(i);
    }
    //number of truck routes
    int K1 = 3;
    //number of drone routes
    int K2 = 2;     
    //max route length
    double maxPathLength = TRAVELING_RANGE;
    std::fstream myFile;
    //open raod distance data file
    myFile.open("GCDistanceMatrix.txt", std::ios::in);//read
    std::vector<std::vector<double>> roadDistance;
    if (myFile.is_open()) {
        std::string line, word;
        std::istringstream iss;
        int rowNum = 0;
        while (std::getline(myFile, line)) {
            if (rowNum > 0) {
                std::vector<double> dist;
                iss.clear();
                iss.str(line);
                int colNum = 0;
                while (iss.good()) {
                    iss >> word;
                    //&& colNum <= NUMBER_OF_NODES
                    if (colNum > 0) {
                        double abc = std::stod(word);//abc == road distance
                        //std::cout << abc << std::endl;
                        dist.push_back(abc);
                    }
                    colNum += 1;
                }
                roadDistance.push_back(dist);
            }
            rowNum += 1;
        }
        myFile.close();
    }
    //custom node vector
    std::vector<int> nodes;
    nodes.push_back(0);   //0  //Depot node
    nodes.push_back(15); //cus5   //1   //Tcus1
    nodes.push_back(53); //cus43  //2   //Tcus2 
    nodes.push_back(20); //cus10  //3   //Tcus3
    nodes.push_back(56); //cus46  //4   //Tcus4 
    nodes.push_back(34); //cus24  //5   //Tcus5
    nodes.push_back(33); //cus23  //6   //Tcus6
    nodes.push_back(54); //cus44  //7   //Tcus7
    nodes.push_back(30); //cus20  //8   //Tcus8
    nodes.push_back(59); //cus49  //9  //Tcus9
    nodes.push_back(14); //cus4   //10  //Tcus10 
    nodes.push_back(49); //cus39  //11 // truck customers  //Tcus11
    nodes.push_back(27); //cus17  //12  //Tcus12
    nodes.push_back(50); //cus40  //13  //Tcus13
    nodes.push_back(37); //cus27  //14  //Tcus14
    nodes.push_back(32); //cus22  //15  //Tcus15
    nodes.push_back(48); //cus38  //16  //Tcus16
    nodes.push_back(57); //cus47  //17  //Tcus17
    nodes.push_back(13); //cus3   //18  //Tcus18
    nodes.push_back(41); //cus31  //19  //Tcus19
    nodes.push_back(44); //cus34  //20  //Tcus20 
    nodes.push_back(43); //cus33  //21  //Tcus21
    nodes.push_back(40); //cus30  //22  //Tcus22
    nodes.push_back(12); //cus2   //23  //Tcus23
    nodes.push_back(16); //cus6   //24 //sat cust  //Tcus24 
    nodes.push_back(23); //cus13  //25  //Dcus25
    nodes.push_back(45); //cus35  //26  //Dcus26
    nodes.push_back(24); //cus14  //27  //Dcus27
    nodes.push_back(35); //cus25  //28  //Dcus28
    nodes.push_back(28); //cus18  //29  //Dcus29
    nodes.push_back(11); //cus1   //30  //Dcus30
    nodes.push_back(21); //cus11  //31  //Dcus31
    nodes.push_back(60); //cus50  //32  //Dcus32
    nodes.push_back(38); //cus28  //33  //Dcus33
    nodes.push_back(58); //cus48  //34  //Dcus34
    nodes.push_back(26); //cus16  //35  //Dcus35
    nodes.push_back(17); //cus7   //36  //Dcus36
    nodes.push_back(52); //cus42  //37  //Dcus37
    nodes.push_back(51); //cus41  //38  //Dcus38
    nodes.push_back(29); //cus19  //39  //Dcus39
    nodes.push_back(47); //cus37  //40  //Dcus40
    nodes.push_back(22); //cus12  //41  //Dcus41
    nodes.push_back(25); //cus15  //42  //Dcus42
    nodes.push_back(42); //cus32  //43  //Dcus43
    nodes.push_back(39); //cus29  //44 //drone cust //Dcus44
    //distance matrix for the new problem.
    std::vector < std::vector<double>> distanceMatrix;
    std::vector<double> dis;
    for (auto it : nodes) {
        for (auto itt : nodes) {
            double len = roadDistance[it][itt];
            dis.push_back(len);
        }
        distanceMatrix.push_back(dis);
        dis.clear();
    }
  
    //call cg utilities class
    CGUtilities cgu(K1, K2, C, Ct, Cd, S, distanceMatrix);
    cgu.runCGBasedHeuristic();
       
    return 0;
}

