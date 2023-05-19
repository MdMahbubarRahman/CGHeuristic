#include <iostream>
#include <iterator>
#include <cmath>
#include <vector>
#include <list>
#include <set>
#include <tuple>
#include <map>
#include <fstream>
#include <string>
#include <sstream>
#include <gurobi_c++.h>
#include <algorithm>

#include "Geneticalgorithm.h"

#ifndef CGUTILITIES_H
#define CGUTILITIES_H

//data structure for the truck paths
class TruckPaths {
private:
    int noTruckPaths;
    std::list<std::vector<int>> vecTruckPaths;
public:
    TruckPaths();
    TruckPaths(const TruckPaths & tp);
    TruckPaths(int a, std::list<std::vector<int>> b);
    int getNoTruckPaths();
    std::list<std::vector<int>> getVecTruckPaths();
};

//data structure for the drone paths
class DronePaths {
private:
    int noDronePaths;
    std::list<std::vector<int>> vecDronePaths;
public:
    DronePaths();
    DronePaths(const DronePaths & dp);
    DronePaths(int a, std::list<std::vector<int>> b);
    int getNoDronePaths();
    std::list<std::vector<int>> getVecDronePaths();
};

//data structure for the parameters of the restricted master problem
class RMPParameters {
private:
    TruckPaths truckPaths;
    DronePaths dronePaths;
    std::map<std::tuple<int, int, int>, double> arcsCBDronePaths;        //e_d^ij
    std::map<std::tuple<int, int, int>, double> arcsCBTruckPaths;        //h_t^ij
    std::map<std::tuple<int, int>, double> nodesCBDronePaths;            //e_d^i
    std::map<std::tuple<int, int>, double> nodesCBTruckPaths;            //h_t^i
    std::map<std::tuple<int, int>, double> dronePathsSFRNodes;           //g_d^s
public:
    RMPParameters();
    RMPParameters(const RMPParameters & rmpparm);
    RMPParameters(TruckPaths tp, DronePaths dp, std::map<std::tuple<int, int, int>, double> acdp, std::map<std::tuple<int, int, int>, double> actp, std::map<std::tuple<int, int>, double> ncdp, std::map<std::tuple<int, int>, double> nctp, std::map<std::tuple<int, int>, double> nsr);
    TruckPaths getTruckPaths();
    DronePaths getDronePaths();
    std::map<std::tuple<int, int, int>, double> getArcsCBDronePaths();        
    std::map<std::tuple<int, int, int>, double> getArcsCBTruckPaths();        
    std::map<std::tuple<int, int>, double> getNodesCBDronePaths();            
    std::map<std::tuple<int, int>, double> getNodesCBTruckPaths();            
    std::map<std::tuple<int, int>, double> getDronePathsSFRNodes();   
    void setTruckPaths(TruckPaths tpaths);
    void setDronePaths(DronePaths dpaths);
    void setArcsCBDronePaths(std::map<std::tuple<int, int, int>, double> adpaths);
    void setArcsCBTruckPaths(std::map<std::tuple<int, int, int>, double> atpaths);
    void setNodesCBDronePaths(std::map<std::tuple<int, int>, double> ndpaths);
    void setNodesCBTruckPaths(std::map<std::tuple<int, int>, double> ntpaths);
    void setDronePathsSFRNodes(std::map<std::tuple<int, int>, double> dprn);
    void showPRMParameters();
};

//data structure for column generation 
class CGData {
private:
    double alpha;
    std::map<std::pair<int, int>, double> betaDict;
    std::map<int, double> gammaDict;
    std::map<int, double> lambdaDict;
    std::map<std::pair<int, int>, double> sigmaDict;
    std::map<int, double> rhoDict;
    std::map<int, double> piDict;
    std::map<int, double> diDict;
    std::map<std::pair<int, int>, double> taDict;
    std::map<std::pair<int, int>, double> daDict;
    double lbd;
    double ubd;
    double objVal;
    double objGap;
    int iter;
public:
    CGData();
    CGData(const CGData& cgdata);
    CGData(double alpha, std::map<std::pair<int, int>, double> betaDict, std::map<int, double> gammaDict, std::map<int, double> lambdaDict, std::map<std::pair<int, int>, double> sigmaDict, std::map<int, double> rhoDict, std::map<int, double> piDict, std::map<int, double> diDict, std::map<std::pair<int, int>, double> taDict, std::map<std::pair<int, int>, double> daDict, double lbd, double ubd, double objVal, double objGap, int iter);
    double getAlpha();
    std::map<std::pair<int, int>, double> getBetaDict();
    std::map<int, double> getGammaDict();
    std::map<int, double> getLambdaDict();
    std::map<std::pair<int, int>, double> getSigmaDict();
    std::map<int, double> getRhoDict();
    std::map<int, double> getPiDict();
    std::map<int, double> getDiDict();
    std::map<std::pair<int, int>, double> getTaDict();
    std::map<std::pair<int, int>, double> getDaDict();
    double getLbd();
    double getUbd();
    double getObjVal();
    double getObjGap();
    int getIter();
    void setAlpha(double alpha);
    void setBetaDict(std::map<std::pair<int, int>, double> betaDict);
    void setGammaDict(std::map<int, double> gammaDict);
    void setLambdaDict(std::map<int, double> lambdaDict);
    void setSigmaDict(std::map<std::pair<int, int>, double> sigmaDict);
    void setRhoDict(std::map<int, double> rhoDict);
    void setPiDict(std::map<int, double> piDict);
    void setDiDict(std::map<int, double> diDict);
    void setTaDict(std::map<std::pair<int, int>, double> taDict);
    void setDaDict(std::map<std::pair<int, int>, double> daDict);
    void setLbd(double lbd);
    void setUbd(double ubd);
    void setObjVal(double objVal);
    void setObjGap(double objGap);
    void setIter(int iter);
    void showCGData();
};
   
//data structure for the Gurobi model
/*
struct ModelData {
    GRBEnv env;
    GRBModel model;
    GRBVar pi_var;
    GRBVar di_var;
    GRBVar ta_var;
    GRBVar da_var;
};
*/
//cg utilities
class CGUtilities{
private:
    int K1;
    int K2;
    std::set<int> C;
    std::set<int> Ct;
    std::set<int> Cd;
    std::set<int> S;
    std::list<std::pair<int, int>> truckArcs;
    std::list<std::pair<int, int>> droneArcs;
    std::map<std::pair<int, int>, double> truckArcCosts;
    std::map<std::pair<int, int>, double> droneArcCosts;
    std::list<std::vector<int>> listOfTruckPaths;
    std::list<std::vector<int>> listOfDronePaths;
    std::list<std::vector<int>> promTPaths;
    std::list<std::vector<int>> promDPaths;
    std::vector < std::vector<double>> distanceMatrix;
    RMPParameters rmpp;
    CGData cgd;
    //GRBModel details;
public:
    CGUtilities(); //default constructor
    CGUtilities(const CGUtilities & cgu); //copy constructor
    CGUtilities(int k1, int k2, std::set<int> C, std::set<int> Ct, std::set<int> Cd, std::set<int> S, std::vector < std::vector<double>> distanceMatrix);
    void generateInitialTruckAndDronePaths();
    void populateInitialCGData();
    void populateInitialRMPParameters();
    void buildAndSolveRMProblem();
    void findPromisingTruckPaths();
    void findPromisingDronePaths();
    void updateRMPParameters();
    bool isSolutionInteger();
    void getIntegerRMPSolution();
    void printCGSolution();
    void runCGBasedHeuristic();
    int getK1();
    int getK2();
    std::set<int> getC();
    std::set<int> getCt();
    std::set<int> getCd();
    std::set<int> getS();
    std::list<std::pair<int, int>> getTruckArcs();
    std::list<std::pair<int, int>> getDroneArcs();
    std::map<std::pair<int, int>, double> getTruckArcCosts();
    std::map<std::pair<int, int>, double> getDroneArcCosts();
    std::list<std::vector<int>> getListOfTruckPaths();
    std::list<std::vector<int>> getListOfDronePaths();
    std::list<std::vector<int>> getPromTPaths();
    std::list<std::vector<int>> getPromDPaths();
    std::vector < std::vector<double>> getDistanceMatrix();
    RMPParameters getRMPP();
    CGData getCGD();
};

#endif //CGUTILITIES_H

