#include "CGUtilities.h"

//default constructor
TruckPaths::TruckPaths(){ 

}

//copy constructor
TruckPaths::TruckPaths(const TruckPaths& tp) {
    noTruckPaths = tp.noTruckPaths;
    vecTruckPaths = tp.vecTruckPaths;
}

//constructor
TruckPaths::TruckPaths(int a, std::list<std::vector<int>> b) {
    noTruckPaths = a;
    vecTruckPaths = b;
}

int TruckPaths::getNoTruckPaths() {
    return noTruckPaths;
}

std::list<std::vector<int>> TruckPaths::getVecTruckPaths() {
    return vecTruckPaths;
}

//default constructor
DronePaths::DronePaths() {

}

//copy constructor
DronePaths::DronePaths(const DronePaths& dp) {
    noDronePaths = dp.noDronePaths;
    vecDronePaths = dp.vecDronePaths;
}

//constructor
DronePaths::DronePaths(int a, std::list<std::vector<int>> b) {
    noDronePaths = a;
    vecDronePaths = b;
}

int DronePaths::getNoDronePaths() {
    return noDronePaths;
}

std::list<std::vector<int>> DronePaths::getVecDronePaths() {
    return vecDronePaths;
}

//default constructor
RMPParameters::RMPParameters() {

}

//copy constructor
RMPParameters::RMPParameters(const RMPParameters& rmpparm) {
    truckPaths             = rmpparm.truckPaths;
    dronePaths             = rmpparm.dronePaths;
    arcsCBDronePaths       = rmpparm.arcsCBDronePaths;        //e_d^ij
    arcsCBTruckPaths       = rmpparm.arcsCBTruckPaths;        //h_t^ij
    nodesCBDronePaths      = rmpparm.nodesCBDronePaths   ;            //e_d^i
    nodesCBTruckPaths      = rmpparm.nodesCBTruckPaths;            //h_t^i
    dronePathsSFRNodes     = rmpparm.dronePathsSFRNodes;
}

//constructor
RMPParameters::RMPParameters(TruckPaths tp, DronePaths dp, std::map<std::tuple<int, int, int>, double> acdp, std::map<std::tuple<int, int, int>, double> actp, std::map<std::tuple<int, int>, double> ncdp, std::map<std::tuple<int, int>, double> nctp, std::map<std::tuple<int, int>, double> nsr){
    truckPaths = tp;
    dronePaths = dp;
    arcsCBDronePaths = acdp;        //e_d^ij
    arcsCBTruckPaths = actp;        //h_t^ij
    nodesCBDronePaths = ncdp;            //e_d^i
    nodesCBTruckPaths = nctp;            //h_t^i
    dronePathsSFRNodes = nsr;
}

TruckPaths RMPParameters::getTruckPaths() {
    return truckPaths;
}

DronePaths RMPParameters::getDronePaths() {
    return dronePaths;
}

std::map<std::tuple<int, int, int>, double> RMPParameters::getArcsCBDronePaths() {
    return arcsCBDronePaths;
}

std::map<std::tuple<int, int, int>, double> RMPParameters::getArcsCBTruckPaths() {
    return arcsCBTruckPaths;
}

std::map<std::tuple<int, int>, double> RMPParameters::getNodesCBDronePaths() {
    return nodesCBDronePaths;
}

std::map<std::tuple<int, int>, double> RMPParameters::getNodesCBTruckPaths() {
    return nodesCBTruckPaths;
}

std::map<std::tuple<int, int>, double> RMPParameters::getDronePathsSFRNodes() {
    return dronePathsSFRNodes;
}

void RMPParameters::setTruckPaths(TruckPaths tpaths) {
    truckPaths = tpaths;
}

void RMPParameters::setDronePaths(DronePaths dpaths) {
    dronePaths = dpaths;
}

void RMPParameters::setArcsCBDronePaths(std::map<std::tuple<int, int, int>, double> adpaths) {
    arcsCBDronePaths = adpaths;
}

void RMPParameters::setArcsCBTruckPaths(std::map<std::tuple<int, int, int>, double> atpaths) {
    arcsCBTruckPaths = atpaths;
}

void RMPParameters::setNodesCBDronePaths(std::map<std::tuple<int, int>, double> ndpaths) {
    nodesCBDronePaths = ndpaths;
}

void RMPParameters::setNodesCBTruckPaths(std::map<std::tuple<int, int>, double> ntpaths) {
    nodesCBTruckPaths = ntpaths;
}

void RMPParameters::setDronePathsSFRNodes(std::map<std::tuple<int, int>, double> dprn) {
    dronePathsSFRNodes = dprn;
}

void RMPParameters::showPRMParameters() {
    std::cout << "No of truck paths : " << truckPaths.getNoTruckPaths() << std::endl;
    std::cout << "No of drone paths : " << dronePaths.getNoDronePaths() << std::endl;
    std::cout << "No of elements in the arcsCBDronePaths map : " << arcsCBDronePaths.size() << std::endl;
    std::cout << "No of elements in the arcsCBTruckPaths map : " << arcsCBTruckPaths.size() << std::endl;
    std::cout << "No of elements in the nodesCBDronePaths map : " << nodesCBDronePaths.size() << std::endl;
    std::cout << "No of elements in the nodesCBTruckPaths map : " << nodesCBTruckPaths.size() << std::endl;
    std::cout << "No of elements in the dronePathsSFRNodes map : " << dronePathsSFRNodes.size() << std::endl;
}

//default constructor
CGData::CGData() {

}

//copy constructor
CGData::CGData(const CGData& cd) {
    alpha         = cd.alpha;
    /*
    beta          = cd.beta;
    gamma         = cd.gamma;
    lambda        = cd.lambda;
    sigma         = cd.sigma;
    rho           = cd.rho;
    */
    betaDict      = cd.betaDict;
    gammaDict     = cd.gammaDict;
    lambdaDict    = cd.lambdaDict;
    sigmaDict     = cd.sigmaDict;
    rhoDict       = cd.rhoDict;
    piDict        = cd.piDict;
    diDict        = cd.diDict;
    taDict        = cd.taDict;
    daDict        = cd.daDict;
    lbd           = cd.lbd;
    ubd           = cd.ubd;
    objVal        = cd.objVal;
    objGap        = cd.objGap;
    iter          = cd.iter;
}

//constructor
CGData::CGData(double alp, std::map<std::pair<int, int>, double> beDict, std::map<int, double> gaDict, std::map<int, double> laDict, std::map<std::pair<int, int>, double> siDict, std::map<int, double> rhDict, std::map<int, double> pDict, std::map<int, double> dDict, std::map<std::pair<int, int>, double> tDict, std::map<std::pair<int, int>, double> ddDict, double ld, double ud, double objV, double objG, int itr) {
    alpha         = alp;
    /*
    beta          = bta;
    gamma         = gma;
    lambda        = lda;
    sigma         = sma;
    rho           = ro; 
    */
    betaDict      = beDict;
    gammaDict     = gaDict;
    lambdaDict    = laDict;
    sigmaDict     = siDict;
    rhoDict       = rhDict;
    piDict        = pDict;
    diDict        = dDict;
    taDict        = tDict;
    daDict        = ddDict;
    lbd           = ld;
    ubd           = ud;
    objVal        = objV;
    objGap        = objG;
    iter          = itr;
}

double CGData::getAlpha() {
    return alpha;
}

std::map<std::pair<int, int>, double> CGData::getBetaDict() {
    return betaDict;
}

std::map<int, double> CGData::getGammaDict() {
    return gammaDict;
}

std::map<int, double> CGData::getLambdaDict() {
    return lambdaDict;
}

std::map<std::pair<int, int>, double> CGData::getSigmaDict() {
    return sigmaDict;
}

std::map<int, double> CGData::getRhoDict() {
    return rhoDict;
}

/*
std::list<double> CGData::getAlpha() {
    return alpha;
}

std::list<double> CGData::getBeta() {
    return beta;
}

std::list<double> CGData::getGamma() {
    return gamma;
}

std::list<double> CGData::getLambda() {
    return lambda;
}

std::list<double> CGData::getSigma() {
    return sigma;
}

std::list<double> CGData::getRho() {
    return rho;
}

std::map<std::pair<int, int>, int> CGData::getBetaDict() {
    return betaDict;
}

std::map<int, int> CGData::getGammaDict() {
    return gammaDict;
}

std::map<int, int> CGData::getLambdaDict() {
    return lambdaDict;
}

std::map<std::pair<int, int>, int> CGData::getSigmaDict() {
    return sigmaDict;
}

std::map<int, int> CGData::getRhoDict() {
    return rhoDict;
}
*/
std::map<int, double> CGData::getPiDict() {
    return piDict;
}

std::map<int, double> CGData::getDiDict() {
    return diDict;
}

std::map<std::pair<int, int>, double> CGData::getTaDict() {
    return taDict;
}

std::map<std::pair<int, int>, double> CGData::getDaDict() {
    return daDict;
}

double CGData::getLbd() {
    return lbd;
}

double CGData::getUbd() {
    return ubd;
}

double CGData::getObjVal() {
    return objVal;
}

double CGData::getObjGap() {
    return objGap;
}

int CGData::getIter() {
    return iter;
}

//print the data contained in cgdata class
void CGData::showCGData() {
    std::cout << "Show the data of alpha : " << alpha <<  std::endl;
    std::cout << "Show the keys and values of the betaDict" << std::endl;
    for (auto &it: betaDict) {
        std::cout << "Key : " << it.first.first<<it.first.second << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the gammaDict" << std::endl;
    for (auto& it : gammaDict) {
        std::cout << "Key : " << it.first << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the lambdaDict" << std::endl;
    for (auto& it : lambdaDict) {
        std::cout << "Key : " << it.first << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the sigmaDict" << std::endl;
    for (auto& it : sigmaDict) {
        std::cout << "Key : " << it.first.first << it.first.second << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the rhoDict" << std::endl;
    for (auto& it : rhoDict) {
        std::cout << "Key : " << it.first << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the piDict" << std::endl;
    for (auto& it : piDict) {
        std::cout << "Key : " << it.first << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the diDict" << std::endl;
    for (auto& it : diDict) {
        std::cout << "Key : " << it.first << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the taDict" << std::endl;
    for (auto& it : taDict) {
        std::cout << "Key : " << it.first.first << it.first.second << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Show the keys and values of the daDict" << std::endl;
    for (auto& it : daDict) {
        std::cout << "Key : " << it.first.first << it.first.second << ", Value : " << it.second << " ";
    }
    std::cout << " " << std::endl;
    std::cout << "Lower bound, lbd : " << lbd << std::endl;
    std::cout << "Upper bound, ubd : " << ubd << std::endl;
    std::cout << "ObjVal : " << objVal << std::endl;
    std::cout << "ObjGap : " << objGap << std::endl;
    std::cout << "Interation no : " << iter << std::endl;    
}

void CGData::setAlpha(double al) {
    alpha = al;
}

void CGData::setBetaDict(std::map<std::pair<int, int>, double> beta) {
    betaDict = beta;
}

void CGData::setGammaDict(std::map<int, double> gamma) {
    gammaDict = gamma;
}

void CGData::setLambdaDict(std::map<int, double> lambda) {
    lambdaDict = lambda;
}

void CGData::setSigmaDict(std::map<std::pair<int, int>, double> sigma) {
    sigmaDict = sigma;
}

void CGData::setRhoDict(std::map<int, double> rho) {
    rhoDict = rho;
}

/*
void CGData::setAlpha(std::list<double> apa) {
    alpha = apa;
}

void CGData::setBeta(std::list<double> bt) {
    beta = bt;
}

void CGData::setGamma(std::list<double> gm) {
    gamma = gm;
}

void CGData::setLambda(std::list<double> lm) {
    lambda = lm;
}

void CGData::setSigma(std::list<double> sm) {
    sigma = sm;
}

void CGData::setRho(std::list<double> rh) {
    rho = rh;
}

void CGData::setBetaDict(std::map<std::pair<int, int>, int> btd) {
    betaDict = btd;
}

void CGData::setGammaDict(std::map<int, int> gmd) {
    gammaDict = gmd;
}

void CGData::setLambdaDict(std::map<int, int> lmd) {
    lambdaDict = lmd;
}

void CGData::setSigmaDict(std::map<std::pair<int, int>, int> smd) {
    sigmaDict = smd;
}

void CGData::setRhoDict(std::map<int, int> rhd) {
    rhoDict = rhd;
}
*/
void CGData::setPiDict(std::map<int, double> pid) {
    piDict = pid;
}

void CGData::setDiDict(std::map<int, double> did) {
    diDict = did;
}

void CGData::setTaDict(std::map<std::pair<int, int>, double> tad) {
    taDict = tad;
}

void CGData::setDaDict(std::map<std::pair<int, int>, double> dad) {
    daDict = dad;
}

void CGData::setLbd(double ld) {
    lbd = ld;
}

void CGData::setUbd(double ud) {
    ubd = ud;
}

void CGData::setObjVal(double obv) {
    objVal = obv;
}

void CGData::setObjGap(double objg) {
    objGap = objg;
}

void CGData::setIter(int it) {
    iter = it;
}

//populate default constructor
CGUtilities::CGUtilities() {

}

//populate copy constructor
CGUtilities::CGUtilities(const CGUtilities& cgu) {
    K1                 = cgu.K1;
    K2                 = cgu.K2;
    C                  = cgu.C;
    Ct                 = cgu.Ct;
    Cd                 = cgu.Cd;
    S                  = cgu.S;
    truckArcs          = cgu.truckArcs;
    droneArcs          = cgu.droneArcs;
    truckArcCosts      = cgu.truckArcCosts;
    droneArcCosts      = cgu.droneArcCosts;
    promTPaths         = cgu.promTPaths;
    promDPaths         = cgu.promDPaths;
    rmpp               = cgu.rmpp;
    cgd                = cgu.cgd;
    distanceMatrix     = cgu.distanceMatrix;
}

//populate constructor
CGUtilities::CGUtilities(int k1, int k2, std::set<int> c, std::set<int> ct, std::set<int> cd, std::set<int> s, std::vector < std::vector<double>> disMatrix) {
    K1 = k1;
    K2 = k2;
    C = c;
    Ct = ct;
    Cd = cd;
    S = s;
    distanceMatrix = disMatrix;
    rmpp = RMPParameters();
    cgd = CGData();   
    //generate truckArcs
    for (int i = 0; i < 25; i++) {
        for (int j = 0; j < 25; j++) {
            if (i!=j) {
                truckArcs.push_back(std::make_pair(i,j));
            }
        }
    }
    //generate droneArcs
    for (int i = 13; i < 25; i++) {
        for (int j = 25; j < 45; j++) {
            droneArcs.push_back(std::make_pair(i,j));
        }
    }
    for (int i = 25; i < 45; i++) {
        for (int j = 13; j < 45; j++) {
            if (i!=j) {
                droneArcs.push_back(std::make_pair(i,j));
            }
        }
    }
    //generate truckArcs distances
    for (int i = 0; i < 25; i++) {
        for (int j = 0; j < 25; j++) {
            if (i != j) {
                truckArcCosts[std::make_pair(i, j)] = distanceMatrix[i][j];
            }
        }
    }   
    //generate drone arc distances
    for (int i = 13; i < 25; i++) {
        for (int j = 25; j < 45; j++) {
            droneArcCosts[std::make_pair(i, j)] = distanceMatrix[i][j];
        }
    }
    for (int i = 25; i < 45; i++) {
        for (int j = 13; j < 45; j++) {
            if (i != j) {
                droneArcCosts[std::make_pair(i, j)] = distanceMatrix[i][j];
            }
        }
    }
}

int CGUtilities::getK1() {
    return K1;
}

int CGUtilities::getK2() {
    return K2;
}

std::set<int> CGUtilities::getC() {
    return C;
}

std::set<int> CGUtilities::getCt() {
    return Ct;
}

std::set<int> CGUtilities::getCd() {
    return Cd;
}

std::set<int> CGUtilities::getS() {
    return S;
}

std::list<std::pair<int, int>> CGUtilities::getTruckArcs() {
    return truckArcs;
}

std::list<std::pair<int, int>> CGUtilities::getDroneArcs() {
    return droneArcs;
}

std::map<std::pair<int, int>, double> CGUtilities::getTruckArcCosts() {
    return truckArcCosts;
}

std::map<std::pair<int, int>, double> CGUtilities::getDroneArcCosts() {
    return droneArcCosts;
}

std::list<std::vector<int>> CGUtilities::getListOfTruckPaths() {
    return listOfTruckPaths;
}

std::list<std::vector<int>> CGUtilities::getListOfDronePaths() {
    return listOfDronePaths;
}

std::list<std::vector<int>> CGUtilities::getPromTPaths() {
    return promTPaths;
}

std::list<std::vector<int>> CGUtilities::getPromDPaths() {
    return promDPaths;
}

std::vector < std::vector<double>> CGUtilities::getDistanceMatrix() {
    return distanceMatrix;
}

RMPParameters CGUtilities::getRMPP() {
    return rmpp;
}

CGData CGUtilities::getCGD() {
    return cgd;
}

//generate initial truck and drone paths
void CGUtilities::generateInitialTruckAndDronePaths() {
    //generate truck paths
    std::list<std::vector<int>> initialTruckPaths;
    std::vector<int> cusCluster;
    //one node paths
    for (auto &c:Ct) {
        std::vector<int> oneTPath;
        oneTPath.push_back(0);
        oneTPath.push_back(c);
        oneTPath.push_back(0);
        initialTruckPaths.push_back(oneTPath);
        oneTPath.clear();
        cusCluster.push_back(c);
    }
    //two node paths
    for (int i = 1; i < 25-1; i++) {
        std::vector<int> twoTPath;
        twoTPath.push_back(0);
        twoTPath.push_back(i);
        twoTPath.push_back(i+1);
        twoTPath.push_back(0);
        initialTruckPaths.push_back(twoTPath);
        twoTPath.clear();
    }
    //multi node paths
    int capLimit = 10;
    double maxPathLength = 75000.0;
    //std::cout << "Uniform demand for each customer." << std::endl;
    std::map<int, int> cusToDemand;
    for (int i = 1; i < 25; i++) {
        cusToDemand.insert(std::pair<int, int>(i, 1));//unit demand
    }
    Geneticalgorithm genAlg;
    genAlg = Geneticalgorithm(0, capLimit, maxPathLength, cusToDemand, distanceMatrix, cusCluster);
    //std::cout << "\nGenetic algorithm started." << std::endl;
    genAlg.runGeneticAlgorithm();
    //genAlg.showGASolution();
    for (auto &it: genAlg.getLastPopulation()) {
        if(it.getFitness() <= genAlg.getGASolution().getFitness()*1.2) {
            std::vector<int> solVec = it.getChromosomeRepresentation();
            std::vector<int> mulvec;
            for (int i = 0; i < solVec.size(); i++) {
                if (mulvec.empty() && solVec[i] == 0) {
                    mulvec.push_back(0);
                }
                else if (!mulvec.empty() && (mulvec.back() != 0) && (solVec[i] == 0)) {
                    mulvec.push_back(0);
                    initialTruckPaths.push_back(mulvec);
                    mulvec.clear();
                    mulvec.push_back(0);
                }
                else if (solVec[i] != 0) {
                    mulvec.push_back(solVec[i]);
                }
            }
        }
    }
    //eliminate duplicates 
    for (auto it = initialTruckPaths.begin(); it != initialTruckPaths.end(); ++it) {
        for (auto itt = std::next(it); itt != initialTruckPaths.end();) {
            if (*it == *itt) {
                itt = initialTruckPaths.erase(itt);
            }
            else {
                ++itt;
            }
        }     
    }  
    //populate listOfTruckPaths
    listOfTruckPaths = initialTruckPaths;
    //generate drone paths from each rendezvous nodes
    std::list<std::vector<int>> initialDronePaths;
    for (auto &s: S) {
        std::list<std::vector<int>> initDPaths;
        std::vector<int> csCluster;
        //one node paths
        for (auto& c : Cd) {
            std::vector<int> oneDPath;
            if (distanceMatrix[s][c] <= 7500.0) {
                oneDPath.push_back(s);
                oneDPath.push_back(c);
                oneDPath.push_back(s);
                initDPaths.push_back(oneDPath);
                oneDPath.clear();
                csCluster.push_back(c);
            }
        }
        //multi node paths
        int cpLimit = 3;
        double maxPLength = 15000.0;
        //std::cout << "Uniform demand for each customer." << std::endl;
        std::map<int, int> cusTDemand;
        for (int i = 25; i < 45; i++) {
            cusTDemand.insert(std::pair<int, int>(i, 1));//unit demand
        }
        Geneticalgorithm gAlg;
        gAlg = Geneticalgorithm(s, cpLimit, maxPLength, cusTDemand, distanceMatrix, csCluster);
        //std::cout << "\nGenetic algorithm started for rendezvous node "+std::to_string(s) << std::endl;
        gAlg.runGeneticAlgorithm();
        //gAlg.showGASolution();
        std::vector<int> solVec = gAlg.getGASolution().getChromosomeRepresentation();
        std::vector<int> mulvec;
        for (int i = 0; i < solVec.size(); i++) {
            if (mulvec.empty() && solVec[i] == s) {
                mulvec.push_back(s);
            }
            else if (!mulvec.empty() && (mulvec.back() != s) && (solVec[i] == s)) {
                mulvec.push_back(s);
                initDPaths.push_back(mulvec);
                mulvec.clear();
                mulvec.push_back(s);
            }
            else if (solVec[i] != s) {
                mulvec.push_back(solVec[i]);
            }
        }
        //eliminate duplicates 
        for (auto it = initDPaths.begin(); it != initDPaths.end(); ++it) {
            for (auto itt = std::next(it); itt != initDPaths.end();) {
                if (*it == *itt) {
                    itt = initDPaths.erase(itt);
                }
                else {
                    ++itt;
                }
            }
        }
        //populate listOfDronePaths
        for (auto & it: initDPaths) {
            initialDronePaths.push_back(it);
        }
    }
    listOfDronePaths = initialDronePaths;
}

//initiate cgdata structure with initial values  
void CGUtilities::populateInitialCGData() {
    //populate beta dictionary using trcukArcs
    /*
    std::map<std::pair<int, int>, int> beDict;
    int i = 0;
    for (auto& it : truckArcs) {
        beDict[it] = i;
        i++;
    }
    //populate gamma dict
    i = 0;
    std::map<int, int> gaDict;
    for (auto& it : Ct) {
        gaDict[it] = i;
        i++;
    }
    //populate lambda dict
    i = 0;
    std::map<int, int> laDict;
    for (auto& it : S) {
        laDict[it] = i;
        i++;
    }
    //populate sigma dict
    i = 0;
    std::map<std::pair<int, int>, int> siDict;
    for (auto& it : droneArcs) {
        siDict[it] = i;
        i++;
    }
    //populate rho dict
    i = 0;
    std::map<int, int> rDict;
    for (auto& it : Cd) {
        rDict[it] = i;
        i++;
    }
    //populate cgd
    cgd.setBetaDict(beDict);
    cgd.setGammaDict(gaDict);
    cgd.setLambdaDict(laDict);
    cgd.setSigmaDict(siDict);
    cgd.setRhoDict(rDict);
    */
    cgd.setLbd(-1000000000.0);
    cgd.setUbd(10000000000.0);
    cgd.setIter(0);
}

//populate the restricted master problem parameters
void CGUtilities::populateInitialRMPParameters() {
    //populate data structure of TruckPaths and DronePaths
    int noTruckPaths = listOfTruckPaths.size();
    int noDronePaths = listOfDronePaths.size();
    TruckPaths truckPaths(noTruckPaths, listOfTruckPaths);
    DronePaths dronePaths(noDronePaths, listOfDronePaths);
    //populate arcs covered by truck paths parameters
    std::map<std::tuple<int, int, int>, double> arcsCBTPaths;
    std::map<std::tuple<int, int>, double> nodesCBTPaths;
    //default value
    for (int i = 0; i < noTruckPaths; i++) {
        for (auto& it : truckArcs) {
            arcsCBTPaths[std::make_tuple(i, it.first, it.second)] = 0.0;
        }
        for (auto& c : Ct) {
            nodesCBTPaths[std::make_tuple(i, c)] = 0.0;
        }
    }
    //insert truck path values
    int i = 0;
    for (auto& it : listOfTruckPaths) {
        for (int j = 0; j < it.size() - 1; j++) {
            arcsCBTPaths[std::make_tuple(i, it[j], it[j + 1])] = 1.0;
            if (j >= 1) {
                nodesCBTPaths[std::make_tuple(i, it[j])] = 1.0;
            }
        }
        i++;
    }
    //populate arcs covered by drone paths parameters
    std::map<std::tuple<int, int, int>, double> arcsCBDPaths;
    std::map<std::tuple<int, int>, double> nodesCBDPaths;
    std::map<std::tuple<int, int>, double> dronePSFRNodes;
    //default value
    for (int j = 0; j < noDronePaths; j++) {
        for (auto& it : droneArcs) {
            arcsCBDPaths[std::make_tuple(j, it.first, it.second)] = 0.0;
        }
        for (auto& c : Cd) {
            nodesCBDPaths[std::make_tuple(j, c)] = 0.0;
        }
        for (auto& s : S) {
            dronePSFRNodes[std::make_tuple(j, s)] = 0.0;
        }
    }
    //insert drone path values
    i = 0;
    for (auto& it : listOfDronePaths) {
        for (int j = 0; j < it.size() - 1; j++) {
            arcsCBDPaths[std::make_tuple(i, it[j], it[j + 1])] = 1.0;
            if (j >= 1) {
                nodesCBDPaths[std::make_tuple(i, it[j])] = 1.0;
            }
        }
        dronePSFRNodes[std::make_tuple(i, it[0])] = 1.0;
        i++;
    }  
    //populate rmpparameteres
    rmpp.setTruckPaths(truckPaths);
    rmpp.setDronePaths(dronePaths);
    rmpp.setArcsCBTruckPaths(arcsCBTPaths);
    rmpp.setArcsCBDronePaths(arcsCBDPaths);
    rmpp.setNodesCBTruckPaths(nodesCBTPaths);
    rmpp.setNodesCBDronePaths(nodesCBDPaths);
    rmpp.setDronePathsSFRNodes(dronePSFRNodes);
}

//create and solve rmp from available data
void CGUtilities::buildAndSolveRMProblem() {
    //build rmp model
    try{
        //set gurobi environment
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        // Create truck path variables
        GRBVar* pi = new GRBVar[rmpp.getTruckPaths().getNoTruckPaths()];
        for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++) {
            pi[i] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "pi_"+std::to_string(i));
        }
        // Create drone path variables
        GRBVar* di = new GRBVar[rmpp.getDronePaths().getNoDronePaths()];
        for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {
            di[i] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "di_"+std::to_string(i));
        }
        // Create truck arc variables
        GRBVar** ta = new GRBVar * [25];
        for (int i = 0; i < 25; i++) {
            ta[i] = new GRBVar[25];
            for (int j = 0; j < 25; j++) {
                ta[i][j] = model.addVar(0.0, 1.0, 1.0, GRB_CONTINUOUS, "ta_"+std::to_string(i) + "v" + std::to_string(j));
            }
        }
        // Create drone arc variables
        GRBVar** da = new GRBVar * [45];
        for (int i = 13; i < 45; i++) {                        
            da[i] = new GRBVar[45];
            for (int j = 13; j < 45; j++) {
                da[i][j] = model.addVar(0.0, 1.0, 1.0, GRB_CONTINUOUS, "da_"+std::to_string(i) + "v" + std::to_string(j));
            }
        }
        //define constraints 
        //constraint #2b
        //con2b = @constraint(rmp_model, sum(pi[t] for t in 1:cd.rmp_params.truckPaths.noTruckPaths) == cd.K1) #  == #
        GRBLinExpr expr = 0.0;
        for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++) {
            expr = expr + pi[i];
        }
        model.addConstr(expr == K1, "con2b");
        
        //constraint #2c
        //con2c = []
        //for arc in cd.truckArcs
        //con = @constraint(rmp_model, sum(pi[t] * cd.rmp_params.arcsCBTruckPaths[t, arc[1], arc[2]] for t in 1:cd.rmp_params.truckPaths.noTruckPaths) == ta[arc[1], arc[2]]) #   == #
        //push!(con2c, con)
        //end
        std::map<std::tuple<int, int, int>, double> arcCBTPMap = rmpp.getArcsCBTruckPaths();
        for (auto &it: truckArcs) {
            GRBLinExpr expr = 0.0;
            for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++ ) {
                expr = expr + pi[i] * arcCBTPMap[std::make_tuple(i, it.first, it.second)];
            }
            model.addConstr(expr == ta[it.first][it.second], "con2c"+std::to_string(it.first)+ "c" + std::to_string(it.second));
        }
        
        //constraint #2d
        //con2d = []
        //for c in cd.Ct
        //con = @constraint(rmp_model, sum(pi[t] * cd.rmp_params.nodesCBTruckPaths[t, c] for t in 1:cd.rmp_params.truckPaths.noTruckPaths) == 1) #   == #
        //push!(con2d, con)
        //end
        std::map<std::tuple<int, int>, double> nodesCBTPMap = rmpp.getNodesCBTruckPaths();
        for (auto &c: Ct) {
            GRBLinExpr expr = 0.0;
            for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++) {
                expr = expr + pi[i] * nodesCBTPMap[std::make_tuple(i, c)];
            }
            model.addConstr(expr == 1, "con2d"+std::to_string(c));
        }
        
        //constraint #2e
        //con2e = []
        //for s in cd.S
        //con = @constraint(rmp_model, sum(di[d] * cd.rmp_params.dronePathsSFRNodes[d, s] for d in 1:cd.rmp_params.dronePaths.noDronePaths) <= cd.K2) #   <= #
        //push!(con2e, con)
        //end
        std::map<std::tuple<int, int>, double> dronePSFRNMap = rmpp.getDronePathsSFRNodes();
        for (auto &s:S) {
            GRBLinExpr expr = 0.0;
            for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {
                //std::cout << "s : " << s << ", i : " << i << std::endl;
                expr = expr + di[i] * dronePSFRNMap[std::make_tuple(i,s)];
            }
            model.addConstr(expr <= K2, "con2e"+std::to_string(s));
        }
        
        //constraints #2f
        //con2f = []
        //for arc in cd.droneArcs
        //con = @constraint(rmp_model, sum(di[d] * cd.rmp_params.arcsCBDronePaths[d, arc[1], arc[2]] for d in 1:cd.rmp_params.dronePaths.noDronePaths) == da[arc[1], arc[2]]) #   == #
        //push!(con2f, con)
        //end
        std::map<std::tuple<int, int, int>, double> arcsCBDPMap = rmpp.getArcsCBDronePaths();
        for (auto &it:droneArcs) {
            GRBLinExpr expr = 0.0;
            for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {
                expr = expr + di[i] * arcsCBDPMap[std::make_tuple(i, it.first, it.second)];
            }
            model.addConstr(expr == da[it.first][it.second], "con2f"+std::to_string(it.first) + "c" + std::to_string(it.second));
        }
        
        //constraint #2g
        //con2g = []
        //for c in cd.Cd
        //con = @constraint(rmp_model, sum(di[d] * cd.rmp_params.nodesCBDronePaths[d, c] for d in 1:cd.rmp_params.dronePaths.noDronePaths) == 1) #   == #
        //push!(con2g, con)
        //end
        std::map<std::tuple<int, int>, double> nodesCBDPMap = rmpp.getNodesCBDronePaths();
        for (auto &c: Cd) {
            GRBLinExpr expr = 0.0;
            for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {
                expr = expr + di[i] * nodesCBDPMap[std::make_tuple(i, c)];
            }
            model.addConstr(expr == 1, "con2g"+std::to_string(c));
        }
        
        //#objective function
        //@objective(rmp_model, Min, sum(truckArcDistances[arc[1], arc[2]] * ta[arc[1], arc[2]] for arc in cd.truckArcs) + sum(droneArcDistances[arc[1], arc[2]] * da[arc[1], arc[2]] for arc in cd.droneArcs))
        GRBLinExpr objexpr = 0.0;
        for (auto &it:truckArcs) {
            objexpr += truckArcCosts[it] * ta[it.first][it.second];
        }
        for (auto &it:droneArcs) {
            objexpr += droneArcCosts[it] * da[it.first][it.second];
        }
        //set the objective function to the model
        model.setObjective(objexpr, GRB_MINIMIZE);
        //write the Gurobi model for later use
        model.write("model.mps");
        //optimize the model
        model.optimize();
        //get optimization status
        int status = model.get(GRB_IntAttr_Status);  //2 for optimal solution, and 13 for suboptimal solution
        if (status == 2 || status == 13) {
            //get obj val and update lower bound
            std::cout << "The model is feasible." << std::endl;
            double objval;
            objval = model.get(GRB_DoubleAttr_ObjVal);
            cgd.setObjVal(objval);
            if (objval < cgd.getLbd()) {
                cgd.setLbd(objval);
            }
            //save rmp primal solution
            std::map<int, double> piDict;
            std::map<int, double> diDict;
            std::map<std::pair<int, int>, double> taDict;
            std::map<std::pair<int, int>, double> daDict;
            for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++) {
                piDict[i] = pi[i].get(GRB_DoubleAttr_X);
            }
            for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {
                diDict[i] = di[i].get(GRB_DoubleAttr_X);
            }
            for (auto &it: truckArcs) {
                taDict[it] = ta[it.first][it.second].get(GRB_DoubleAttr_X);
            }    
            for (auto &it: droneArcs) {
                daDict[it] = da[it.first][it.second].get(GRB_DoubleAttr_X);
            }
            cgd.setPiDict(piDict);
            cgd.setDiDict(diDict);
            cgd.setTaDict(taDict);
            cgd.setDaDict(daDict);
            //save dual solution
            std::map<std::pair<int, int>, double> beta;
            std::map<int, double> gamma;
            std::map<int, double> lambda;
            std::map<std::pair<int, int>, double> sigma;
            std::map<int, double> rho;
            //dual solution for con2b
            //push!(alpha, dual(con2b))
            GRBConstr constr = model.getConstrByName("con2b");
            cgd.setAlpha(constr.get(GRB_DoubleAttr_Pi));
            //dual solutions for con2c
            //for i in 1:size(con2c)[1]
            //    push!(beta, dual(con2c[i]))
            //end
            for (auto &it: truckArcs) {
                GRBConstr constr = model.getConstrByName("con2c"+std::to_string(it.first)+ "c" + std::to_string(it.second));
                beta[std::make_pair(it.first, it.second)] = constr.get(GRB_DoubleAttr_Pi);
            }
            cgd.setBetaDict(beta);
            //dual solutions for con2d
            //for i in 1:size(con2d)[1]
            //   push!(gamma, dual(con2d[i]))
            //end
            for (auto &c: Ct) {
                GRBConstr constr = model.getConstrByName("con2d"+std::to_string(c));
                gamma[c] = constr.get(GRB_DoubleAttr_Pi);
            }
            cgd.setGammaDict(gamma);
            //dual solutions for con2e
            //for i in 1:size(con2e)[1]
            //   push!(lambda, dual(con2e[i]))
            //end
            for (auto &s: S) {
                GRBConstr constr = model.getConstrByName("con2e"+std::to_string(s));
                lambda[s] = constr.get(GRB_DoubleAttr_Pi);
            }
            cgd.setLambdaDict(lambda);
            //dual solutions for con2f
            //for i in 1:size(con2f)[1]
            //  push!(sigma, dual(con2f[i]))
            //end
            for (auto &it: droneArcs) {
                GRBConstr constr = model.getConstrByName("con2f"+std::to_string(it.first)+ "c" + std::to_string(it.second));
                sigma[std::make_pair(it.first, it.second)] = constr.get(GRB_DoubleAttr_Pi);
            }
            cgd.setSigmaDict(sigma);
            //dual solutions for con2g
            //for i in 1:size(con2g)[1]
            //  push!(rho, dual(con2g[i]))
            //end
            for (auto &c: Cd) {
                GRBConstr constr = model.getConstrByName("con2g"+std::to_string(c));
                rho[c] = constr.get(GRB_DoubleAttr_Pi);
            }
            cgd.setRhoDict(rho);
        }
    }
    catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }   
}

//find promising truck paths.
void CGUtilities::findPromisingTruckPaths() {
    //get new truck routes using ga 
    int capLimit = 10;
    double maxPathLength = 75000.0;
    //std::cout << "Uniform demand for each customer." << std::endl;
    std::map<int, int> cusToDemand;
    std::vector<int> cusCluster;
    for (int i = 1; i < 25; i++) {
        cusToDemand.insert(std::pair<int, int>(i, 1));//unit demand
        cusCluster.push_back(i);
    }
    Geneticalgorithm genAlg;
    genAlg = Geneticalgorithm(0, capLimit, maxPathLength, cusToDemand, distanceMatrix, cusCluster);
    //std::cout << "\nGenetic algorithm started." << std::endl;
    genAlg.runGeneticAlgorithm();
    std::list<std::vector<int>> truckPaths;
    std::vector<int> path;
    for (auto &it: genAlg.getLastPopulation()) {
        if (it.getFitness() <= genAlg.getGASolution().getFitness()*1.5) {
            for (auto &itt:it.getChromosomeRepresentation()) {
                if (itt == 0 && path.empty()) {
                    path.push_back(0);
                }
                else if (itt == 0 && (!path.empty()) && (path.back() != 0)) {
                    path.push_back(0);
                    truckPaths.push_back(path);
                    path.clear();
                    path.push_back(0);
                }
                else if (itt != 0) {
                    path.push_back(itt);
                }
            }
        }
    }
    //remove duplicates from the paths
    //eliminate duplicates 
    for (auto it = truckPaths.begin(); it != truckPaths.end(); ++it) {
        for (auto itt = std::next(it); itt != truckPaths.end();) {
            if (*it == *itt) {
                itt = truckPaths.erase(itt);
            }
            else {
                ++itt;
            }
        }
    }
    //find promising truck paths
    std::list<std::vector<int>> pTruckPaths;
    std::map<std::pair<int, int>, double> betaMap = cgd.getBetaDict();
    std::map<int, double> gammaMap = cgd.getGammaDict();
    for (auto &it: truckPaths) {
        double reducedCost = -1*cgd.getAlpha();
        for (int i = 0; i < it.size()-1; i++) {
            reducedCost = reducedCost - (betaMap[std::make_pair(it[i],it[i+1])] + gammaMap[it[i+1]]);
        }
        reducedCost = reducedCost - betaMap[std::make_pair(it[it.size()-2], it[it.size()-1])];
        if (reducedCost < -0.10) {
            //std::cout << "The reduced cost is : " << reducedCost << std::endl;
            //check whether the path already exist in the truck path list
            bool isTaken = false;
            for (auto &tp:rmpp.getTruckPaths().getVecTruckPaths()) {
                if (tp == it) {
                    isTaken = true;
                }
            }
            if (!isTaken) {
                pTruckPaths.push_back(it);
            }
        }
    }
    //update promising truck path list
    promTPaths = pTruckPaths;    
}

//find promising drone paths
void CGUtilities::findPromisingDronePaths() {
    std::list<std::vector<int>> pDronePaths;
    for (auto &s:S) {
        //get new drone routes using ga 
        int capLimit = 3;
        double maxPathLength = 15000.0;
        //std::cout << "Uniform demand for each customer." << std::endl;
        std::map<int, int> cusToDemand;
        std::vector<int> cusCluster;
        for (int i = 25; i < 45; i++) {
            if (distanceMatrix[s][i] <= 7500.0) {
                cusToDemand.insert(std::pair<int, int>(i, 1));//unit demand
                cusCluster.push_back(i);
            }
        }
        Geneticalgorithm genAlg;
        genAlg = Geneticalgorithm(s, capLimit, maxPathLength, cusToDemand, distanceMatrix, cusCluster);
        //std::cout << "\nGenetic algorithm started." << std::endl;
        genAlg.runGeneticAlgorithm();
        std::list<std::vector<int>> dronePaths;
        std::vector<int> path;
        for (auto& it : genAlg.getLastPopulation()) {
            if (it.getFitness() <= genAlg.getGASolution().getFitness() * 1.2) {
                for (auto& itt : it.getChromosomeRepresentation()) {
                    if (itt == s && path.empty()) {
                        path.push_back(s);
                    }
                    else if (itt == s && (!path.empty()) && (path.back() != s)) {
                        path.push_back(s);
                        dronePaths.push_back(path);
                        path.clear();
                        path.push_back(s);
                    }
                    else if (itt != s) {
                        path.push_back(itt);
                    }
                }
            }
        }
        //eliminate duplicates 
        for (auto it = dronePaths.begin(); it != dronePaths.end(); ++it) {
            for (auto itt = std::next(it); itt != dronePaths.end();) {
                if (*it == *itt) {
                    itt = dronePaths.erase(itt);
                }
                else {
                    ++itt;
                }
            }
        }
        //find promising drone paths
        std::list<std::vector<int>> pDronePaths;
        std::map<std::pair<int, int>, double> sigmaMap = cgd.getSigmaDict();
        std::map<int, double> rhoMap = cgd.getRhoDict(); 
        for (auto& it : dronePaths) {
            double reducedCost = -1 * cgd.getLambdaDict()[s];
            for (int i = 0; i < it.size() - 1; i++) {
                reducedCost = reducedCost - (sigmaMap[std::make_pair(it[i], it[i + 1])] + rhoMap[it[i + 1]]);
            }
            reducedCost = reducedCost - sigmaMap[std::make_pair(it[it.size() - 2], it[it.size() - 1])];
            if (reducedCost < -0.1) {
                //std::cout << "The reduced cost is : " << reducedCost << std::endl;
                bool isTaken = false;
                for (auto& tp : rmpp.getDronePaths().getVecDronePaths()) {
                    if (tp == it) {
                        isTaken = true;
                    }
                }
                if (!isTaken) {
                    pDronePaths.push_back(it);
                }
            }
        }
    }
    //update promising truck path list
    promDPaths = pDronePaths;
}

//update rmpp with new paths
void CGUtilities::updateRMPParameters() {
    //populate data structure of TruckPaths and DronePaths 
    int noTruckPaths = rmpp.getTruckPaths().getNoTruckPaths();
    int noDronePaths = rmpp.getDronePaths().getNoDronePaths();
    std::list<std::vector<int>> vecTruckPaths = rmpp.getTruckPaths().getVecTruckPaths();
    std::list<std::vector<int>> vecDronePaths = rmpp.getDronePaths().getVecDronePaths();
    int noNewTruckPaths = promTPaths.size();
    int noNewDronePaths = promDPaths.size();
    //for i in 1:noNewTruckPaths
    //    push!(vecTruckPaths, cd.promTPaths[i])
    //end
    for (auto &it:promTPaths) {
        vecTruckPaths.push_back(it);
    }
    //for i in 1 : noNewDronePaths
    //    push!(vecDronePaths, cd.promDPaths[i])
    //end
    for (auto &it:promDPaths) {
        vecDronePaths.push_back(it);
    }
    //truckPaths = TruckPaths((noTruckPaths + noNewTruckPaths), vecTruckPaths)
    //dronePaths = DronePaths((noDronePaths + noNewDronePaths), vecDronePaths)
    TruckPaths truckPaths((noTruckPaths+noNewTruckPaths), vecTruckPaths);
    DronePaths dronePaths((noDronePaths+noNewDronePaths), vecDronePaths);
    //include arcs and nodes covered by new truck paths parameters 
    //arcsCBTruckPaths::Any  #h_t ^ ij
    //nodesCBTruckPaths::Any  #h_t ^ i
    std::map<std::tuple<int, int, int>, double> arcsCBTPaths = rmpp.getArcsCBTruckPaths();
    std::map<std::tuple<int, int>, double> nodesCBTPaths = rmpp.getNodesCBTruckPaths();
    //default value
    //for i in 1:noNewTruckPaths
    //    for arc in cd.truckArcs
    //        push!(arcsCBTPaths, ((noTruckPaths + i), arc[1], arc[2]) = > 0.0)
    //    end
    //    for c in cd.Ct
    //        push!(nodesCBTPaths, ((noTruckPaths + i), c) = > 0.0)
    //    end
    //end
    for (int i = 1; i <= noNewTruckPaths; i++) {
        for (auto &arc: truckArcs) {
            arcsCBTPaths[std::make_tuple((noTruckPaths + i), arc.first, arc.second)] = 0.0;
        }
        for (auto &c:Ct) {
            nodesCBTPaths[std::make_tuple((noTruckPaths + i), c)] = 0.0;
        }
    }
    //insert truck path values
    //for i in 1:noNewTruckPaths
    //   route = cd.promTPaths[i]
    //   routeSize = size(route)[1]
    //   for j in 1 : (routeSize - 1)
    //      push!(arcsCBTPaths, ((noTruckPaths + i), route[j], route[j + 1]) = > 1.0)
    //      if j >= 2
    //         push!(nodesCBTPaths, ((noTruckPaths + i), route[j]) = > 1.0)
    //      end
    //   end
    //end
    int i = 1;
    for (auto &it:promTPaths) {
        int size = it.size();
        for (int j = 0; j < size-1; j++) {
            arcsCBTPaths[std::make_tuple((noTruckPaths + i), it[j], it[j+1])] = 1.0;
            if (j>=1) {
                nodesCBTPaths[std::make_tuple((noTruckPaths + i), it[j])] = 1.0;
            }
        }
        i++;
    }
    //add arcs and nodes covered by new drone paths parameters 
    //arcsCBDronePaths::Any  #e_d^ij
    //nodesCBDronePaths::Any  #e_d^i
    //dronePathsSFRNodes::Any  #g_d^s 
    std::map<std::tuple<int, int, int>, double> arcsCBDPaths = rmpp.getArcsCBDronePaths();
    std::map<std::tuple<int, int>, double> nodesCBDPaths = rmpp.getNodesCBDronePaths();
    std::map<std::tuple<int, int>, double> dronePSFRNodes = rmpp.getDronePathsSFRNodes();
    //default value 
    //for i in 1:noNewDronePaths
    //    for arc in cd.droneArcs
    //        push!(arcsCBDPaths, ((noDronePaths+i), arc[1], arc[2]) => 0.0)
    //    end 
    //    for c in cd.Cd
    //        push!(nodesCBDPaths, ((noDronePaths+i), c) => 0.0)
    //    end 
    //    for s in cd.S
    //        push!(dronePSFRNodes, ((noDronePaths+i), s) => 0.0)
    //    end 
    //end 
    for (int i = 1; i <= noNewDronePaths; i++) {
        for (auto &arc: droneArcs) {
            arcsCBDPaths[std::make_tuple((noDronePaths + i), arc.first, arc.second)] = 0.0;
        }
        for (auto &c: Cd) {
            nodesCBDPaths[std::make_tuple((noDronePaths + i), c)] = 0.0;
        }
        for (auto &s: S) {
            dronePSFRNodes[std::make_tuple((noDronePaths + i), s)] = 0.0;
        }
    }
    //insert drone path values 
    //for i in 1:noNewDronePaths
    //    route = cd.promDPaths[i]
    //    routeSize = size(route)[1]
    //    for j in 1:(routeSize-1)
    //        push!(arcsCBDPaths, ((noDronePaths+i), route[j], route[j+1]) => 1.0)
    //        if j >= 2
    //            push!(nodesCBDPaths, ((noDronePaths+i), route[j]) => 1.0)   
    //        end  
    //    end 
    //    push!(dronePSFRNodes, ((noDronePaths+i), route[1]) => 1.0) #first node of the route represent the rendezvous node 
    //end 
    int k = 1;
    for (auto & it: promDPaths) {
        int size = it.size();
        for (int j = 0; j < size - 1; j++) {
            arcsCBDPaths[std::make_tuple((noDronePaths + k), it[j], it[j + 1])] = 1.0;
            if (j >= 1.0) {
                nodesCBDPaths[std::make_tuple((noDronePaths + k), it[j])] = 1.0;
            }
        }
        dronePSFRNodes[std::make_tuple((noDronePaths + k), it[0])] = 1.0;
        k++;
    }
    //#populate rmpparameteres    
    rmpp = RMPParameters(truckPaths, dronePaths, arcsCBDPaths, arcsCBTPaths, nodesCBDPaths, nodesCBTPaths, dronePSFRNodes);
}

//check whether the solution is integer
bool CGUtilities::isSolutionInteger() {
    bool isSolInt = true;
    for (auto& it : cgd.getPiDict()) {
        if (it.second > 0.0 && it.second < 1.0) {
            isSolInt = false;
            return isSolInt;
        }
    }
    for (auto &it : cgd.getDiDict()) {
        if (it.second > 0.0 && it.second < 1.0) {
            isSolInt = false;
            return isSolInt;
        }
    }
    for (auto &it : cgd.getTaDict()) {
        if (it.second > 0.0 && it.second < 1.0) {
            isSolInt = false;
            return isSolInt;
        }
    }
    for (auto &it : cgd.getDaDict()) {
        if (it.second > 0.0 && it.second < 1.0) {
            isSolInt = false;
            return isSolInt;
        }
    }
    return isSolInt;
}

//find integer solution of the rmp problem
void CGUtilities::getIntegerRMPSolution() {
    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env, "model.mps");
        //set truck path variables as binary
        GRBVar* pi = new GRBVar[rmpp.getTruckPaths().getNoTruckPaths()];//-promTPaths.size()
        for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++) {//-promTPaths.size()
            pi[i] = model.getVarByName("pi_" + std::to_string(i));
            pi[i].set(GRB_CharAttr_VType, GRB_BINARY);
        }
        //set drone path variables as binary
        GRBVar* di = new GRBVar[rmpp.getDronePaths().getNoDronePaths()];//-promDPaths.size()
        for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {//-promDPaths.size()
            di[i] = model.getVarByName("di_" + std::to_string(i));
            di[i].set(GRB_CharAttr_VType, GRB_BINARY);
        }
        //set truck arc variables as binary
        GRBVar** ta = new GRBVar * [25];
        for (int i = 0; i < 25; i++) {
            ta[i] = new GRBVar[25];
            for (int j = 0; j < 25; j++) {
                ta[i][j] = model.getVarByName("ta_"+std::to_string(i) + "v" + std::to_string(j));
                ta[i][j].set(GRB_CharAttr_VType, GRB_BINARY);
            }
        }
        //set drone arc variables as binary 
        GRBVar** da = new GRBVar * [45];
        for (int i = 13; i < 45; i++) {
            da[i] = new GRBVar[45];
            for (int j = 13; j < 45; j++) {
                da[i][j] = model.getVarByName("da_"+std::to_string(i) + "v" + std::to_string(j));
                da[i][j].set(GRB_CharAttr_VType, GRB_BINARY);
            }
        }
        //optimize the model
        model.optimize();
        //get optimization status
        int status = model.get(GRB_IntAttr_Status);  //2 for optimal solution, and 13 for suboptimal solution
        if (status == 2 || status == 13) {
            //get obj val and update lower bound
            std::cout << "The integer model is feasible." << std::endl;
            double objval;
            objval = model.get(GRB_DoubleAttr_ObjVal);
            cgd.setObjVal(objval);
            if (objval < cgd.getUbd()) {
                cgd.setUbd(objval);
            }
            //save rmp primal solution
            std::map<int, double> piDict;
            std::map<int, double> diDict;
            std::map<std::pair<int, int>, double> taDict;
            std::map<std::pair<int, int>, double> daDict;
            for (int i = 0; i < rmpp.getTruckPaths().getNoTruckPaths(); i++) {//-promTPaths.size()
                piDict[i] = pi[i].get(GRB_DoubleAttr_X);
            }
            for (int i = 0; i < rmpp.getDronePaths().getNoDronePaths(); i++) {//-promDPaths.size()
                diDict[i] = di[i].get(GRB_DoubleAttr_X);
            }
            for (auto& it : truckArcs) {
                taDict[it] = ta[it.first][it.second].get(GRB_DoubleAttr_X);
            }
            for (auto& it : droneArcs) {
                daDict[it] = da[it.first][it.second].get(GRB_DoubleAttr_X);
            }
            cgd.setPiDict(piDict);
            cgd.setDiDict(diDict);
            cgd.setTaDict(taDict);
            cgd.setDaDict(daDict);
        }
    }
    catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
}

void CGUtilities::printCGSolution() {
    std::cout << "The integer solution is given below : " << std::endl;
    std::cout << "The pi solution are : " << std::endl;
    for(auto &it: cgd.getPiDict()){
        if (it.second == 1.0) {
            std::cout << "Truck path key : " << it.first << ", Value : " << it.second << std::endl;
        }
    }
    std::cout << "The di solution are : " << std::endl;
    for (auto &it:cgd.getDiDict()) {
        if (it.second == 1.0) {
            std::cout << "Drone path key : " << it.first << ", Value : " << it.second << std::endl;
        }
    }
    std::cout << "The truck arc solution are : " << std::endl;
    for (auto &it:cgd.getTaDict()) {
        if (it.second == 1.0) {
            std::cout << "Truck arc : <" << it.first.first << ", " << it.first.second << ">, " << "Value : " << it.second << std::endl;
        }
    }
    std::cout << "The drone arc solution are : " << std::endl;
    for (auto& it : cgd.getDaDict()) {
        if (it.second == 1.0) {
            std::cout << "Drone arc : <" << it.first.first << ", " << it.first.second << ">, " << "Value : " << it.second << std::endl;
        }
    }
    std::cout << "No of CG iterations : " << cgd.getIter() << std::endl;
    std::cout << "Objective value : " << cgd.getObjVal() << std::endl;

}

void CGUtilities::runCGBasedHeuristic() {
    //generate initial truck and drone paths
    generateInitialTruckAndDronePaths();
    //populate CGData class with initial data
    populateInitialCGData();
    //populate RMPParameter class with inital data
    populateInitialRMPParameters();
    int iter = 0;
    while (true) {
        iter += 1;
        //build and solve restricted master problem
        buildAndSolveRMProblem();
        //find promising truck paths using hybrid genetic algorithm
        findPromisingTruckPaths();
        //find promising drone paths using hybrid genetic algorithm
        findPromisingDronePaths();
        if (promTPaths.size()+promDPaths.size() >= 1) {
            updateRMPParameters();
        }
        else {
            std::cout << "No more promising truck and drone paths found." << std::endl;
            break;
        }
    }
    cgd.setIter(iter);
    //check integrality of the cg solution
    if (isSolutionInteger()) {
        std::cout << "\nThe initial cgh gives integer solution." << std::endl;
        printCGSolution();
    }
    else {
        //solve the restricted master problem as integer programming problem
        std::cout << "For integer solution rmp needs to be solved as an integer programming problem." << std::endl;
        getIntegerRMPSolution();
        printCGSolution();
    }
}

