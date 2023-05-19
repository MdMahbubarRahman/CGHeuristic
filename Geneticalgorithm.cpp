#include "Geneticalgorithm.h"

//default constructor
Chromosome::Chromosome() {
	fitness					  = 0.0;
	sepInt					  = 0;
	isCapFeasible			  = false;
	isPathLengthFeasible      = false;
	maxRouteCapacity		  = 10;
	maxPathLength             = 5000.0;
}

//copy constructor
Chromosome::Chromosome(const Chromosome& chrom) {
	chromosome				  = chrom.chromosome;
	fitness					  = chrom.fitness;
	sepInt					  = chrom.sepInt;
	isCapFeasible			  = chrom.isCapFeasible;
	isPathLengthFeasible      = chrom.isPathLengthFeasible;
	maxRouteCapacity		  = chrom.maxRouteCapacity;
	maxPathLength             = chrom.maxPathLength;
}

//constructs chromosome from elements
Chromosome::Chromosome(std::vector<int> chrom, double fitNess, int spInt, bool isCapFeb, bool isPathLenFeb, int maxCap, double maxPathLen) {
	chromosome = chrom;
	fitness = fitNess;
	sepInt = spInt;
	isCapFeasible = isCapFeb;
	isPathLengthFeasible = isPathLenFeb;
	maxRouteCapacity = maxCap;
	maxPathLength = maxPathLen;
}

//returns chromosome
std::vector<int> Chromosome::getChromosomeRepresentation() {
	return chromosome;
}

//returns fitness value
double Chromosome::getFitness() {
	return fitness;
}

//returns seperator integer value
int Chromosome::getSepInt() {
	return sepInt;
}

//returns feasibility status
bool Chromosome::getFeasibilityStatus() {
	bool isFeasible = isCapFeasible && isPathLengthFeasible;
	return isFeasible;
}

//returns size of the chromosome
int Chromosome::getSize() {
	return chromosome.size();
}

//returns route capacity
int Chromosome::getMaxRouteCapacity() {
	return maxRouteCapacity;
}

//returns maximum path length
double Chromosome::getMaxPathLength() {
	return maxPathLength;
}

//prints the chromosome
void Chromosome::showChromosome() {
	//std::cout << "\nThe Chromosome representation is : " << std::endl;
	for (auto it : chromosome) {
		std::cout << it << " ";
	}
	std::cout << ";" << std::endl;
	std::cout << "The fitness value is : " << fitness << ";" << std::endl;
	std::cout << "The seperator integer is : " << sepInt << ";" << std::endl;
	std::cout << "The capacity feasibility status is : " << isCapFeasible << ";" << std::endl;
	std::cout << "The path length feasibility status is : " << isPathLengthFeasible << ";" << std::endl;
	std::cout << "The max route capacity is : " << maxRouteCapacity << ";" << std::endl;
	std::cout << "The max path length is : " << maxPathLength << ";" << std::endl;
}

//updates to feasible chromosome
void Chromosome::updateToFeasibleChromosome(std::map<int, int> demand, std::vector<std::vector<double>> distance) {
	if (getFeasibilityStatus()) {
		//std::cout << "\nThe chromosome representation is already feasible!" << std::endl;
	}
	else {
		//std::cout << "\nThe chromosome is not feasible! Feasibility restoration is in progress!!!" << std::endl;
		std::vector<int> infSol = chromosome;
		std::vector<int> tempStorage;
		chromosome.clear();
		//make the chromosome feasible		
		int capacity = 0;
		double cost = 0.0;
		double tailCost = 0.0;
		int preval = sepInt;
		for (int i = 0; i < infSol.size(); ++i) {
			if (infSol.at(i) != sepInt) {
				tempStorage.push_back(infSol.at(i));
			}
		}
		//update chromosome
		chromosome.push_back(sepInt);
		for (int i = 0; i < tempStorage.size(); i++) {
			capacity += demand[tempStorage[i]];
			cost += distance[preval][tempStorage[i]];
			tailCost = distance[tempStorage[i]][sepInt];
			if (capacity <= maxRouteCapacity && (cost+tailCost) <= maxPathLength) {
				chromosome.push_back(tempStorage[i]);
				preval = tempStorage[i];
			}
			else {
				chromosome.push_back(sepInt);
				chromosome.push_back(tempStorage[i]);
				capacity = demand[tempStorage[i]];
				cost = distance[sepInt][tempStorage[i]];//consider single path is exceptional and not subject to maxpathlength restriction
				preval = tempStorage[i];
			}
		}
		chromosome.push_back(sepInt);
		// update fitness
		double fitCost = 0.0;
		int pval = sepInt;
		int val = 0;
		for (auto it = chromosome.begin(); it != chromosome.end(); ++it) {
			if (*it != sepInt) {
				val = *it;
			}
			else {
				val = sepInt;
			}
			fitCost += distance[pval][val];
			pval = val;
		}
		fitCost += distance[pval][sepInt];
		fitness = fitCost;
		//update feasibility
		isCapFeasible = true;
		isPathLengthFeasible = true; 
		//std::cout << "\nFeasible chromosome has been found!" << std::endl;
	}
}


//default constructor
CrossOver::CrossOver() {
	//std::cout << "\nThe default constructor for cross over class is called!" << std::endl;
}

//copy constructor
CrossOver::CrossOver(const CrossOver& crossr) {
	//std::cout << "\nThe copy constructor of crossover class is called!" << std::endl;
	parent1 = crossr.parent1;
	parent2 = crossr.parent2;
	offspring = crossr.offspring;
}

//crossover class construction with parents
CrossOver::CrossOver(Chromosome par1, Chromosome par2) {
	parent1 = par1;
	parent2 = par2;
}

//performs partially mapped crossover operation
void CrossOver::performPertiallyMappedCrossover(std::map<int, int> demand, std::vector<std::vector<double>> distance) {
	std::vector<int> par1 = parent1.getChromosomeRepresentation();
	std::vector<int> par2 = parent2.getChromosomeRepresentation();
	int sepInt1 = parent1.getSepInt();
	int sepInt2 = parent2.getSepInt();
	int sizeOne = par1.size();
	int sizeTwo = par2.size();
	int sizeDeff = 0;
	bool flag1 = false;
	bool flag2 = false;
	if (sizeOne - sizeTwo > 0) {
		sizeDeff = sizeOne - sizeTwo;
		flag1 = true;
	}
	else if(sizeTwo - sizeOne > 0) {
		sizeDeff = sizeTwo - sizeOne;
		flag2 = true;
	}
	else {
		sizeDeff = 0;
	}
	//adjust the lengths of the parents
	for (int k = 0; k < sizeDeff; ++k) {
		if (flag1 == true){
			par2.push_back(sepInt2);
		}
		else if(flag2 == true) {
			par1.push_back(sepInt1);
		}
		else {
			//std::cout << "\nBoth parents have same size.\n" << std::endl;
		}
	}
	
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> distr(int(par1.size()/2), (par1.size()-1));
	std::map<int, int> dict;
	int randVar = distr(gen);
	int sepInt = parent1.getSepInt();
	//dictionary	
	for (int i = 0; i < par2.size(); ++i) {
		if (par2.at(i) != sepInt) {
			dict.insert(std::pair<int, int>(par2.at(i), i));
		}
	}
	//partially mapped crossover
	for (int i = randVar; i < par1.size(); ++i) {
		int val1 = par1.at(i);
		int val2 = par2.at(i);
		if (val1 != val2 && val1 != sepInt && val2 != sepInt) {
			int indx = dict[val1];
			par2[indx] = val2;
			par2[i] = val1;
			dict.erase(val1);
			dict.erase(val2);
			dict.insert(std::pair<int, int>(val2, indx));
			dict.insert(std::pair<int, int>(val1, i));
		}
		else if (val1 != val2 && val1 == sepInt) {
			par2[i] = val1;
			for (int j = 0; j < par2.size(); ++j) {
				if (par2.at(j) == sepInt) {
					dict.erase(val2);
					dict.insert(std::pair<int, int>(val2, j));
					par2[j] = val2;
					break;
				}
			}
		}
		else if (val1 != val2 && val2 == sepInt) {
			int indx = dict[val1];
			par2[i] = val1;
			dict.erase(val1);
			dict.insert(std::pair<int, int>(val1, i));
			par2[indx] = val2;
		}
		else
			continue;
	}
	//fitness
	double cost = 0.0;
	int preval = sepInt;
	int val = 0;
	for (auto it = par2.begin(); it != par2.end(); ++it) {
		if (*it != sepInt) {
			val = *it;
		}
		else {
			val = sepInt;
		}
		cost += distance[preval][val];
		preval = val;
	}
	cost += distance[preval][sepInt];
	//check feasibility
	bool feasible = true;
	int capacity = 0;
	double pathCost = 0.0;
	int pval = sepInt;
	for (int i = 0; i < par2.size(); i++) {
		if (par2[i] != sepInt) {
			capacity += demand[par2[i]];
			pathCost += distance[pval][par2[i]];
			if (capacity > parent1.getMaxRouteCapacity() || pathCost > parent1.getMaxPathLength()) {
				feasible = false;
				break;
			}
			else {
				pval = par2[i];
			}
		}
		else {
			pathCost += distance[pval][sepInt];
			if (pathCost > parent1.getMaxPathLength()) {
				feasible = false;
				break;
			}
			else {
				capacity = 0;
				pathCost = distance[sepInt][par2[i]];
				pval = par2[i];
			}
		}
	}
	//offspring after crossover
	Chromosome child(par2, cost, sepInt, feasible, feasible, parent1.getMaxRouteCapacity(), parent1.getMaxPathLength());
	if(!child.getFeasibilityStatus()){
		child.updateToFeasibleChromosome(demand, distance);
	}
	offspring = child;
}

//prints parent 1
void CrossOver::showParent1() {
	std::cout << "The first parent is : " << std::endl;//need to catch in buffer
	parent1.showChromosome();
}

//prints parent 2
void CrossOver::showParent2() {
	std::cout << "The second parent is : " << std::endl;
	parent2.showChromosome();
}

//prints offspring
void CrossOver::showOffspring() {
	std::cout << "The offspring is : " << std::endl;
	offspring.showChromosome();
}

//returns parent 1
Chromosome CrossOver::getParent1() {
	return parent1;
}

//returns parent 2
Chromosome CrossOver::getParent2() {
	return parent2;
}

//returns offspring
Chromosome CrossOver::getOffspring() {
	return offspring;
}

//default constructor
Mutation::Mutation() {
	//std::cout << "The default mutation constructor is called!" << std::endl;
}
	
//copy constructor
Mutation::Mutation(const Mutation & mutatn){
	//std::cout << "The copy constructor of the mutation class is called!" << std::endl;
	originalChrom = mutatn.originalChrom;
	mutatedChrom = mutatn.mutatedChrom;
}

//mutation constructs with original chromosome
Mutation::Mutation(Chromosome origChrom) {
	originalChrom = origChrom;
}

//returns premutation chromosome
Chromosome Mutation::getOriginalChromosome() {
	return originalChrom;
}

//returns mutated chromosome
Chromosome Mutation::getMutatedOffspring() {
	return mutatedChrom;
}

//prints premutation chromosome
void Mutation::showOriginalChromosome() {
	std::cout << "The premutated chromosome is : " << std::endl;
	originalChrom.showChromosome();
}

//prints mutated chromosome
void Mutation::showMutatedChromosome() {
	std::cout << "The mutated chromosome is : " << std::endl;
	mutatedChrom.showChromosome();
}

//performs chain mutation
void Mutation::performChainMutation(std::map<int, int> demand, std::vector<std::vector<double>> distance) {
	std::vector<int> chromToMutat = getOriginalChromosome().getChromosomeRepresentation();
	int size = getOriginalChromosome().getSize();
	int sepInt = getOriginalChromosome().getSepInt();
	int capacityLimit = getOriginalChromosome().getMaxRouteCapacity();
	double maxPathLen = getOriginalChromosome().getMaxPathLength();
	//find no of genes
	std::set<int> GeneDiversity;
	int val = 0;
	for (auto it : chromToMutat) {
		GeneDiversity.insert(it);
	}
	if (GeneDiversity.size() < 20) {
		val = 2;
	}
	else {
		val = ceil(sqrt(GeneDiversity.size()));
	}
	//no. of genes
	std::random_device rd;
	std::mt19937 gen(rd());
	int numGene = 2;
	if (val > 2) {
		std::uniform_int_distribution<> distr(2, val);
		numGene = distr(gen);
	}
	//choose the genes
	std::vector<int> geneIndxVec;
	bool flag = false;
	while (geneIndxVec.size() != numGene) {
		std::uniform_int_distribution<> distr(0, (size - 1));
		int geneIndx = distr(gen);
		//std::cout << "the indx is : " << geneIndx << std::endl;
		if (chromToMutat[geneIndx] != sepInt) {
			if (geneIndxVec.size() > 0) {
				for (int i = 0; i < geneIndxVec.size(); ++i) {
					if (geneIndxVec.at(i) == geneIndx) {
						flag = true;
						break;
					}
				}
			}
			if (flag == false) {
				geneIndxVec.push_back(geneIndx);
			}
			flag = false;
		}
	}
	//perform mutation
	int prev = chromToMutat[geneIndxVec.at(0)];
	int current = 0;
	for (int i = 1; i < geneIndxVec.size(); ++i) {
		current = chromToMutat[geneIndxVec.at(i)];
		chromToMutat[geneIndxVec.at(i)] = prev;
		prev = current;
	}
	chromToMutat[geneIndxVec.at(0)] = prev;
	//calculate fitness/cost
	double cost = 0;
	int preVal = sepInt;
	int vall = 0;
	for (auto& it : chromToMutat) {
		if (it != sepInt) {
			vall = it;
		}
		else {
			vall = sepInt;
		}
		cost = cost + distance[preVal][vall];
		//std::cout << preVal << " " << val << " " << distance[preVal][vall] << " " << std::endl;
		preVal = vall;
	}
	cost = cost + distance[preVal][sepInt];
	//check feasibility
	bool feasible = true;
	int capacity = 0;
	double pathCost = 0.0;
	int pval = sepInt;
	for (int i = 0; i < chromToMutat.size(); i++) {
		if (chromToMutat[i] != sepInt) {
			capacity += demand[chromToMutat[i]];
			pathCost += distance[pval][chromToMutat[i]];
			if (capacity > capacityLimit || pathCost > maxPathLen) {
				feasible = false;
				break;
			}
			else {
				pval = chromToMutat[i];
			}
		}
		else {
			pathCost += distance[pval][sepInt];
			if (pathCost > maxPathLen) {
				feasible = false;
				break;
			}
			else {
				capacity = 0;
				pathCost = distance[sepInt][chromToMutat[i]];
				pval = chromToMutat[i];
			}
		}
	}
	//get the mutated child
	Chromosome child(chromToMutat, cost, sepInt, feasible, feasible, capacityLimit, maxPathLen);
	if (!child.getFeasibilityStatus()) {
		child.updateToFeasibleChromosome(demand, distance);
	}
	mutatedChrom = child;
}

//comparator
bool Comparator::operator()(Chromosome &a, Chromosome &b){
	return (a.getFitness() > b.getFitness());
}


//default constructor
Population::Population() {
	//std::cout << "The default constructor of the population class is called!" << std::endl;
	diversitySize  = 0;
	populationSize = 50;
	numberOfNodes  = 0;
	depotNode      = 0;
	capacityLimit  = 0;
	kChainLength   = 0;
	sWapLength     = 0;
}

//copy constructor
Population::Population(const Population & gentn) {
	//std::cout << "The copy constructor of the population class is called!" << std::endl;
	diversitySize  = gentn.diversitySize;
	populationSize = gentn.populationSize;
	numberOfNodes  = gentn.numberOfNodes;
	depotNode      = gentn.depotNode;
	capacityLimit  = gentn.capacityLimit;
	maxPathLength  = gentn.maxPathLength;
	kChainLength   = gentn.kChainLength;
	sWapLength     = gentn.sWapLength;
	population     = gentn.population;
	demand		   = gentn.demand;
	distance	   = gentn.distance;
	customerCluster = gentn.customerCluster;
	offspring	   = gentn.offspring;
	populationBest = gentn.populationBest;
	crossOverChild = gentn.crossOverChild;
	mutationChild  = gentn.mutationChild;
}

//construct population with parameters
Population::Population(int popSize, int numNodes, int depNode, int capLimit, double maxPathlen, int kChain, int sWap, std::map<int, int> dem, std::vector<std::vector<double>> dist, std::vector<int> cusCluster) {
	populationSize = popSize;
	numberOfNodes  = numNodes;
	depotNode      = depNode;
	capacityLimit  = capLimit;
	maxPathLength  = maxPathlen;
	kChainLength   = kChain;
	sWapLength     = sWap;
	demand         = dem;
	distance       = dist;
	customerCluster = cusCluster;
}

//return population
std::list<Chromosome> Population::getPopulation() {
	return population;
}

//print population
void Population::showPopulation() {
	//std::cout << "The chromosomes of the current population are : " << std::endl;
	for (auto & it: population) {
		it.showChromosome();
		std::cout << std::endl;
	}
}

//updates population with the offspring
void Population::updatePopulationWithOffspring() {
	population.push_back(offspring);
	std::list<Chromosome>::iterator iter;
	double cost = 0;
	for (auto it = population.begin(); it != population.end(); ++it) {
		if ((*it).getFitness() > cost) {
			cost = (*it).getFitness();
			iter = it;
		}
	}
	population.erase(iter);
}

//returns offspring after genetic operations 
Chromosome Population::getOffspring() {
	return offspring;
}

//removes clones with random Chromosome
void Population::manageClones() {
	/*
	std::list<Chromosome>::iterator iter1, iter2;
	double cost = 0;
	for (auto it = population.begin(); it != population.end(); ++it) {
		if ((*it).getFitness() > cost) {
			cost = (*it).getFitness();
			iter1 = it;
		}
	}
	population.erase(iter1);
	*/
}

//measures diversity size of a generation of chromosomes
void Population::measureDiversitySize() {
	std::set<double> costs;
	for (auto & it : population) {
		costs.insert(it.getFitness());
	}
	diversitySize = costs.size();
	//std::cout << "The diveristy size is : " << diversitySize << std::endl;
}

//returns the size of the diversity present in a generation
int Population::getDiversitySize() {
	measureDiversitySize();
	return diversitySize;
}

//returns generation best chromosome
Chromosome Population::getBestChromosome() {
	double cost = INFINITY;
	std::list<Chromosome>::iterator iter;
	for (auto it = population.begin(); it != population.end(); ++it) {
		if ((*it).getFitness()<cost) {
			cost = (*it).getFitness();
			iter = it;
		}
	}
	populationBest = (*iter);
	return populationBest;
}

//returns offspring after cross over
Chromosome Population::getCrossOverChild() {
	return crossOverChild;
}

//returns offspring after mutation
Chromosome Population::getMutationChild() {
	return mutationChild;
}

//generates a random chromosome
Chromosome Population::generateRandomChromosome() {
	std::random_device rd;
	std::mt19937 gen(rd());
	//populate chromosome representation
	int totalDemand = 0;
	for (int i = 0; i < customerCluster.size(); ++i) {
		totalDemand += demand[customerCluster.at(i)];
		//std::cout << "\nCustomer location in the cluster : " << i << " customer : " << customerCluster.at(i) << std::endl;
	}
	int numDepotNodes = ceil(double(totalDemand) / double(capacityLimit));
	std::list<int> nodeBox;
	for (int i = 0; i < customerCluster.size(); ++i)
		nodeBox.push_back(customerCluster.at(i));
	for (int j = 0; j < (4 * numDepotNodes - 1); ++j)
		nodeBox.push_back(depotNode);
	/*
	std::cout << "\nShow the content of the node Box" << std::endl;
	for (auto it: nodeBox) {
		std::cout << " " << it << std::endl;
	}
	*/
	std::vector<int> chrom_container;
	chrom_container.push_back(depotNode);
	while(!nodeBox.empty()) {
		auto it = nodeBox.begin();
		std::uniform_int_distribution<> distr(0, (nodeBox.size() - 1));
		int num = distr(gen);
		std::advance(it, num);
		chrom_container.push_back(*it);
		nodeBox.erase(it);
	}
	//std::cout << "\nChromosome has been generated" << std::endl;
	chrom_container.push_back(depotNode);
	//std::cout << "The depot node is : " << depotNode << std::endl;
	/*
	std::cout << "Show the chromosome representation :" << std::endl;
	for (auto &it: chrom_container) {
		std::cout <<  it << " ";
	}
	std::cout << std::endl;
	*/
	//calculate fitness/cost
	double cost = 0;
	int preVal = depotNode;
	int val = 0;
	for (auto& it : chrom_container) {
		if (it != depotNode) {
			val = it;
		}
		else {
			val = depotNode;
		}
		cost = cost + distance[preVal][val];
		//std::cout << "source : " << preVal << " destination : " << val << " distance : " << distance[preVal][val] << " " << std::endl;
		preVal = val;
	}
	cost = cost + distance[preVal][depotNode];
	//std::cout << "\nCost of the chromosome representation has been updated" << std::endl;
	//check feasibility
	bool feasible = true;
	int capacity = 0;
	double pathCost = 0.0;
	int pval = depotNode;
	for (int i = 0; i < chrom_container.size(); i++) {
		if (chrom_container[i] != depotNode) {
			capacity += demand[chrom_container[i]];
			pathCost += distance[pval][chrom_container[i]];
			if (capacity > capacityLimit || pathCost > maxPathLength) {
				feasible = false;
				break;
			}
			else {
				pval = chrom_container[i];
			}
		}
		else {
			pathCost += distance[pval][depotNode];
			if (pathCost > maxPathLength) {
				feasible = false;
				break;
			}
			else {
				capacity = 0;
				pathCost = distance[depotNode][chrom_container[i]];
				pval = chrom_container[i];
			}
		}
	}
	//std::cout << "\nFeasibility check for the chromosome representation has been done." << std::endl;
	Chromosome crm(chrom_container, cost, depotNode, feasible, feasible, capacityLimit, maxPathLength);
	if (!crm.getFeasibilityStatus()) {
		crm.updateToFeasibleChromosome(demand, distance);
	}
	return crm;
}

//populates first generation of population
void Population::populatePopulation() {
	//std::cout << "\nPopulation size : " << populationSize << std::endl;
	int j = 0;
	for (int i = 0; i < populationSize; ++i) {
		Chromosome crm = generateRandomChromosome();
		population.push_back(crm);
		j++;
	}
	//std::cout << "\nNumber of chromosome generated : " << j << std::endl;
	manageClones();
}

//perfoms crossover on the current population
void Population::performCrossOver() {
	//std::cout << "\nCrossover started." << std::endl;
	std::list<Chromosome>::iterator iter1, iter2, iter3, iter4, iter5;
	double cost1 = INFINITY;
	double cost2 = INFINITY;
	double cost3 = INFINITY;
	double cost4 = INFINITY;
	double cost5 = INFINITY;
	for (auto it = population.begin(); it != population.end(); ++it) {
		if ((*it).getFitness() < cost1) {
			cost1 = (*it).getFitness();
			iter1 = it;
		}
		else if ((*it).getFitness() <= cost2 && (*it).getFitness() >= cost1) {
			if (it != iter1) {
				cost2 = (*it).getFitness();
				iter2 = it;
			}
		}
		else if ((*it).getFitness() <= cost3 && (*it).getFitness() >= cost2) {
			if (it != iter1 && it != iter2) {
				cost3 = (*it).getFitness();
				iter3 = it;
			}
		}
		else if ((*it).getFitness() <= cost4 && (*it).getFitness() >= cost3) {
			if (it != iter1 && it != iter2 && it != iter3) {
				cost4 = (*it).getFitness();
				iter4 = it;
			}
			
		}
		else if ((*it).getFitness() <= cost5 && (*it).getFitness() >= cost4) {
			if (it != iter1 && it != iter2 && it != iter3 && it != iter4) {
				cost5 = (*it).getFitness();
				iter5 = it;
			}
		}
	}
	
	Chromosome parent1;
	Chromosome parent2;
	
	//randomly choose two chromosomes out of the top 5 chromosomes
	std::random_device rd;
	std::mt19937 genr(rd());
	int parent1Number = 0;
	int parent2Number = 0;
	int val = 0;
	
	for (int i = 0; i < 100; i++) {
		std::uniform_int_distribution<> distr(1, 5);
		val = distr(genr);
		if (parent1Number == 0) {
			parent1Number = val;
			//std::cout<<"\nParent 1 Number: " << parent1Number << std::endl;
		}
		else if (parent2Number == 0 && val != parent1Number) {
			parent2Number = val;
			//std::cout<<"val : "<<val<<"\nParent 2 Number: " << parent2Number << std::endl;
		}
		else if (parent1Number != 0 && parent2Number != 0) {
			break;
		}
	}
	
	//assign first parent
	switch (parent1Number){
	case 1: parent1 = *iter1;
		break;
	case 2: parent1 = *iter2;
		break;
	case 3: parent1 = *iter3;
		break;
	case 4: parent1 = *iter4;
		break;
	case 5: parent1 = *iter5;
		break;
	}
	//assign second parent
	switch (parent2Number) {
	case 1: parent2 = *iter1;
		break;
	case 2: parent2 = *iter2;
		break;
	case 3: parent2 = *iter3;
		break;
	case 4: parent2 = *iter4;
		break;
	case 5: parent2 = *iter5;
		break;
	}
	//perform crossover
	CrossOver cross(parent2, parent1);
	cross.performPertiallyMappedCrossover(demand, distance);
	crossOverChild = cross.getOffspring();
	//std::cout << "\nCrossover ended." << std::endl; 
}

//return tabu search running time
double Population::returnTSRunningTime() {
	return TSRunningTime;
}

//performs mutation on the current population
void Population::performMutation() {
	//std::cout << "Show the cross over child" << std::endl;
	//crossOverChild.showChromosome();
	Mutation mut(crossOverChild);
	mut.performChainMutation(demand, distance);
	mutationChild = mut.getMutatedOffspring();
	//std::cout << "The mutated chromosome is : " << std::endl;
	//mutationChild.showChromosome();
	FeasibleSolution febSol = getFeasibleSolutionFromChromosome(mutationChild);
	//std::cout << "The Feasible solution version of the mutated child is : " << std::endl;
	//febSol.showSolution();

	auto startTS = high_resolution_clock::now();
	using std::chrono::duration;

	Tabusearch tabu(febSol, demand, distance, kChainLength, sWapLength, capacityLimit, maxPathLength);
	//std::cout << "\nRun tabu search" << std::endl;
	tabu.runTabuSearch();
	FeasibleSolution febChild = tabu.getIncumbentSolution();
	//std::cout << "The tabu search solution : " << std::endl;
	//febChild.showSolution();

	auto stopTS = high_resolution_clock::now();
	duration<double, std::milli> ms_double = stopTS - startTS;

	TSRunningTime = double(ms_double.count()) / 1000;

	Chromosome chromChild = getChromosomeFromFeasibleSolution(febChild);
	//std::cout << "The chromosome version of the tabu search solution is : " << std::endl;
	//chromChild.showChromosome();
	offspring = chromChild;
	//offspring = mutationChild;
}
//converst a chromosome to a feasible solution 
FeasibleSolution Population::getFeasibleSolutionFromChromosome(Chromosome chrom) {
	std::vector<int> chrom_rep = chrom.getChromosomeRepresentation();
	int sepInt = chrom.getSepInt();
	int fitness = chrom.getFitness();
	double maxPath = maxPathLength;
	std::list<int> sol_rep;
	for (auto it : chrom_rep) {
		sol_rep.push_back(it);
	}
	FeasibleSolution febSol(sol_rep, fitness, sepInt, 0, 0, maxPath);
	return febSol;
}

//converts feasible solution to a chromosome
Chromosome Population::getChromosomeFromFeasibleSolution(FeasibleSolution febSol) {
	std::list<int> feb_sol = febSol.getSolution();
	int sepInt = febSol.getSeparatorIntVal();
	int fitness = febSol.getCost();
	bool feasible = true;
	std::vector<int> chrom_sol;
	for (auto it : feb_sol) {
		chrom_sol.push_back(it);
	}
	Chromosome crm(chrom_sol, fitness, sepInt, feasible, feasible, capacityLimit, maxPathLength);
	return crm;
}

//default constructor
Geneticalgorithm::Geneticalgorithm() {
	//std::cout << "The default constructor of the genetic algorithm has been called!" << std::endl;
	maxIterations			= 100;
	populationSize			= 0;
	numberOfNodes			= 0;
	depotNode				= 0;
	capacityLimit			= 0;
	maxPathLength           = 0.0;
	kChainLength			= 0;
	sWapLength				= 0;
}

//copy constructor	
Geneticalgorithm::Geneticalgorithm(const Geneticalgorithm & ga) {
	//std::cout << "The copy constructor of the genetic algorithm has been called!" << std::endl;
	maxIterations			= ga.maxIterations;
	populationSize			= ga.populationSize;
	numberOfNodes			= ga.numberOfNodes;
	depotNode				= ga.depotNode;
	capacityLimit			= ga.capacityLimit;
	maxPathLength           = ga.maxPathLength;
	kChainLength		    = ga.kChainLength;
	sWapLength				= ga.sWapLength;
	demand                  = ga.demand;
	distance                = ga.distance;
	customerCluster         = ga.customerCluster;
	ppl						= ga.ppl;
	initialSolution			= ga.incumbentSolution;
	incumbentSolution		= ga.incumbentSolution;
	bestSolution			= ga.bestSolution;
	generationBestSolutions = ga.generationBestSolutions;
	generationalOffsprings	= ga.generationalOffsprings;
}

//construct ga with initial values
Geneticalgorithm::Geneticalgorithm(int popSize, int numNodes, int depNode, int capLimit, double maxPathLen, int kChain, int sWap, std::map<int, int> dem, std::vector<std::vector<double>> dist, std::vector<int> cusCluster) {
	maxIterations			= 100;
	populationSize			= popSize;
	numberOfNodes		    = numNodes;
	depotNode				= depNode;
	capacityLimit			= capLimit;
	maxPathLength           = maxPathLen;
	kChainLength			= kChain;
	sWapLength				= sWap;
	demand					= dem;
	distance				= dist;
	customerCluster         = cusCluster;
}

//constructor
Geneticalgorithm::Geneticalgorithm(int depNode, int capLimit, double maxPathLen, std::map<int, int> dem, std::vector<std::vector<double>> dist, std::vector<int> cusCluster) {
	maxIterations        = 50;//20
	populationSize       = 50;
	depotNode            = depNode;
	capacityLimit        = capLimit;
	maxPathLength        = maxPathLen;
	kChainLength         = 5;
	sWapLength           = 5;
	demand               = dem;
	distance             = dist;
	customerCluster      = cusCluster;
	std::set<int> nodes;
	for (auto it: cusCluster) {
		nodes.insert(it);
	}
	nodes.insert(depotNode);
	numberOfNodes = nodes.size();
}

//populate initial generation
void Geneticalgorithm::populateInitialGeneration() {
	Population population(populationSize, numberOfNodes, depotNode, capacityLimit, maxPathLength, kChainLength, sWapLength, demand, distance, customerCluster);
	ppl = population;
	ppl.populatePopulation();
	//std::cout << "\nInitial generation has been populated" << std::endl;
}

//prints current generation best solution
void Geneticalgorithm::showCurrentGenerationBestChromosome() {
	Chromosome bestSol = ppl.getBestChromosome();
	bestSol.showChromosome();
}

//returns best solutions of every generation
std::vector<Chromosome> Geneticalgorithm::getGenerationBestSolutions() {
	return generationBestSolutions;
}

//returns offsprings of every generation
std::vector<Chromosome> Geneticalgorithm::getGenerationalOffsprings() {
	return generationalOffsprings;
}

//returns ga solution
Chromosome Geneticalgorithm::getGASolution() {
	return bestSolution;
}

//runs ga algorithm
void Geneticalgorithm::runGeneticAlgorithm() {
	auto startga = high_resolution_clock::now();
	using std::chrono::duration;
	if(numberOfNodes > 3){
		//run ga
		//std::cout << "\nInitial generation is being populated" << std::endl;
		populateInitialGeneration();
		initialSolution = ppl.getBestChromosome();
		//std::cout << "\nShow initial solution" << std::endl;
		//initialSolution.showChromosome();
		incumbentSolution = ppl.getBestChromosome();
		bestSolution = ppl.getBestChromosome();
		//std::cout << "\nThe best solution in the first generation is : " << std::endl;
		//bestSolution.showChromosome();
		int numNonImprovingIterLimit = 10;
		int counter = 0;
		double prevCost = INFINITY;
		double currentCost = 0;
		int iter = 0;
		int diversitySize = 0;
		for (int i = 0; i < maxIterations; ++i) {
			iter++;
			diversitySize = ppl.getDiversitySize();
			if (diversitySize <= 5) {
				numNonImprovingIterLimit = 5;
			}
			if (diversitySize >= 2) {
				//std::cout << "\nPerform crossover" << std::endl;
				ppl.performCrossOver();
				//std::cout << "\nCrossover is done, perform mutation" << std::endl;
				ppl.performMutation();
				double tsTime = ppl.returnTSRunningTime();
				cumulativeTSRunningTime += tsTime;
				Chromosome crm = ppl.getOffspring();
				generationalOffsprings.push_back(crm);
				ppl.updatePopulationWithOffspring();
				Chromosome crom = ppl.getBestChromosome();
				generationBestSolutions.push_back(crom);
				if (crom.getFitness() < incumbentSolution.getFitness()) {
					incumbentSolution = crom;
				}
				currentCost = incumbentSolution.getFitness();
				if (currentCost < prevCost) {
					prevCost = currentCost;
					counter = 0;
				}
				else {
					counter++;
				}
				if (counter >= numNonImprovingIterLimit) {
					break;
				}
			}
			else {
				break;
			}
		}	
	}
	else {
		std::vector<int> sol;
		sol.push_back(depotNode);
		for (auto it: customerCluster) {
			if (it != depotNode) {
				sol.push_back(it);
			}
		}
		sol.push_back(depotNode);
		double cost = 0;
		for (int i = 0; i < sol.size()-1; i++) {
			cost += distance[sol.at(i)][sol.at(i + 1)];
		}
		Chromosome crm(sol, cost, depotNode, true, true, capacityLimit, maxPathLength);
		incumbentSolution = crm;
		cumulativeTSRunningTime = cumulativeTSRunningTime;
	}
	
	bestSolution = incumbentSolution;
	auto stopga = high_resolution_clock::now();
	duration<double, std::milli> ms_double = stopga - startga;
	//std::cout << "\nThe initial solution is : " << std::endl;
	//initialSolution.showChromosome();
	//std::cout << "\nThe best solution is : " << std::endl;
	//incumbentSolution.showChromosome();
	GARunningTime = double(ms_double.count()) / 1000;
	//std::cout << "\nGenetic Algorithm Running Time : "<< GARunningTime << " seconds" << std::endl;
	//std::cout << "\nCumulative Tabu Search Running Time : " << cumulativeTSRunningTime << " seconds" << std::endl;
	//std::cout << "\nThe number of Genetic Algorithm iterations : " << iter << ";" << std::endl;
	//ppl.clearPopulation();
}

//prints ga solution
void Geneticalgorithm::showGASolution() {
	std::cout << "\nThe GA best solution is : " << std::endl;
	incumbentSolution.showChromosome();
	/*
	std::cout << "\nShow other generation best GA solutions." << std::endl;
	int counter = 0;
	for (auto & it:generationBestSolutions) {
		counter++;
		std::cout << "\nSolution no : " << counter << std::endl;
		it.showChromosome();
	}
	*/
}

//prints current generations chromosomes
void Geneticalgorithm::showCurrentGeneration() {
	ppl.showPopulation();
}


double Geneticalgorithm::returnGATime() {
	return GARunningTime;
}


double Geneticalgorithm::returnCumulativeTSTime() {
	return cumulativeTSRunningTime;
}

std::list<Chromosome> Geneticalgorithm::getLastPopulation() {
	return ppl.getPopulation();
}