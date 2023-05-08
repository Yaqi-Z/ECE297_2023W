#ifndef MAPPER_M4_EXTEND_H
#define MAPPER_M4_EXTEND_H

#include <unordered_map>
#include "m4.h"
#include <cstdlib>

bool LoadMatrix(std::vector<double>&travelTimeMatrix, std::vector<CourierSubPath>& subPathMatrix,
                int matrixSize, const std::vector<IntersectionIdx>& inters, const float turn_penalty);

bool MultiDestDijkstraSearch(
        std::vector<myNode>& myNodesVec, const IntersectionIdx src,
        const std::vector<IntersectionIdx>& dest, const double turn_penalty);

bool pathOptimizer(std::vector<int>& bestPath, double& bestTime,
                   const std::vector<DeliveryInf>& deliveries,
                   const std::vector<double>& travelTimeMatrix, int matrixSize,
                   const std::multimap<IntersectionIdx,int>& intersectionsToIndex,
                   double weight[], int size);
int weightedRandom(int n, double weights[]);
std::vector<int> partialShuffle(std::vector<int> path, bool reverse_or_random);
bool nodeSwap(std::vector<int>& path, double& time, const std::vector<double>& travelTimeMatrix, int matrixSize);
std::vector<int> segmentShift(std::vector<int> path, bool shiftOneElement, bool rotateLeft);

double timeFromPath(const std::vector<int>& path, const std::vector<DeliveryInf>& deliveries,
                    const std::vector<double>& travelTimeMatrix, int matrixSize,
                    const std::multimap<IntersectionIdx,int>& intersectionsToIndex);
std::vector<int> Simulated_Annealing(std::vector<int> initPath, double initTime, 
                                     const std::vector<DeliveryInf>& deliveries, std::vector<double>& travelTimeMatrix, 
                                     int matrixSize, const std::multimap<IntersectionIdx,int>& intersectionsToIndex,
                                     std::chrono::high_resolution_clock::time_point StartTime);
std::vector<CourierSubPath> resultFromPath(const std::vector<int>& path, int numOfDeliv, int numOfDepot,
                                           const std::vector<double>& travelTimeMatrix,
                                           const std::vector<CourierSubPath>& subPathMatrix);

#endif //MAPPER_M4_EXTEND_H
