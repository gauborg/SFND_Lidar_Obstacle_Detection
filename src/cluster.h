
#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <string>
#include "kdtree.h"

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol);

#endif /* CLUSTER_H_ */