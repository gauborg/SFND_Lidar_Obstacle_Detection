
#include "cluster.h"

// recursive function
void clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed_points, KdTree *tree, float distanceTol)
{
	processed_points[index] = true;
	cluster.push_back(index);

	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for (int id : nearest)
	{
		if (!processed_points[id])
			clusterHelper(id, points, cluster, processed_points, tree, distanceTol);
	}
}



std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed_points(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed_points[i])
		{
			i++;
			continue;
		}
		else
		{
			std::vector<int> cluster;
			clusterHelper(i, points, cluster, processed_points, tree, distanceTol);
			clusters.push_back(cluster);
			i++;
		}
	}

	return clusters;
}