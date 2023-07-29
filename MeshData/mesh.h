#pragma once
#include "cmath"
#include <numeric>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/maximum_weighted_matching.hpp> 

#include "Domain.h"

struct VertexProps { std::shared_ptr<DoublyConnectedList::Face> face; };
typedef boost::property<boost::edge_weight_t, float> EdgeProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProps, EdgeProperty> my_graph;
typedef boost::graph_traits<my_graph >::vertex_descriptor vertex_t;
typedef boost::graph_traits<my_graph>::edge_descriptor edge_t;

class BasicQuadMesh
{
public:
    BasicQuadMesh() = default;

    void Mesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    void SetMaxQuadAngle(double maxQuadAngle);
    void SetStdDeviationFunc(std::function<double(double)> stdDeviationFunction);

private:
    void BuildInitialGrid(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    bool TryMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains, int tryCount);
    void FindOptimumEdgeLength(const std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    void ReturnToCartesian(std::shared_ptr<MeshData::Domain> domain);
    bool AddConstraint(std::shared_ptr<MeshData::Domain> domain, const DoublyConnectedList::Vertex::Coordinates& constrainCoordinate, DoublyConnectedList::Vertex::Constraints type);
    bool AddPointConstraints(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    bool AddLineConstraint(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    void TriangleToQuad(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    bool CheckQualityOfMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    my_graph GenerateGraph(std::shared_ptr<MeshData::Domain> domain);
    void ClearDomain(std::vector<std::shared_ptr<MeshData::Domain>>& domains);

    // Some utility function for geometric calculations.
    DoublyConnectedList::Vertex::Coordinates RayIntersection(const DoublyConnectedList::Vertex::Coordinates& origin1, DoublyConnectedList::Vertex::Coordinates& direction1,
        const DoublyConnectedList::Vertex::Coordinates& origin2, DoublyConnectedList::Vertex::Coordinates& direction2);
    DoublyConnectedList::Vertex::Coordinates MapFromNeutralCoordinate(double neutralKsi, double neutralEta, const std::vector<std::pair<double, double>>& nodeLocation, std::function<std::vector<double>(double, double)> shapeFunction);
    double ApproximateGCD(const std::vector<double>& nums);
    double ApproximateLCM(const std::vector<double>& nums);
    double StandardDeviation(const std::vector<double> nums);

    // this is the vector that holds numbers that say how many lines that edge 1-3 and 2-4 will going to have on it respectively.; 
    std::vector<std::pair<int, int>> m_EdgeDivideNum;
    double m_OptimumLength;
    double m_MaxQuadAngle = 145.0;
    std::function<double(double)> stdDeviationFunction = [](double stdDev) {return 156.0 - stdDev; };
};