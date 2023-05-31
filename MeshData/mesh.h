#pragma once
#include "Domain.h"
#include "cmath"
#include <boost/math/common_factor_ct.hpp>
#include <boost/math/common_factor_rt.hpp>

class BasicQuadMesh
{
public:
    BasicQuadMesh() = default;

    void Mesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains);

private:
    void BuildInitialGrid(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    void FindOptimumEdgeLength(const std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    void ReturnToCartesian(std::shared_ptr<MeshData::Domain> domain);
    bool AddConstraint(std::shared_ptr<MeshData::Domain> domain, const DoublyConnectedList::Vertex::Coordinates& constrainCoordinate, DoublyConnectedList::Vertex::Constraints type);
    bool AddPointConstraints(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    bool AddLineConstraint(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    bool CheckQualityOfMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains);
    bool TryMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains, int tryCount);

    void ClearDomain(std::vector<std::shared_ptr<MeshData::Domain>>& domains);

    // Some utility function for geometric calculations.
    DoublyConnectedList::Vertex::Coordinates RayIntersection(const DoublyConnectedList::Vertex::Coordinates& origin1, DoublyConnectedList::Vertex::Coordinates& direction1,
        const DoublyConnectedList::Vertex::Coordinates& origin2, DoublyConnectedList::Vertex::Coordinates& direction2);
    DoublyConnectedList::Vertex::Coordinates MapFromNeutralCoordinate(double neutralKsi, double neutralEta, const std::vector<std::pair<double, double>>& nodeLocation, std::function<std::vector<double>(double, double)> shapeFunction);
    double ApproximateGCD(const std::vector<double>& nums);
    double ApproximateLCM(const std::vector<double>& nums);


    // this is the vector that holds numbers that say how many lines that edge 1-3 and 2-4 will going to have on it respectively.; 
    std::vector<std::pair<int, int>> m_EdgeDivideNum;
    double m_OptimumLength;
};