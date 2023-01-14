#include "Domain.h"

class BasicQuadMesh
{
public:
    BasicQuadMesh() = default;

    void Mesh(std::vector<MeshData::Domain>& domains);

private:
    void BuildInitialGrid(std::vector<MeshData::Domain>& domains);
    double FindOptimumEdgeLength(std::vector<MeshData::Domain>& domains);
    void AddLineConstraint(std::vector<MeshData::Domain>& domains);
    void ReturnToCartesian(std::vector<MeshData::Domain>& domains);
    void AddPointConstraintDomains(std::vector<MeshData::Domain>& domains);
    void AddPointConstraint(MeshData::Domain& domain, const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& constrains);
    bool CheckQualityOfMesh(std::vector<MeshData::Domain>& domains);
    bool TryMesh(std::vector<MeshData::Domain>& domains, int tryCount);

    // Some utility function for geometric calculations.
    DoublyConnectedList::Vertex::Coordinates RayIntersection(const DoublyConnectedList::Vertex::Coordinates& origin1, const DoublyConnectedList::Vertex::Coordinates& direction1,
        const DoublyConnectedList::Vertex::Coordinates& origin2, const DoublyConnectedList::Vertex::Coordinates& direction2);

    double m_OptimumLength;
};