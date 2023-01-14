#include "Mesh.h"

void BasicQuadMesh::Mesh(std::vector<MeshData::Domain>& domains)
{
	for (int i = 0; i < 5; i++)
		if (this->TryMesh(domains, i))
			break;
}
bool BasicQuadMesh::TryMesh(std::vector<MeshData::Domain>& domains, int tryCount)
{
	if (tryCount == 0)
		this->m_OptimumLength = this->FindOptimumEdgeLength(domains);
	else
		this->m_OptimumLength /= 2;

	this->BuildInitialGrid(domains);
	this->ReturnToCartesian(domains);
	this->AddPointConstraintDomains(domains);
	this->AddLineConstraint(domains);

	return this->CheckQualityOfMesh(domains);
}
void BasicQuadMesh::BuildInitialGrid(std::vector<MeshData::Domain>& domains)
{

}
void BasicQuadMesh::AddPointConstraintDomains(std::vector<MeshData::Domain>& domains)
{
	for (auto& domain : domains)
		this->AddPointConstraint(domain, domain.getPointConstriants());
}
void BasicQuadMesh::AddPointConstraint(MeshData::Domain& domain, const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& constrains)
{
	for (auto constraint : constrains) {
	
	}
}
void BasicQuadMesh::AddLineConstraint(std::vector<MeshData::Domain>& domains)
{
	for (auto& domain : domains)
		for (auto& lineConstraint : domain.getLineConstraints())
			this->AddPointConstraint(domain, lineConstraint->getConstraints());
}
void BasicQuadMesh::ReturnToCartesian(std::vector<MeshData::Domain>& domains)
{

}
double BasicQuadMesh::FindOptimumEdgeLength(std::vector<MeshData::Domain>& domains)
{
	for (auto& domain : domains)
	{

	}
	return 0.0;
}
bool BasicQuadMesh::CheckQualityOfMesh(std::vector<MeshData::Domain>& domains)
{
	for (auto& domain : domains)
	{

	}
	return true;
}

DoublyConnectedList::Vertex::Coordinates BasicQuadMesh::RayIntersection(const DoublyConnectedList::Vertex::Coordinates& origin1, const DoublyConnectedList::Vertex::Coordinates& direction1,
	const DoublyConnectedList::Vertex::Coordinates& origin2, const DoublyConnectedList::Vertex::Coordinates& direction2)
{
	// normalize the direction vectors
	auto dir1 = direction1 / (sqrt(direction1.xCoord * direction1.xCoord + direction1.yCoord * direction1.yCoord));
	auto dir2 = direction2 / (sqrt(direction2.xCoord * direction2.xCoord + direction2.yCoord * direction2.yCoord));
	// Caulculate the distance between the intersection point and the origin 2 by solving the intersection
	// equation between 2 rays.
	auto distanceBetweenTheIntersectionAndOrigin2 = (origin1.yCoord + dir1.yCoord * (origin2.xCoord - origin1.xCoord) / dir1.xCoord - origin2.yCoord) /
		(dir2.yCoord - dir2.xCoord * dir1.yCoord / dir1.xCoord);

	auto result = origin2 + dir2 * distanceBetweenTheIntersectionAndOrigin2;
	return result;
}
