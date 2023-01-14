#include "Domain.h"
//------------------------------------------------------ Corner -----------------------------------------------------------
MeshData::Domain::Corner::Corner(double x, double y)
{
	this->m_Coordinates.xCoord = x;
	this->m_Coordinates.yCoord = y;

	this->m_CornerVertex = std::make_shared<DoublyConnectedList::Vertex>(x, y);
}
std::shared_ptr<DoublyConnectedList::Vertex> MeshData::Domain::Corner::getCornerVertex() { return this->m_CornerVertex; }
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::Corner::getCoordinates() { return this->m_Coordinates; }


//------------------------------------------------------ Edge -------------------------------------------------------------
MeshData::Domain::Edge::Edge(std::shared_ptr<MeshData::Domain::Corner> start,
	std::shared_ptr<MeshData::Domain::Corner> end) : m_StartCorner(start), m_EndCorner(end)
{
	calculateLength();
}
std::shared_ptr<MeshData::Domain::Corner> MeshData::Domain::Edge::getStartCorner() { return this->m_StartCorner; }
std::shared_ptr<MeshData::Domain::Corner> MeshData::Domain::Edge::getEndCorner() { return this->m_EndCorner; }
double MeshData::Domain::Edge::getLength() { return this->m_Length; }
double MeshData::Domain::Edge::calculateLength() { this->m_Length = sqrt(pow((m_EndCorner->getCoordinates().xCoord - m_StartCorner->getCoordinates().xCoord), 2)
	+ pow((m_EndCorner->getCoordinates().yCoord - m_StartCorner->getCoordinates().yCoord), 2)); return getLength(); }

//------------------------------------------------ Point Constriant -------------------------------------------------------
MeshData::Domain::PointConstraint::PointConstraint(double x, double y)
{
	this->m_Coordinates.xCoord = x;
	this->m_Coordinates.yCoord = y;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::PointConstraint::getCoordinates() { return this->m_Coordinates; }

//------------------------------------------------ Line Constriant --------------------------------------------------------
MeshData::Domain::LineConstraint::LineConstraint(double startX, double startY, double endX, double endY)
{
	this->m_StartCoord.xCoord = startX;
	this->m_EndCoord.xCoord = endX;
	this->m_StartCoord.yCoord = startY;
	this->m_EndCoord.yCoord = endY;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::LineConstraint::getStartCoordinates() { return this->m_StartCoord; }
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::LineConstraint::getEndCoordinates() { return this->m_EndCoord; }
void MeshData::Domain::LineConstraint::addPointConstraint(std::shared_ptr<PointConstraint> pointConstraint) 
{ this->m_PointConstraints.push_back(pointConstraint); }
const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& 
MeshData::Domain::LineConstraint::getConstraints() { return this->m_PointConstraints; }

//------------------------------------------------ Edge Constraint --------------------------------------------------------
MeshData::Domain::EdgeConstraint::EdgeConstraint(std::shared_ptr<MeshData::Domain::Edge> edge) : m_Edge(edge) {}
std::shared_ptr<MeshData::Domain::Edge> MeshData::Domain::EdgeConstraint::getEdge() { return this->m_Edge; }
void MeshData::Domain::EdgeConstraint::addRelativeConstraintLocations(double relativeConstraintLocation)
{ this->m_RelativeConstraintLocations.push_back(relativeConstraintLocation); }
const std::vector<double>& MeshData::Domain::EdgeConstraint::getRelativeConstraintLocations() { return this->m_RelativeConstraintLocations; }

//------------------------------------------------------ Domain -----------------------------------------------------------
void MeshData::Domain::addCorner(std::shared_ptr<Corner> corner) { corner->getCornerVertex()->setID(m_Corners.size()); this->m_Corners.push_back(corner); }
void MeshData::Domain::addEdge(std::shared_ptr<Edge> edge) { this->m_Edges.push_back(edge); }
void MeshData::Domain::addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint) { this->m_PointConstraints.push_back(pointConstraint); }
void MeshData::Domain::addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint) { this->m_LineConstraints.push_back(lineConstraint); }
void MeshData::Domain::addEdgeConstraint(std::shared_ptr<EdgeConstraint> edgeConstraint) { this->m_EdgeConstraints.push_back(edgeConstraint); }
const std::vector<std::shared_ptr<MeshData::Domain::Corner>>& MeshData::Domain::getCorners() { return this->m_Corners; }
const std::vector<std::shared_ptr<MeshData::Domain::Edge>>& MeshData::Domain::getEdges() { return this->m_Edges; }
const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& MeshData::Domain::getPointConstriants() { return this->m_PointConstraints; }
const std::vector<std::shared_ptr<MeshData::Domain::LineConstraint>>& MeshData::Domain::getLineConstraints() { return this->m_LineConstraints; }
const std::vector<std::shared_ptr<MeshData::Domain::EdgeConstraint>>& MeshData::Domain::getEdgeConstraints() { return this->m_EdgeConstraints; }
std::shared_ptr<DoublyConnectedList::DCEL> MeshData::Domain::getDCEL() { return m_DCEL; }
