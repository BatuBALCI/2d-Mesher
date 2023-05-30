#include "Domain.h"
//------------------------------------------------------ Corner -----------------------------------------------------------
MeshData::Domain::Corner::Corner(double x, double y, double ksi, double eta)
{
	this->m_Coordinates.xCoord = x;
	this->m_Coordinates.yCoord = y;

	this->m_CornerVertex = std::make_shared<DoublyConnectedList::Vertex>(ksi, eta);
}
std::shared_ptr<DoublyConnectedList::Vertex> MeshData::Domain::Corner::getCornerVertex() const { return this->m_CornerVertex; }
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::Corner::getCoordinates() const { return this->m_Coordinates; }


//------------------------------------------------------ Edge -------------------------------------------------------------
MeshData::Domain::Edge::Edge(std::shared_ptr<MeshData::Domain::Corner> start,
	std::shared_ptr<MeshData::Domain::Corner> end) : m_StartCorner(start), m_EndCorner(end)
{
	calculateLength();
}
std::shared_ptr<MeshData::Domain::Corner> MeshData::Domain::Edge::getStartCorner() const { return this->m_StartCorner; }
std::shared_ptr<MeshData::Domain::Corner> MeshData::Domain::Edge::getEndCorner() const { return this->m_EndCorner; }
void MeshData::Domain::Edge::addEdgeConstraint(std::shared_ptr<MeshData::Domain::EdgeConstraint> edgeConstraint) { this->m_EdgeConstraint = edgeConstraint; }
std::shared_ptr<MeshData::Domain::EdgeConstraint> MeshData::Domain::Edge::getEdgeConstraint() const { return this->m_EdgeConstraint; }
double MeshData::Domain::Edge::getLength() const { return this->m_Length; }
double MeshData::Domain::Edge::calculateLength() { this->m_Length = sqrt(pow((m_EndCorner->getCoordinates().xCoord - m_StartCorner->getCoordinates().xCoord), 2)
	+ pow((m_EndCorner->getCoordinates().yCoord - m_StartCorner->getCoordinates().yCoord), 2)); return getLength(); }

//------------------------------------------------ Point Constriant -------------------------------------------------------
MeshData::Domain::PointConstraint::PointConstraint(double x, double y)
{
	this->m_Coordinates.xCoord = x;
	this->m_Coordinates.yCoord = y;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::PointConstraint::getCoordinates() const { return this->m_Coordinates; }

//------------------------------------------------ Line Constriant --------------------------------------------------------
MeshData::Domain::LineConstraint::LineConstraint(double startX, double startY, double endX, double endY)
{
	this->m_StartCoord.xCoord = startX;
	this->m_EndCoord.xCoord = endX;
	this->m_StartCoord.yCoord = startY;
	this->m_EndCoord.yCoord = endY;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::LineConstraint::getStartCoordinates() const { return this->m_StartCoord; }
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::LineConstraint::getEndCoordinates() const { return this->m_EndCoord; }

//------------------------------------------------ Edge Constraint --------------------------------------------------------
MeshData::Domain::EdgeConstraint::EdgeConstraint(const std::vector<double>& realtiveLocations) : m_RelativeConstraintLocations(realtiveLocations) {}
void MeshData::Domain::EdgeConstraint::addRelativeConstraintLocations(double relativeConstraintLocation)
{ this->m_RelativeConstraintLocations.push_back(relativeConstraintLocation); }
const std::vector<double>& MeshData::Domain::EdgeConstraint::getRelativeConstraintLocations() const { return this->m_RelativeConstraintLocations; }

//------------------------------------------------------ Domain -----------------------------------------------------------
void MeshData::Domain::addCorner(std::shared_ptr<Corner> corner) { this->m_Corners.push_back(corner); }
void MeshData::Domain::addEdge(std::shared_ptr<Edge> edge) { this->m_Edges.push_back(edge); }
void MeshData::Domain::addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint) { this->m_PointConstraints.push_back(pointConstraint); }
void MeshData::Domain::addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint) { this->m_LineConstraints.push_back(lineConstraint); }
void MeshData::Domain::setEdgeLength(double length) { this->m_EdgeLength = length; }
void MeshData::Domain::setAspectRaito(double aspectRaito) { this->m_AspectRatio = aspectRaito; }
void MeshData::Domain::generateDCEL(const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& vertexInput, const std::vector<std::vector<int>>& edgeInput) { this->m_DCEL = std::make_shared<DoublyConnectedList::DCEL>(vertexInput, edgeInput); }
const std::vector<std::shared_ptr<MeshData::Domain::Corner>>& MeshData::Domain::getCorners() const { return this->m_Corners; }
const std::vector<std::shared_ptr<MeshData::Domain::Edge>>& MeshData::Domain::getEdges() const { return this->m_Edges; }
const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& MeshData::Domain::getPointConstriants() const { return this->m_PointConstraints; }
const std::vector<std::shared_ptr<MeshData::Domain::LineConstraint>>& MeshData::Domain::getLineConstraints() const { return this->m_LineConstraints; }
std::shared_ptr<DoublyConnectedList::DCEL> MeshData::Domain::getDCEL() { return m_DCEL; }
double MeshData::Domain::getEdgeLength() const { return this->m_EdgeLength; }
double MeshData::Domain::getAspectRaito() const { return this->m_AspectRatio; }
