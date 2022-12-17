#include "mesh.h"
//------------------------------------------------ Point Constriant -------------------------------------------------------
MeshData::Domain::PointConstraint::PointConstraint(double x, double y)
{
	this->coordinates.xCoord = x;
	this->coordinates.yCoord = y;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::PointConstraint::getCoordinates() { return this->coordinates; }

//------------------------------------------------ Line Constriant --------------------------------------------------------
MeshData::Domain::LineConstraint::LineConstraint(double startX, double startY, double endX, double endY)
{
	this->startCoord.xCoord = startX;
	this->endCoord.xCoord = endX;
	this->startCoord.yCoord = startY;
	this->endCoord.yCoord = endY;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::LineConstraint::getStartCoordinates() { return this->startCoord; }
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::LineConstraint::getEndCoordinates() { return this->endCoord; }
void MeshData::Domain::LineConstraint::addPointConstraint(std::shared_ptr<PointConstraint> pointConstraint) 
{ this->pointConstraints.push_back(pointConstraint); }
const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& 
MeshData::Domain::LineConstraint::getConstraints() { return this->pointConstraints; }

//------------------------------------------------------ Corner -----------------------------------------------------------
MeshData::Domain::Corner::Corner(double x, double y)
{
	this->coordinates.xCoord = x;
	this->coordinates.yCoord = y;
}
const DoublyConnectedList::Vertex::Coordinates& MeshData::Domain::Corner::getCoordinates() { return this->coordinates; }

//------------------------------------------------------ Domain -----------------------------------------------------------
void MeshData::Domain::addCorner(std::shared_ptr<Corner> corner) { this->corners.push_back(corner); }
void MeshData::Domain::addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint) { this->pointConstraints.push_back(pointConstraint); }
void MeshData::Domain::addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint) { this->lineConstraints.push_back(lineConstraint); }
const std::vector<std::shared_ptr<MeshData::Domain::Corner>>& MeshData::Domain::getCorners() { return this->corners; }
const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& MeshData::Domain::getPointConstriants() { return this->pointConstraints; }
const std::vector<std::shared_ptr<MeshData::Domain::LineConstraint>>& MeshData::Domain::getLineConstraints() { return this->lineConstraints; }
std::shared_ptr<DoublyConnectedList::DCEL> MeshData::Domain::getDCEL() { return DCEL; }
