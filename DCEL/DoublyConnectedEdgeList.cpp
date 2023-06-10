#include "DoublyConnectedEdgeList.h"
// -------------------------------------------------VERTEX----------------------------------------------
DoublyConnectedList::Vertex::Vertex(double x, double y) 
{
	this->m_Coordinates.xCoord = x;
	this->m_Coordinates.yCoord = y;
	this->m_ID = -1;
}
DoublyConnectedList::Vertex::Vertex(double x, double y, int ID)
{
	this->m_Coordinates.xCoord = x;
	this->m_Coordinates.yCoord = y;
	this->m_ID = ID;
}
void DoublyConnectedList::Vertex::setID(int ID) { this->m_ID = ID; }
void DoublyConnectedList::Vertex::setConstraint(Constraints cons) { this->m_ConstraintType = cons; }
void DoublyConnectedList::Vertex::clearConstraint() { this->m_ConstraintType = Constraints::NONE; }
DoublyConnectedList::Vertex::Constraints DoublyConnectedList::Vertex::constriantType() { return m_ConstraintType; }
const std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>& DoublyConnectedList::Vertex::getHalfEdge() const { return this->m_HalfEdges; }
void DoublyConnectedList::Vertex::clearHalfEdge() { this->m_HalfEdges.clear(); }
int DoublyConnectedList::Vertex::getID() const { return this->m_ID; }
void DoublyConnectedList::Vertex::addHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) { this->m_HalfEdges.push_back(halfEdge); };
void DoublyConnectedList::Vertex::angleSortEdges()
{
	auto sort = [&](std::shared_ptr<DoublyConnectedList::HalfEdge> halfEgde1, std::shared_ptr<DoublyConnectedList::HalfEdge> halfEgde2)
	{
		return halfEgde1->getAngle() < halfEgde2->getAngle();
	};
	std::sort(this->m_HalfEdges.begin(), this->m_HalfEdges.end(), sort);
}
void DoublyConnectedList::Vertex::updateConnections() {
	for (auto hEdge : this->getHalfEdge()) {
		// Update face and half edge properties.
		hEdge->getIncidentFace()->updateProperties();
	}
	// Sort half edges
	this->angleSortEdges();
}
void DoublyConnectedList::Vertex::prevAndNextAssignments()
{
	this->angleSortEdges();
	auto vEdgeSize = this->getHalfEdge().size();
	if (vEdgeSize < 2)
		throw;
	else
	{
		for (int i = 0; i < vEdgeSize - 1; i++)
		{
			this->getHalfEdge()[i + 1]->getTwinHalfEdge()->setNextHalfEdge(this->getHalfEdge()[i]);
			this->getHalfEdge()[i]->setPrevHalfEdge(this->getHalfEdge()[i + 1]->getTwinHalfEdge());
		}
		this->getHalfEdge()[0]->getTwinHalfEdge()->setNextHalfEdge(this->getHalfEdge()[vEdgeSize - 1]);
		this->getHalfEdge()[vEdgeSize - 1]->setPrevHalfEdge(this->getHalfEdge()[0]->getTwinHalfEdge());
	}
}
void DoublyConnectedList::Vertex::deleteEdgeFromList(std::shared_ptr<DoublyConnectedList::HalfEdge> edge)
{
	int counter = 0;
	for (auto& halfEdge : this->m_HalfEdges) {
		if (halfEdge == edge)
			break;
		counter++;
	}
	this->m_HalfEdges.erase(m_HalfEdges.begin() + counter, m_HalfEdges.begin() + counter + 1);
}
void DoublyConnectedList::Vertex::setCoordinate(DoublyConnectedList::Vertex::Coordinates coordinate)
{
	this->m_Coordinates = coordinate;
}
const DoublyConnectedList::Vertex::Coordinates& DoublyConnectedList::Vertex::getCoordinate() const { return this->m_Coordinates;}

// -------------------------------------------------HALF-EDGE-------------------------------------------
void DoublyConnectedList::HalfEdge::calculateProperties(std::shared_ptr<DoublyConnectedList::Vertex> point1,
	std::shared_ptr<DoublyConnectedList::Vertex> point2)
{
	double dx = point2->getCoordinate().xCoord - point1->getCoordinate().xCoord,
		dy = point2->getCoordinate().yCoord - point1->getCoordinate().yCoord;
	this->m_Length = sqrt(pow(dx, 2) + pow(dy, 2));
	if (dy >= 0)
		this->m_Angle = acos(dx / this->m_Length);
	else
		this->m_Angle = 2 * PI - acos(dx / this->m_Length);
}
DoublyConnectedList::HalfEdge::HalfEdge(std::shared_ptr<DoublyConnectedList::Vertex> point1,
	std::shared_ptr<DoublyConnectedList::Vertex> point2)
{
	this->m_Origin = point1;
	calculateProperties(point1, point2);
}
void DoublyConnectedList::HalfEdge::setNextHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> next) { this->m_Next = next; }
void DoublyConnectedList::HalfEdge::setPrevHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> prev) { this->m_Prev = prev; }
void DoublyConnectedList::HalfEdge::setTwinHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> twin) { this->m_Twin = twin; }
void DoublyConnectedList::HalfEdge::setIncidentFace(std::shared_ptr<DoublyConnectedList::Face> incidentFace) { this->m_IncidentFace = incidentFace; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::HalfEdge::getNextHalfEdge() const { return this->m_Next; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::HalfEdge::getPrevHalfEdge() const { return this->m_Prev; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::HalfEdge::getTwinHalfEdge() const { return this->m_Twin; }
std::shared_ptr<DoublyConnectedList::Face> DoublyConnectedList::HalfEdge::getIncidentFace() const { return this->m_IncidentFace; }
std::shared_ptr<DoublyConnectedList::Vertex> DoublyConnectedList::HalfEdge::getOrigin() const { return this->m_Origin; }
double DoublyConnectedList::HalfEdge::getAngle() const { return this->m_Angle; }
double DoublyConnectedList::HalfEdge::getLength() const { return this->m_Length; }
void DoublyConnectedList::HalfEdge::updateProperties() {
	auto origin = this->getOrigin();
	auto twinOrigin = this->getTwinHalfEdge()->getOrigin();
	this->calculateProperties(origin, twinOrigin);
}
double DoublyConnectedList::HalfEdge::determinant(const std::vector<double>& point1, const std::vector<double>& point2){ return (point1[0]) * (point2[1]) - (point1[1]) * (point2[0]); }
bool DoublyConnectedList::HalfEdge::isPointOnLeft(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates)
{
	auto twinOrgCoor = this->m_Twin->getOrigin()->getCoordinate();
	auto orgCoor = this->m_Origin->getCoordinate();
	auto area = determinant({ (pointCoordinates.xCoord - orgCoor.xCoord) ,(pointCoordinates.yCoord - orgCoor.yCoord) },
		{ (twinOrgCoor.xCoord - orgCoor.xCoord) ,  (twinOrgCoor.yCoord - orgCoor.yCoord) });
	return area < 0.0;
}
bool DoublyConnectedList::HalfEdge::isPointOn(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates)
{
	auto twinOrgCoor = this->m_Twin->getOrigin()->getCoordinate();
	auto orgCoor = this->m_Origin->getCoordinate();
	auto area = determinant({ (pointCoordinates.xCoord - orgCoor.xCoord) ,(pointCoordinates.yCoord - orgCoor.yCoord) },
		{ (twinOrgCoor.xCoord - orgCoor.xCoord) ,  (twinOrgCoor.yCoord - orgCoor.yCoord) });
	auto pointToOriginLength = sqrt(pow((pointCoordinates.xCoord - this->m_Origin->getCoordinate().xCoord), 2) +
		pow((pointCoordinates.yCoord - this->m_Origin->getCoordinate().yCoord), 2));
	// Get the dot product of two vectors: HalfEdge and the vector between the given point and HalfEdge origin
	auto vec1 = (twinOrgCoor - orgCoor).normalize();
	auto vec2 = (pointCoordinates - orgCoor);
	if (vec2 * vec2 < 0.000001)
		return true;
	vec2 = vec2.normalize();

	return (abs(area) < 0.000001 && pointToOriginLength < this->m_Length && (vec1 * vec2) > 0.0);
}
std::shared_ptr<DoublyConnectedList::Face> DoublyConnectedList::HalfEdge::faceAssignmentToEdges(std::shared_ptr<DoublyConnectedList::HalfEdge> itself)
{
	std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge = itself;
	auto face = std::make_shared<DoublyConnectedList::Face>();
	face->setHalfEdge(halfEdge);
	face->getHalfEdge()->setIncidentFace(face);
	while (halfEdge->getNextHalfEdge() != face->getHalfEdge())
	{
		halfEdge = halfEdge->getNextHalfEdge();
		halfEdge->setIncidentFace(face);
	}
	return face;
}

// ------------------------------------------------- Face ----------------------------------------------
double DoublyConnectedList::Face::calculateArea()
{
	if (this->m_Area > 0)
		return this->m_Area;
	auto halfEdge = this->m_HalfEdgeComponent;
	double area = 0.0;
	std::shared_ptr<DoublyConnectedList::Vertex> point1;
	std::shared_ptr<DoublyConnectedList::Vertex> point2;
	do
	{
		point1 = halfEdge->getOrigin();
		point2 = halfEdge->getNextHalfEdge()->getOrigin();
		area += point1->getCoordinate().xCoord * point2->getCoordinate().yCoord - point2->getCoordinate().xCoord * point1->getCoordinate().yCoord;
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	point1 = halfEdge->getOrigin();
	point2 = halfEdge->getNextHalfEdge()->getOrigin();
	this->m_Area = area/2.0;
	return this->m_Area;
}
double DoublyConnectedList::Face::calculatePerimeter()
{
	if (this->m_Perimeter > 0)
		return this->m_Perimeter;
	auto halfEdge = this->m_HalfEdgeComponent;
	double perimeter = 0.0;
	do 
	{
		perimeter += halfEdge->getLength();
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	this->m_Perimeter = perimeter;
	return this->m_Perimeter;
}
void DoublyConnectedList::Face::updateProperties() {
	auto halfEdge = this->getHalfEdge();
	do
	{
		halfEdge->updateProperties();
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	this->calculateArea();
	this->calculatePerimeter();
}

void DoublyConnectedList::Face::calculateEdgeCount()
{
	this->m_EdgeCount = 0;
	auto halfEdge = this->m_HalfEdgeComponent;
	do
	{
		halfEdge = halfEdge->getNextHalfEdge();
		this->m_EdgeCount++;
	} while (halfEdge != this->m_HalfEdgeComponent);
}
int DoublyConnectedList::Face::getEdgeCount() const { return this->m_EdgeCount; }
void DoublyConnectedList::Face::setHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) { this->m_HalfEdgeComponent = halfEdge; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::Face::getHalfEdge() const { return this->m_HalfEdgeComponent; }
bool DoublyConnectedList::Face::isInside(const DoublyConnectedList::Vertex::Coordinates& pointCooridnate)
{
	auto halfEdge = this->m_HalfEdgeComponent;
	do
	{
		if (!halfEdge->isPointOnLeft(pointCooridnate))
			return false;
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	return true;
}
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::Face::isOn(const DoublyConnectedList::Vertex::Coordinates& pointCooridnate)
{
	auto halfEdge = this->m_HalfEdgeComponent;
	do
	{
		if (halfEdge->isPointOn(pointCooridnate))
			return halfEdge;
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	return nullptr;
}
std::shared_ptr<DoublyConnectedList::Vertex> DoublyConnectedList::Face::getClosestPoint(const DoublyConnectedList::Vertex::Coordinates& pointCooridnate)
{
	double closestDistance = DBL_MAX;
	std::shared_ptr<DoublyConnectedList::Vertex> closestpoint;
	auto halfEdge = this->m_HalfEdgeComponent;
	do {
		auto origin = halfEdge->getOrigin();
		double xDifference = pointCooridnate.xCoord - origin->getCoordinate().xCoord;
		double yDifference = pointCooridnate.yCoord - origin->getCoordinate().yCoord;
		double distance = sqrt(pow(xDifference, 2) + pow(yDifference, 2));
		if (distance < closestDistance) {
			closestDistance = distance;
			closestpoint = origin;
		}
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	return closestpoint;
}
bool DoublyConnectedList::Face::isExternal() { return this->m_IsTheFaceExternal; }
void DoublyConnectedList::Face::setIfExternal(bool isExt) { this->m_IsTheFaceExternal = isExt; }
void DoublyConnectedList::Face::clearFaceAssignments()
{
	auto halfEdge = this->getHalfEdge();
	do
	{
		halfEdge->setIncidentFace(nullptr);
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
}

// ------------------------------------------------- DCEL ----------------------------------------------
DoublyConnectedList::DCEL::DCEL() : m_Vertices(std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>()),
m_HalfEdges(std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>()),
m_Faces(std::vector<std::shared_ptr<DoublyConnectedList::Face>>()) {}
DoublyConnectedList::DCEL::DCEL(const std::vector<std::pair<double, double>>& vertexInput, 
	const std::vector<std::pair<int, int>>& edgeInput) : m_Vertices(std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>()),
	m_HalfEdges(std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>()),
	m_Faces(std::vector<std::shared_ptr<DoublyConnectedList::Face>>()) 
{
	if (vertexInput.size() != 0)
		buildDCEL(vertexInput, edgeInput);
}
DoublyConnectedList::DCEL::DCEL(const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& vertexInput,
	const std::vector<std::pair<int, int>>& edgeInput) : m_Vertices(std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>()),
	m_HalfEdges(std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>()),
	m_Faces(std::vector<std::shared_ptr<DoublyConnectedList::Face>>())
{
	if (vertexInput.size() != 0)
		buildDCEL(vertexInput, edgeInput);
}
size_t DoublyConnectedList::DCEL::getNumberOfVertices() const { return this->m_Vertices.size(); }
size_t DoublyConnectedList::DCEL::getNumberOfHalfEdges() const { return this->m_HalfEdges.size(); }
size_t DoublyConnectedList::DCEL::getNumberOfFaces() const { return this->m_Faces.size(); }
const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& DoublyConnectedList::DCEL::getVertices() const { return this->m_Vertices; }
const std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>& DoublyConnectedList::DCEL::getHalfEdges() const { return this->m_HalfEdges; }
const std::vector<std::shared_ptr<DoublyConnectedList::Face>>& DoublyConnectedList::DCEL::getFaces() const { return this->m_Faces; }
void DoublyConnectedList::DCEL::updateVertexIds() { int counter = 0; for (auto& vertex : this->m_Vertices) vertex->setID(counter++); }
void DoublyConnectedList::DCEL::updateSystem() {
	for (auto face : this->getFaces())
		face->updateProperties();
}
void DoublyConnectedList::DCEL::addEdge(int vertId1, int vertId2)
{
	auto vertex1 = this->m_Vertices[vertId1];
	auto vertex2 = this->m_Vertices[vertId2];
	// Check whether this edge exist or not.
	// If it exists do not add new edge
	for (auto& halfEdge : vertex1->getHalfEdge())
		if (halfEdge->getTwinHalfEdge()->getOrigin() == vertex2)
			break;

	bool doesNewEdgeCrossAnEdge = false;
	// If the new edge doesn't cross an existing edge.
	if(!doesNewEdgeCrossAnEdge)
	{
		// Create Half-Edges
		auto edges = this->createHalfEdge(vertex1, vertex2);
		auto hEdge1 = edges.first;
		auto hEdge2 = edges.second;

		// Since the new edge is added re-run the
		// sort algorithm.
		vertex1->angleSortEdges();
		vertex2->angleSortEdges();

		// Find the new position of the edge
		// inside the half edges list inside the vertex
		int counter = 0;
		for (auto& halfEdge : vertex1->getHalfEdge())
		{
			if (halfEdge == hEdge1)
				break;
			counter++;
		}
		// Find the face that the edge crosses.
		if (counter == 0)
			counter = vertex1->getHalfEdge().size();
		auto oldFace = vertex1->getHalfEdge()[counter - 1]->getIncidentFace();
		oldFace->clearFaceAssignments();
		this->deleteFaceFromList(oldFace);
 
		// Assign new prevs and nexts for the edges 
		// around the two vertices.
		vertex1->prevAndNextAssignments();
		vertex2->prevAndNextAssignments();

		// Assign the new faces.
		auto newFace1 = hEdge1->faceAssignmentToEdges(hEdge1);
		auto newFace2 = hEdge2->faceAssignmentToEdges(hEdge2);
		newFace1->calculateArea();
		newFace1->calculateEdgeCount();
		newFace2->calculateArea();
		newFace2->calculateEdgeCount();
		this->m_Faces.push_back(newFace1);
		this->m_Faces.push_back(newFace2);
	}
	// TODO
	else
	{

	}
}
void DoublyConnectedList::DCEL::deleteEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> edge)
{
	// Get the old faces
	auto face1 = edge->getIncidentFace();
	auto face2 = edge->getTwinHalfEdge()->getIncidentFace();

	// Get the vertices which the edge lays between
	auto vertex1 = edge->getOrigin();
	auto vertex2 = edge->getTwinHalfEdge()->getOrigin();

	// Check whether the esge or its twin is facing
	// an external face (if not continue to delete process)
	if (edge->getIncidentFace()->isExternal() || edge->getTwinHalfEdge()->getIncidentFace()->isExternal())
		return;

	// Find the new position of the edge
	// inside the half edges list inside the vertex
	int counter = 0;
	for (auto& halfEdge : vertex1->getHalfEdge())
	{
		if (halfEdge == edge)
			break;
		counter++;
	}
	// Find the previous half-edge to use while creating new face.
	if (counter == 0)
		counter = vertex1->getHalfEdge().size();
	auto previousHalfEdge = vertex1->getHalfEdge()[counter - 1];

	// Delete the edges form the lists
	this->deleteEdgeFromList(edge);
	this->deleteEdgeFromList(edge->getTwinHalfEdge());
	vertex1->deleteEdgeFromList(edge);
	vertex2->deleteEdgeFromList(edge->getTwinHalfEdge());
	
	// Clear face assignments
	face1->clearFaceAssignments();
	face2->clearFaceAssignments();
	// Delete faces from the face list.
	this->deleteFaceFromList(face1);
	this->deleteFaceFromList(face2);

	// Assign new prevs and nexts for the edges 
	// around the two vertices.
	vertex1->prevAndNextAssignments();
	vertex2->prevAndNextAssignments();

	// Assign the new faces.
	auto newFace = previousHalfEdge->faceAssignmentToEdges(previousHalfEdge);
	newFace->calculateArea();
	newFace->calculateEdgeCount();
	this->m_Faces.push_back(newFace);

}
void DoublyConnectedList::DCEL::addVertex(double xCoord, double yCoord, std::vector<int> verticesToConnect)
{
	auto newVertex = std::make_shared<DoublyConnectedList::Vertex>(xCoord, yCoord);
	auto faceThatContainNewVertex = this->findPoint(newVertex->getCoordinate());

	this->m_Vertices.push_back(newVertex);
	this->updateVertexIds();

	auto halfEdge = faceThatContainNewVertex->isOn(newVertex->getCoordinate());
	if (halfEdge) {
		this->addVertexOnTheEdge(halfEdge, newVertex);
	}
	else {
		this->addVertexInsideTheFace(faceThatContainNewVertex, newVertex, verticesToConnect);
	}
}
void DoublyConnectedList::DCEL::addVertex(std::shared_ptr<DoublyConnectedList::Vertex> vertex, std::vector<int> verticesToConnect)
{
	auto faceThatContainNewVertex = this->findPoint(vertex->getCoordinate());

	this->m_Vertices.push_back(vertex);
	this->updateVertexIds();

	auto halfEdge = faceThatContainNewVertex->isOn(vertex->getCoordinate());
	if (halfEdge) {
		this->addVertexOnTheEdge(halfEdge, vertex);
	}
	else {
		this->addVertexInsideTheFace(faceThatContainNewVertex, vertex, verticesToConnect);
	}
}
void DoublyConnectedList::DCEL::deleteEdgeFromList(std::shared_ptr<DoublyConnectedList::HalfEdge> edge)
{
	int counter = 0;
	for (auto& halfEdge : this->m_HalfEdges) {
		if (halfEdge == edge)
			break;
		counter++;
	}
	this->m_HalfEdges.erase(this->m_HalfEdges.begin() + counter, this->m_HalfEdges.begin() + counter + 1);
}
void DoublyConnectedList::DCEL::deleteVertexFromList(int ID)
{
	this->m_Vertices.erase(this->m_Vertices.begin() + ID, this->m_Vertices.begin() + ID + 1);
}
void DoublyConnectedList::DCEL::deleteFaceFromList(std::shared_ptr<DoublyConnectedList::Face> face)
{
	int counter = 0;
	for (auto& face1 : this->m_Faces) {
		if (face1 == face)
			break;
		counter++;
	}
	this->m_Faces.erase(this->m_Faces.begin() + counter, this->m_Faces.begin() + counter + 1);
}
void DoublyConnectedList::DCEL::buildDCEL(const std::vector<std::pair<double, double>>& vertexInput, const std::vector<std::pair<int, int>>& edgeInput)
{
	// Create Vertices
	int counter = this->m_Vertices.size();
	for (auto& vInput : vertexInput)
		this->m_Vertices.push_back(std::make_shared<DoublyConnectedList::Vertex>(vInput.first, vInput.second, counter++));
	
	// Create Half-Edges
	for (const auto& eInput : edgeInput)
		if (eInput.first >= 0 && eInput.second >= 0)
			this->createHalfEdge(this->m_Vertices[eInput.first], this->m_Vertices[eInput.second]);

	// Half-Edge next and prev assignments
	for (auto& vertex : this->m_Vertices)
		vertex->prevAndNextAssignments();

	// Create Faces
	auto dummyList = this->m_HalfEdges;
	size_t numHalfEdges = dummyList.size();
	while (numHalfEdges > 0)
	{
		auto halfEdge = dummyList.back();
		dummyList.pop_back();
		numHalfEdges = dummyList.size();
		if (halfEdge->getIncidentFace() == nullptr)
			this->m_Faces.push_back(halfEdge->faceAssignmentToEdges(halfEdge));
	}
	for (auto& face : this->m_Faces) {
		face->setIfExternal(face->calculateArea() < 0);
		face->calculateEdgeCount();
	}
}
void DoublyConnectedList::DCEL::buildDCEL(const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& vertexInput, const std::vector<std::pair<int, int>>& edgeInput)
{
	// Create Vertices
	int counter = this->m_Vertices.size();
	for (auto& vInput : vertexInput)
		this->m_Vertices.push_back(vInput);
	
	// Update vertex ids
	this->updateVertexIds();

	// Create Half-Edges
	for (auto& eInput : edgeInput)
		if (eInput.first >= 0 && eInput.second >= 0)
			this->createHalfEdge(this->m_Vertices[eInput.first], this->m_Vertices[eInput.second]);

	// Half-Edge next and prev assignments
	for (auto& vertex : this->m_Vertices)
		vertex->prevAndNextAssignments();

	// Create Faces
	auto dummyList = this->m_HalfEdges;
	size_t numHalfEdges = dummyList.size();
	while (numHalfEdges > 0)
	{
		auto halfEdge = dummyList.back();
		dummyList.pop_back();
		numHalfEdges = dummyList.size();
		if (halfEdge->getIncidentFace() == nullptr)
			this->m_Faces.push_back(halfEdge->faceAssignmentToEdges(halfEdge));
	}
	for (auto& face : this->m_Faces) {
		face->setIfExternal(face->calculateArea() < 0);
		face->calculateEdgeCount();
	}
}
std::shared_ptr<DoublyConnectedList::Face> DoublyConnectedList::DCEL::findPoint(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates)
{
	for (auto& face : this->m_Faces)
	{
		if (face->isExternal())
			continue;
		else if (face->isInside(pointCoordinates))
			return face;
		else if (face->isOn(pointCoordinates))
			return face;
	}
	return nullptr;
}
void DoublyConnectedList::DCEL::addVertexOnTheEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> edge, std::shared_ptr<DoublyConnectedList::Vertex> vertex)
{
	auto oldHalfEdge = edge;
	auto oldHalfEdgeTwin = edge->getTwinHalfEdge();

	auto halfEdgeOrigin = oldHalfEdge->getOrigin();
	auto halfEdgeTwinOrigin = oldHalfEdgeTwin->getOrigin();

	// Create new edges and assign their twin edges.
	auto halfEdge1 = std::make_shared<DoublyConnectedList::HalfEdge>(halfEdgeOrigin, vertex);
	auto halfEdge2 = std::make_shared<DoublyConnectedList::HalfEdge>(vertex, halfEdgeTwinOrigin);
	auto halfEdge3 = std::make_shared<DoublyConnectedList::HalfEdge>(vertex, halfEdgeOrigin);
	auto halfEdge4 = std::make_shared<DoublyConnectedList::HalfEdge>(halfEdgeTwinOrigin, vertex);
	halfEdge1->setTwinHalfEdge(halfEdge3);
	halfEdge2->setTwinHalfEdge(halfEdge4);
	halfEdge3->setTwinHalfEdge(halfEdge1);
	halfEdge4->setTwinHalfEdge(halfEdge2);

	// Add new edges to the lists
	vertex->addHalfEdge(halfEdge2);
	vertex->addHalfEdge(halfEdge3);
	halfEdgeOrigin->addHalfEdge(halfEdge1);
	halfEdgeTwinOrigin->addHalfEdge(halfEdge4);
	this->m_HalfEdges.push_back(halfEdge1);
	this->m_HalfEdges.push_back(halfEdge2);
	this->m_HalfEdges.push_back(halfEdge3);
	this->m_HalfEdges.push_back(halfEdge4);

	// Delete old edges from the lists.
	halfEdgeOrigin->deleteEdgeFromList(oldHalfEdge);
	halfEdgeTwinOrigin->deleteEdgeFromList(oldHalfEdgeTwin);
	this->deleteEdgeFromList(oldHalfEdge);
	this->deleteEdgeFromList(oldHalfEdgeTwin);

	// Since the new edge is added re-run the
	// sort algorithm.
	vertex->angleSortEdges();
	halfEdgeOrigin->angleSortEdges();
	halfEdgeTwinOrigin->angleSortEdges();

	// Assign new prevs and nexts for the edges 
	// around the two vertices.
	vertex->prevAndNextAssignments();
	halfEdgeOrigin->prevAndNextAssignments();
	halfEdgeTwinOrigin->prevAndNextAssignments();

	// Assign faces to the new edges.
	auto face1 = oldHalfEdge->getIncidentFace();
	auto face2 = oldHalfEdgeTwin->getIncidentFace();
	halfEdge1->setIncidentFace(face1);
	halfEdge2->setIncidentFace(face1);
	halfEdge3->setIncidentFace(face2);
	halfEdge4->setIncidentFace(face2);
	if (face1->getHalfEdge() == oldHalfEdge)
		face1->setHalfEdge(halfEdge1);
	if (face2->getHalfEdge() == oldHalfEdgeTwin)
		face2->setHalfEdge(halfEdge4);
	face1->calculateEdgeCount();
	face2->calculateEdgeCount();
}
void DoublyConnectedList::DCEL::addVertexInsideTheFace(std::shared_ptr<DoublyConnectedList::Face> face, std::shared_ptr<DoublyConnectedList::Vertex> vertex, std::vector<int> verticesToConnect)
{
	if (verticesToConnect.size() < 2)
		return;
	// delete the old face that the point is indside.
	face->clearFaceAssignments();
	this->deleteFaceFromList(face);
	for (auto vertexID : verticesToConnect)
	{
		// Create Half-Edges
		this->createHalfEdge(vertex, this->m_Vertices[vertexID]);
	}
	for (auto vertexID : verticesToConnect)
	{
		// for the points that has new edge make the prev and next assignments
		this->m_Vertices[vertexID]->prevAndNextAssignments();
	}
	// make the prev and next assignments for the edges around the new added vertex.
	vertex->prevAndNextAssignments();

	// assign new faces
	for (auto hEdge : vertex->getHalfEdge())
	{
		auto newFace = hEdge->faceAssignmentToEdges(hEdge);
		newFace->calculateArea();
		newFace->calculateEdgeCount();
		this->m_Faces.push_back(newFace);
	}
}
std::pair<std::shared_ptr<DoublyConnectedList::HalfEdge>, std::shared_ptr<DoublyConnectedList::HalfEdge>>
DoublyConnectedList::DCEL::createHalfEdge(std::shared_ptr<DoublyConnectedList::Vertex> vertex1, std::shared_ptr<DoublyConnectedList::Vertex> vertex2)
{
	// Create two twin half-edges between given vertices
	auto hEdge1 = std::make_shared<DoublyConnectedList::HalfEdge>(vertex1, vertex2);
	auto hEdge2 = std::make_shared<DoublyConnectedList::HalfEdge>(vertex2, vertex1);
	// set the twin assignments for this new half-edges
	hEdge1->setTwinHalfEdge(hEdge2);
	hEdge2->setTwinHalfEdge(hEdge1);
	// add this edges inside the half edge list of origin vertices.
	vertex1->addHalfEdge(hEdge1);
	vertex2->addHalfEdge(hEdge2);
	// add this edges inside the halfEdges list of DCEL object.
	this->m_HalfEdges.push_back(hEdge1);
	this->m_HalfEdges.push_back(hEdge2);

	// return this new created edges.
	return { hEdge1 , hEdge2 };
}
void DoublyConnectedList::DCEL::ExportVTKFormat(std::string filename)
{
	std::ofstream file;

	file.open(filename);
	{
		file << "# vtk DataFile Version 2.0" << std::endl;
		file << filename << std::endl;
		file << "ASCII" << std::endl << std::endl;
		file << "DATASET POLYDATA" << std::endl;
		file << "POINTS " << this->m_Vertices.size() << " float" << std::endl;
		for (auto& vertex : this->m_Vertices)
		{
			auto coord = vertex->getCoordinate();
			file << coord.xCoord << " " << coord.yCoord  << " " << 0.0 << std::endl;
		}
		// Get total edge count by extracting edge countof the external face
		// from the total half edge count.
		std::shared_ptr<DoublyConnectedList::Face> externalFace;
		for (auto& face : this->m_Faces)
			if (face->isExternal())
				externalFace = face;
		int totalEdge = this->m_HalfEdges.size() - externalFace->getEdgeCount();

		file << std::endl;
		file << "POLYGONS " << this->m_Faces.size()-1 << " " << totalEdge + this->m_Faces.size() - 1 << std::endl;
		
		for (auto& face : this->m_Faces) {
			auto halfEdge = face->getHalfEdge();
			if (face->isExternal())
				continue;
			file << face->getEdgeCount() << " ";
			do
			{
				file << halfEdge->getOrigin()->getID() << " ";
				halfEdge = halfEdge->getNextHalfEdge();
			} while (halfEdge != face->getHalfEdge());
			file << std::endl;
		}
	}
	file.close();
}
