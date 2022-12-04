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
std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>> DoublyConnectedList::Vertex::getHalfEdge() { return this->m_HalfEdges; }
int DoublyConnectedList::Vertex::getID() { return this->m_ID; }
void DoublyConnectedList::Vertex::addHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) { this->m_HalfEdges.push_back(halfEdge); };
void DoublyConnectedList::Vertex::angleSortEdges()
{
	auto sort = [&](std::shared_ptr<DoublyConnectedList::HalfEdge> halfEgde1, std::shared_ptr<DoublyConnectedList::HalfEdge> halfEgde2)
	{
		return halfEgde1->getAngle() < halfEgde2->getAngle();
	};
	std::sort(this->m_HalfEdges.begin(), this->m_HalfEdges.end(), sort);
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
DoublyConnectedList::Vertex::Coordinates DoublyConnectedList::Vertex::getCoordinate() { return this->m_Coordinates;}

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
		this->m_Angle = 2 * pi - acos(dx / this->m_Length);
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
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::HalfEdge::getNextHalfEdge() { return this->m_Next; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::HalfEdge::getPrevHalfEdge() { return this->m_Prev; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::HalfEdge::getTwinHalfEdge() { return this->m_Twin; }
std::shared_ptr<DoublyConnectedList::Face> DoublyConnectedList::HalfEdge::getIncidentFace() { return this->m_IncidentFace; }
std::shared_ptr<DoublyConnectedList::Vertex> DoublyConnectedList::HalfEdge::getOrigin() { return this->m_Origin; }
double DoublyConnectedList::HalfEdge::getAngle() { return this->m_Angle; }
double DoublyConnectedList::HalfEdge::getLength() { return this->m_Length; }
double DoublyConnectedList::HalfEdge::determinant(const std::vector<double>& point1, const std::vector<double>& point2){ return (point1[0]) * (point2[1]) - (point1[1]) * (point2[0]); }
bool DoublyConnectedList::HalfEdge::isPointOnLeft(std::shared_ptr<DoublyConnectedList::Vertex> point)
{
	auto twinOrgCoor = this->m_Twin->getOrigin()->getCoordinate();
	auto orgCoor = this->m_Origin->getCoordinate();
	auto area = determinant({ (point->getCoordinate().xCoord - orgCoor.xCoord) ,(point->getCoordinate().yCoord - orgCoor.yCoord) },
		{ (twinOrgCoor.xCoord - orgCoor.xCoord) ,  (twinOrgCoor.yCoord - orgCoor.yCoord) });
	return area < 0.0;
}
bool DoublyConnectedList::HalfEdge::isPointOn(std::shared_ptr<DoublyConnectedList::Vertex> point)
{
	auto twinOrgCoor = this->m_Twin->getOrigin()->getCoordinate();
	auto orgCoor = this->m_Origin->getCoordinate();
	auto area = determinant({ (point->getCoordinate().xCoord - orgCoor.xCoord) ,(point->getCoordinate().yCoord - orgCoor.yCoord) },
		{ (twinOrgCoor.xCoord - orgCoor.xCoord) ,  (twinOrgCoor.yCoord - orgCoor.yCoord) });
	auto pointToOriginLength = sqrt(pow((point->getCoordinate().xCoord - this->m_Origin->getCoordinate().xCoord), 2) +
		pow((point->getCoordinate().yCoord - this->m_Origin->getCoordinate().yCoord), 2));
	return (abs(area) < 0.001 && pointToOriginLength < this->m_Length);
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
int DoublyConnectedList::Face::getEdgeCount() { return this->m_EdgeCount; }
void DoublyConnectedList::Face::setHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) { this->m_HalfEdgeComponent = halfEdge; }
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::Face::getHalfEdge(){ return this->m_HalfEdgeComponent; }
bool DoublyConnectedList::Face::isInside(std::shared_ptr<DoublyConnectedList::Vertex> point)
{
	auto halfEdge = this->m_HalfEdgeComponent;
	do
	{
		if (!halfEdge->isPointOnLeft(point))
			return false;
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	return true;
}
std::shared_ptr<DoublyConnectedList::HalfEdge> DoublyConnectedList::Face::isOn(std::shared_ptr<DoublyConnectedList::Vertex> point)
{
	auto halfEdge = this->m_HalfEdgeComponent;
	do
	{
		if (halfEdge->isPointOn(point))
			return halfEdge;
		halfEdge = halfEdge->getNextHalfEdge();
	} while (halfEdge != this->m_HalfEdgeComponent);
	return nullptr;
}
std::shared_ptr<DoublyConnectedList::Vertex> DoublyConnectedList::Face::getClosestPoint(std::shared_ptr<DoublyConnectedList::Vertex> vert)
{
	double closestDistance = DBL_MAX;
	std::shared_ptr<DoublyConnectedList::Vertex> closestpoint;
	auto halfEdge = this->m_HalfEdgeComponent;
	do {
		auto origin = halfEdge->getOrigin();
		double xDifference = vert->getCoordinate().xCoord - origin->getCoordinate().xCoord;
		double yDifference = vert->getCoordinate().yCoord - origin->getCoordinate().yCoord;
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
DoublyConnectedList::DCEL::DCEL(std::vector<std::vector<double>> vertexInput,
	std::vector<std::vector<int>> edgeInput) : m_Vertices(std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>()),
	m_HalfEdges(std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>()),
	m_Faces(std::vector<std::shared_ptr<DoublyConnectedList::Face>>()) 
{
	if (vertexInput.size() != 0)
		buildDCEL(vertexInput, edgeInput);
}
size_t DoublyConnectedList::DCEL::getNumberOfVertices() { return this->m_Vertices.size(); }
size_t DoublyConnectedList::DCEL::getNumberOfHalfEdges(){ return this->m_HalfEdges.size(); }
size_t DoublyConnectedList::DCEL::getNumberOfFaces() { return this->m_Faces.size(); }
std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> DoublyConnectedList::DCEL::getVertices() { return this->m_Vertices; }
std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>> DoublyConnectedList::DCEL::getHalfEdges() { return this->m_HalfEdges; }
std::vector<std::shared_ptr<DoublyConnectedList::Face>> DoublyConnectedList::DCEL::getFaces() { return this->m_Faces; }
void DoublyConnectedList::DCEL::updateVertexIds() { int counter = 0; for (auto& vertex : this->m_Vertices) vertex->setID(counter++); }
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
		auto hEdge1 = std::make_shared<DoublyConnectedList::HalfEdge>(vertex1, vertex2);
		auto hEdge2 = std::make_shared<DoublyConnectedList::HalfEdge>(vertex2, vertex1);
		hEdge1->setTwinHalfEdge(hEdge2);
		hEdge2->setTwinHalfEdge(hEdge1);
		vertex1->addHalfEdge(hEdge1);
		vertex2->addHalfEdge(hEdge2);
		this->m_HalfEdges.push_back(hEdge1);
		this->m_HalfEdges.push_back(hEdge2);

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
void DoublyConnectedList::DCEL::addVertex(double xCoord, double yCoord, std::vector<std::vector<int>> edgeInput)
{
	auto newVertex = std::make_shared<DoublyConnectedList::Vertex>(xCoord, yCoord);
	auto faceThatContainNewVertex = this->findPoint(newVertex);

	this->m_Vertices.push_back(newVertex);
	this->updateVertexIds();

	auto halfEdge = faceThatContainNewVertex->isOn(newVertex);
	if (halfEdge) {
		this->addVertexOnTheEdge(halfEdge, newVertex);
	}
	else {
		this->addVertexInsideTheFace(faceThatContainNewVertex, newVertex);
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
void DoublyConnectedList::DCEL::buildDCEL(std::vector<std::vector<double>> vertexInput, std::vector<std::vector<int>> edgeInput)
{
	// Create Vertices
	int counter = 0;
	for (auto& vInput : vertexInput)
		this->m_Vertices.push_back(std::make_shared<DoublyConnectedList::Vertex>(vInput[0], vInput[1], counter++));
	
	// Create Half-Edges
	for (auto& eInput : edgeInput)
	{
		if (eInput[0] >= 0 && eInput[1] >= 0)
		{
			auto hEdge1 = std::make_shared<DoublyConnectedList::HalfEdge>(this->m_Vertices[eInput[0]], this->m_Vertices[eInput[1]]);
			auto hEdge2 = std::make_shared<DoublyConnectedList::HalfEdge>(this->m_Vertices[eInput[1]], this->m_Vertices[eInput[0]]);
			hEdge1->setTwinHalfEdge(hEdge2);
			hEdge2->setTwinHalfEdge(hEdge1);
			this->m_Vertices[eInput[0]]->addHalfEdge(hEdge1);
			this->m_Vertices[eInput[1]]->addHalfEdge(hEdge2);
			this->m_HalfEdges.push_back(hEdge1);
			this->m_HalfEdges.push_back(hEdge2);
		}
	}

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
std::shared_ptr<DoublyConnectedList::Face> DoublyConnectedList::DCEL::findPoint(std::shared_ptr<DoublyConnectedList::Vertex> point)
{
	for (auto& face : this->m_Faces)
	{
		if (face->isExternal())
			continue;
		else if (face->isInside(point))
			return face;
		else if (face->isOn(point))
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
void DoublyConnectedList::DCEL::addVertexInsideTheFace(std::shared_ptr<DoublyConnectedList::Face> face, std::shared_ptr<DoublyConnectedList::Vertex> vertex)
{
	//TODO
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
