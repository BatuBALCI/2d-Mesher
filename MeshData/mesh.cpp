#include "Mesh.h"

void BasicQuadMesh::Mesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (int i = 0; i < 5; i++)
		if (this->TryMesh(domains, i))
			break;
	domains.at(0)->getDCEL()->ExportVTKFormat("try.vtk");
}
bool BasicQuadMesh::TryMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains, int tryCount) {
	if (tryCount == 0) {
		auto comp = [](std::shared_ptr<MeshData::Domain> a, std::shared_ptr<MeshData::Domain> b) {
			return (a->getEdgeLength() < b->getEdgeLength());
		};
		this->m_OptimumLength = std::min_element(domains.begin(), domains.end(), comp)->get()->getEdgeLength();
		this->FindOptimumEdgeLength(domains);
	}
	else {
		this->m_OptimumLength /= 1.125;
		this->FindOptimumEdgeLength(domains);
	}

	this->BuildInitialGrid(domains);
	this->ReturnToCartesian(domains);
	this->AddPointConstraintDomains(domains);
	this->AddLineConstraint(domains);

	return this->CheckQualityOfMesh(domains);
}
void BasicQuadMesh::BuildInitialGrid(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (int i = 0; i < domains.size(); i++) {
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> vertexInput;
		std::vector<std::vector<int>> edgeInput;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge1Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge2Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge3Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge4Vertices;

		auto insertEdgeVertices = [&vertexInput](std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edgeVertices, size_t startOffset, size_t endOffset)
		{
			for (size_t i = startOffset; i < edgeVertices.size() - endOffset; i++) {
				edgeVertices.at(i)->setID(vertexInput.size());
				vertexInput.push_back(edgeVertices.at(i));
			}
		};

		// Assign the edge vertices.
		auto domain = domains.at(i);
		auto edgeDivideNums = m_EdgeDivideNum.at(i);
		for (int j = 0; j < domain->getEdges().size(); j++) {
			auto edge = domain->getEdges().at(j);
			auto startCorner = edge->getStartCorner();
			auto endCorner = edge->getEndCorner();
			int edgeDivideNum;
			if (j % 2 == 0)
				edgeDivideNum = edgeDivideNums.first;
			else
				edgeDivideNum = edgeDivideNums.second;

			auto direction = (endCorner->getCornerVertex()->getCoordinate() - startCorner->getCornerVertex()->getCoordinate()).normalize();
			std::shared_ptr<DoublyConnectedList::Vertex> vertex;
			for (size_t k = 0; k < edgeDivideNum + 1; k++) {
				if (k == 0)
					vertex = startCorner->getCornerVertex();
				else if (k == edgeDivideNum)
					vertex = endCorner->getCornerVertex();
				else {
					auto coordinate = startCorner->getCornerVertex()->getCoordinate() + direction * (((double)k * 2.0)/ (double)edgeDivideNum);
					vertex = std::make_shared<DoublyConnectedList::Vertex>(coordinate.xCoord, coordinate.yCoord);
				}
				switch (j) {
				case 0:
					edge1Vertices.push_back(vertex);
					break;
				case 1:
					edge2Vertices.push_back(vertex);
					break;
				case 2:
					edge3Vertices.push_back(vertex);
					break;
				case 3:
					edge4Vertices.push_back(vertex);
					break;
				}
			}
		}

		// Insert vertices of the edge one inside the input vertices list.
		insertEdgeVertices(edge1Vertices, 0, 0);
		insertEdgeVertices(edge2Vertices, 1, 0);
		insertEdgeVertices(edge3Vertices, 1, 0);
		insertEdgeVertices(edge4Vertices, 1, 1);
		
		// Generate connectivity.
		for (size_t i = 0; i < vertexInput.size() - 1; i++)
		{
			std::vector<int> connectivity;
			connectivity.push_back(vertexInput.at(i)->getID());
			connectivity.push_back(vertexInput.at(i + 1)->getID());
			edgeInput.push_back(connectivity);
		}
		edgeInput.push_back({ vertexInput.back()->getID(), vertexInput.at(0)->getID() });

		// have the edge4 vertices in reverse order to use while generating connectivity.
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> lastColumn;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> lastColumnDummy;
		for (auto i = edge4Vertices.rbegin() + 1; i != edge4Vertices.rend() - 1; i++)
			lastColumn.push_back(*i);
		// Assign interior vertices.
		for (size_t i = 1; i < edge1Vertices.size() - 1; i++) {
			auto lastVertex = edge1Vertices.at(i);
			for (size_t j = 1; j < edge2Vertices.size() - 1; j++) {
				auto direction1 = (edge3Vertices.at(edge3Vertices.size() - 1 - i)->getCoordinate() - edge1Vertices.at(i)->getCoordinate()).normalize();
				auto direction2 = (edge4Vertices.at(edge4Vertices.size() - 1 - j)->getCoordinate() - edge2Vertices.at(j)->getCoordinate()).normalize();

				auto origin1 = edge1Vertices.at(i)->getCoordinate();
				auto origin2 = edge2Vertices.at(j)->getCoordinate();

				// find the intersection between two crossing lines to get intersection point
				auto intersectionCoordinate = RayIntersection(origin1, direction1, origin2, direction2);

				// generate new interior vertex and assign its id.
				auto newVertex = std::make_shared<DoublyConnectedList::Vertex>(intersectionCoordinate.xCoord, intersectionCoordinate.yCoord);
				newVertex->setID(vertexInput.size());
				vertexInput.push_back(newVertex);

				// hold the pointer of this new created vertex to use as last column while generating connectivity.
				lastColumnDummy.push_back(newVertex);

				std::vector<int> connectivity;
				connectivity.push_back(newVertex->getID());
				connectivity.push_back(lastVertex->getID());
				edgeInput.push_back(connectivity);
				connectivity.clear();
				lastVertex = newVertex;

				connectivity.push_back(newVertex->getID());
				connectivity.push_back(lastColumn.at(j-1)->getID());
				edgeInput.push_back(connectivity);
				connectivity.clear();

				if (j == edge2Vertices.size() - 2) {
					connectivity.push_back(newVertex->getID());
					connectivity.push_back(edge3Vertices.at(edge3Vertices.size() - 1 - i)->getID());
					edgeInput.push_back(connectivity);
					connectivity.clear();
				}

				if (i == edge1Vertices.size() - 2) {
					connectivity.push_back(newVertex->getID());
					connectivity.push_back(edge2Vertices.at(j)->getID());
					edgeInput.push_back(connectivity);
					connectivity.clear();
				}
			}
			lastColumn = lastColumnDummy;
			lastColumnDummy.clear();
		}
		domain->generateDCEL(vertexInput, edgeInput);
	}
}
void BasicQuadMesh::AddPointConstraintDomains(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains)
		this->AddPointConstraint(domain, domain->getPointConstriants());
}
void BasicQuadMesh::AddPointConstraint(std::shared_ptr<MeshData::Domain> domain, const std::vector<std::shared_ptr<MeshData::Domain::PointConstraint>>& constrains) {
	auto DCEL = domain->getDCEL();
	for (auto constraint : constrains) {
		// generate new vertex for onstraint.
		auto vertex = std::make_shared<DoublyConnectedList::Vertex>(constraint->getCoordinates().xCoord, constraint->getCoordinates().yCoord);
		auto face = DCEL->findPoint(vertex);
		auto closestVertex = face->getClosestPoint(vertex);
		std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge;
		for (auto hEdge : closestVertex->getHalfEdge()) {
			if (face == hEdge->getIncidentFace()) {
				halfEdge = hEdge;
				break;
			}
		}
		auto nextVertex = halfEdge->getTwinHalfEdge()->getOrigin();
		DCEL->addVertex(vertex, { closestVertex->getID(), nextVertex->getID() });
	}
}
void BasicQuadMesh::AddLineConstraint(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains)
		for (auto& lineConstraint : domain->getLineConstraints())
			this->AddPointConstraint(domain, lineConstraint->getConstraints());
}
void BasicQuadMesh::ReturnToCartesian(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {
		std::vector<std::pair<double, double>> nodes;
		for (auto corner : domain->getCorners())
			nodes.push_back({ corner->getCoordinates().xCoord, corner->getCoordinates().yCoord });
		for (auto vertex : domain->getDCEL()->getVertices())
			vertex->setCoordinate(MapFromNeutralCoordinate(vertex->getCoordinate().xCoord, vertex->getCoordinate().yCoord,
				nodes, [&](double x, double y)
				{return std::vector<double>({
					0.25 * (1 - x) * (1 - y),
					-0.25 * (-1 - x) * (1 - y),
					0.25 * (-1 - x) * (-1 - y),
					-0.25 * (1 - x) * (-1 - y) }); }));

		// update half-edge propertiess
		for (auto halfEdge : domain->getDCEL()->getHalfEdges())
			halfEdge->updateProperties();
		// update face propertiess
		for (auto face : domain->getDCEL()->getFaces()) {
			face->calculateArea();
			face->calculatePerimeter();
		}
	}
}
void BasicQuadMesh::FindOptimumEdgeLength(const std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	this->m_EdgeDivideNum.clear();
	std::vector<long long> constraintLengthsDomains;
	for (auto domain : domains){
		// this vector holds the constreaint distances for EDGE 1 and EDGE 3
		std::vector<long long> constraintLengths1;
		// this vector holds the constreaint distances for EDGE 2 and EDGE 4
		std::vector<long long> constraintLengths2;

		int edgeID = 1;
		auto addConstraintLength = [&](double length) {
			if (edgeID % 2 != 0)
				constraintLengths1.push_back(ceil(length * 1e10));
			else
				constraintLengths2.push_back(ceil(length * 1e10));
		};

		for (auto edge : domain->getEdges()) {
			// get the constraints on the given edge and sort them to use in order.
			auto constraint = edge->getEdgeConstraint();
			auto edgeLength = edge->getLength();
			if (constraint) {
				auto relLocations = constraint->getRelativeConstraintLocations();
				std::sort(relLocations.begin(), relLocations.end());
				double prevLocation = 0;
				for (auto relLocation : relLocations) {
					addConstraintLength((relLocation - prevLocation) * edgeLength);
					prevLocation = relLocation;
				}
				addConstraintLength((1.0 - prevLocation) * edgeLength);
			}
			else 
				addConstraintLength(edgeLength);
			edgeID++;
		}
		// optimum edge length candidate for edge 1-3
		auto edgeLength1 = boost::integer::gcd_range(constraintLengths1.begin(), constraintLengths1.end()).first;
		// optimum edge length candidate for edge 2-4
		auto edgeLength2 = boost::integer::gcd_range(constraintLengths2.begin(), constraintLengths2.end()).first;

		// if the edgeLength input is lower than the found edge length than find new edge length for 
		if (!(std::round(this->m_OptimumLength * 1e10) > edgeLength1))
			edgeLength1 = edgeLength1 / (long long)ceil(edgeLength1 / std::round(this->m_OptimumLength * 1e10));
		if (!(std::round(this->m_OptimumLength * 1e10) > edgeLength2))
			edgeLength2 = edgeLength2 / (long long)ceil(edgeLength2 / std::round(this->m_OptimumLength * 1e10));

		// find the least common divison numbers for egde1-3.
		auto divideNum_1_3 = boost::math::lcm(std::llround(domain->getEdges()[0]->getLength() / (edgeLength1 / 1e10)), std::llround(domain->getEdges()[2]->getLength() / (edgeLength1 / 1e10)));
		// find the least common divison numbers for egde2-4.
		auto divideNum_2_4 = boost::math::lcm(std::llround(domain->getEdges()[1]->getLength() / (edgeLength2 / 1e10)), std::llround(domain->getEdges()[3]->getLength() / (edgeLength2 / 1e10)));
		
		// TODO check aspect ratio.
		
		// TODO if there are more than one domain than change this part to have compatible divide numbers at the intersection edges.
		// add the divide numbers to the vector.
		this->m_EdgeDivideNum.push_back({ divideNum_1_3 , divideNum_2_4 });
	}
}
bool BasicQuadMesh::CheckQualityOfMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {

	}
	return true;
}

DoublyConnectedList::Vertex::Coordinates BasicQuadMesh::RayIntersection(const DoublyConnectedList::Vertex::Coordinates& origin1, const DoublyConnectedList::Vertex::Coordinates& direction1,
	const DoublyConnectedList::Vertex::Coordinates& origin2, const DoublyConnectedList::Vertex::Coordinates& direction2) {
	// normalize the direction vectors
	auto dir1 = direction1 / (sqrt(direction1.xCoord * direction1.xCoord + direction1.yCoord * direction1.yCoord));
	auto dir2 = direction2 / (sqrt(direction2.xCoord * direction2.xCoord + direction2.yCoord * direction2.yCoord));
	// Caulculate the distance between the intersection point and the origin 2 by solving the intersection
	// equation between 2 rays.
	double distanceBetweenTheIntersectionAndOrigin2;
	if (abs(dir1.xCoord) < 10e-15)
		distanceBetweenTheIntersectionAndOrigin2 = (origin1.xCoord - origin2.xCoord) / dir2.xCoord;
	else
		auto distanceBetweenTheIntersectionAndOrigin2 = (origin1.yCoord + dir1.yCoord * (origin2.xCoord - origin1.xCoord) / dir1.xCoord - origin2.yCoord) /
		(dir2.yCoord - dir2.xCoord * dir1.yCoord / dir1.xCoord);

	auto result = origin2 + dir2 * distanceBetweenTheIntersectionAndOrigin2;
	return result;
}

DoublyConnectedList::Vertex::Coordinates BasicQuadMesh::MapFromNeutralCoordinate(double neutralKsi, double neutralEta, const std::vector<std::pair<double, double>>& nodeLocation, std::function<std::vector<double>(double, double)> shapeFunction){
	DoublyConnectedList::Vertex::Coordinates result{ 0,0 };
	auto interpolationFunction = shapeFunction(neutralKsi, neutralEta);

	for(size_t i = 0; i < interpolationFunction.size(); i++) {
		result.xCoord += interpolationFunction[i] * nodeLocation[i].first;
		result.yCoord += interpolationFunction[i] * nodeLocation[i].second;
	}
		
	return result;
}

