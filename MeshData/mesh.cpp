#include "Mesh.h"

void BasicQuadMesh::Mesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (int i = 0; i < 5; i++) {
		if (this->TryMesh(domains, i)) {
			break;
		}
	}
}
bool BasicQuadMesh::TryMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains, int tryCount) {
	this->ClearDomain(domains);
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

	if (!this->AddPointConstraints(domains))
		return false;
	if (!this->AddLineConstraint(domains))
		return false;

	TriangleToQuad(domains);

	return this->CheckQualityOfMesh(domains);
}
void BasicQuadMesh::BuildInitialGrid(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (int i = 0; i < domains.size(); i++) {
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> vertexInput;
		std::vector<std::pair<int, int>> edgeInput;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge1Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge2Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge3Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> edge4Vertices;

		std::unordered_map<std::shared_ptr<DoublyConnectedList::Face>, std::shared_ptr<DoublyConnectedList::HalfEdge>> faceWithBottomHalfEdge;

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
				vertex->setConstraint(DoublyConnectedList::Vertex::Constraints::EDGE);
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
			std::pair<int, int> connectivity;
			connectivity.first = vertexInput.at(i)->getID();
			connectivity.second = vertexInput.at(i + 1)->getID();
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

				std::pair<int, int> connectivity;
				connectivity.first = newVertex->getID();
				connectivity.second = lastVertex->getID();
				edgeInput.push_back(connectivity);
				lastVertex = newVertex;

				connectivity.first = newVertex->getID();
				connectivity.second = lastColumn.at(j-1)->getID();
				edgeInput.push_back(connectivity);

				if (j == edge2Vertices.size() - 2) {
					connectivity.first = newVertex->getID();
					connectivity.second = edge3Vertices.at(edge3Vertices.size() - 1 - i)->getID();
					edgeInput.push_back(connectivity);
				}

				if (i == edge1Vertices.size() - 2) {
					connectivity.first = newVertex->getID();
					connectivity.second = edge2Vertices.at(j)->getID();
					edgeInput.push_back(connectivity);
				}
			}
			lastColumn = lastColumnDummy;
			lastColumnDummy.clear();
		}
		domain->generateDCEL(vertexInput, edgeInput);
		for (auto vertex : vertexInput) {
			if (vertex->getHalfEdge().front()->getAngle() < 0.0000001  && !vertex->getHalfEdge().front()->getIncidentFace()->isExternal())
				faceWithBottomHalfEdge[vertex->getHalfEdge().front()->getIncidentFace()] = vertex->getHalfEdge().front();
			else if(vertex->getHalfEdge().back()->getAngle() > 2 * PI - 0.0000001 && !vertex->getHalfEdge().back()->getIncidentFace()->isExternal())
				faceWithBottomHalfEdge[vertex->getHalfEdge().back()->getIncidentFace()] = vertex->getHalfEdge().back();
		}
		this->ReturnToCartesian(domain);
		
		// Divide the generated quads into triangles.
		// The division takes place from the largest interior angle of the quad.
		std::vector<std::pair<int, int>> newEdges;
		for (auto face : domain->getDCEL()->getFaces()) {
			if (face->isExternal())
				continue;
			std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdgeWithBiggestAngleOrigin;
			double biggestAngle = 0.0;
			auto halfEdge = face->getHalfEdge();

			auto getAngle = [](std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) {
				auto angle1 = halfEdge->getAngle();
				auto prevHalfEdge = halfEdge->getPrevHalfEdge();
				auto angle2 = prevHalfEdge->getAngle();
				auto angle = PI - angle1 + angle2;
				while (angle > 2 * PI)
					angle -= 2 * PI;
				return angle;
			};
			do
			{
				auto angle = getAngle(halfEdge);
				if (angle > biggestAngle) {
					biggestAngle = angle;
					halfEdgeWithBiggestAngleOrigin = halfEdge;
				}
				halfEdge = halfEdge->getNextHalfEdge();
			} while (halfEdge != face->getHalfEdge());
			if (biggestAngle < (PI/2.0) + 0.0000001 || getAngle(faceWithBottomHalfEdge[face]) > (PI / 2.0) || 
				getAngle(faceWithBottomHalfEdge[face]->getNextHalfEdge()->getNextHalfEdge()) > (PI / 2.0))
				halfEdgeWithBiggestAngleOrigin = faceWithBottomHalfEdge[face];
			newEdges.push_back({ halfEdgeWithBiggestAngleOrigin->getOrigin()->getID(), halfEdgeWithBiggestAngleOrigin->getNextHalfEdge()->getNextHalfEdge()->getOrigin()->getID() });
		}
		for (auto edge : newEdges)
			domain->getDCEL()->addEdge(edge.first, edge.second);
	}
}
bool BasicQuadMesh::AddPointConstraints(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {
		for (auto constraint : domain->getPointConstriants()) {
			// Add this constraint.
			if (!AddConstraint(domain, constraint->getCoordinates(), DoublyConnectedList::Vertex::Constraints::POINT))
				return false;
		}
	}
	return true;
}
bool BasicQuadMesh::AddConstraint(std::shared_ptr<MeshData::Domain> domain, const DoublyConnectedList::Vertex::Coordinates& constrainCoordinate, DoublyConnectedList::Vertex::Constraints type) {
	// Get the generated DCEL
	auto DCEL = domain->getDCEL();
	// Find the face that contains the constraint location.
	auto face = DCEL->findPoint(constrainCoordinate);
	// If the face couldn't be found return false
	if (!face)
		return false;
	// Find the closest vertex to constriant.
	auto closestVertex = face->getClosestPoint(constrainCoordinate);
	// If that vertex is already a constraint return false.
	if (closestVertex->constriantType() == DoublyConnectedList::Vertex::Constraints::POINT || closestVertex->constriantType() == DoublyConnectedList::Vertex::Constraints::EDGE ||
		(closestVertex->constriantType() == DoublyConnectedList::Vertex::Constraints::LINE && type == DoublyConnectedList::Vertex::Constraints::POINT))
		return false;
	// Set the constraint type.
	closestVertex->setConstraint(type);
	// Set the new coordinates.
	closestVertex->setCoordinate(constrainCoordinate);
	// Update connections.
	closestVertex->updateConnections();
	return true;
}
bool BasicQuadMesh::AddLineConstraint(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {
		auto DCEL = domain->getDCEL();
		for (auto& lineConstraint : domain->getLineConstraints()) {
			auto startFace = DCEL->findPoint(lineConstraint->getStartCoordinates());
			auto endFace = DCEL->findPoint(lineConstraint->getEndCoordinates());
			auto endVertex = std::make_shared<DoublyConnectedList::Vertex>(lineConstraint->getEndCoordinates().xCoord, lineConstraint->getEndCoordinates().yCoord);
			if (startFace == endFace)
				return false;
			if (!AddConstraint(domain, lineConstraint->getStartCoordinates(), DoublyConnectedList::Vertex::Constraints::LINE))
				return false;
			
			auto iterVert = DCEL->findPoint(lineConstraint->getStartCoordinates())->getClosestPoint(lineConstraint->getStartCoordinates());
			while (true) {
				endFace = DCEL->findPoint(lineConstraint->getEndCoordinates());
				bool isFound = false;
				for (auto hEdge : iterVert->getHalfEdge()) {
					if (hEdge->getIncidentFace() == endFace) {
						isFound = true;
						break;
					}
				}
				if(isFound)
					break;

				auto lineDirection = lineConstraint->getEndCoordinates() - iterVert->getCoordinate();
				auto lineDummyHalfEdge = DoublyConnectedList::HalfEdge(iterVert, endVertex);
				std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge;
				for (auto hEdge : iterVert->getHalfEdge()) {
					if (hEdge->getAngle() > lineDummyHalfEdge.getAngle())
						break;
					halfEdge = hEdge;
				}
				if (!halfEdge)
					halfEdge = iterVert->getHalfEdge().back();

				auto frontEdgeDirection = halfEdge->getPrevHalfEdge()->getOrigin()->getCoordinate() - halfEdge->getNextHalfEdge()->getOrigin()->getCoordinate();
				auto intersecton = RayIntersection(halfEdge->getNextHalfEdge()->getOrigin()->getCoordinate(), frontEdgeDirection, iterVert->getCoordinate(), lineDirection);

				auto dist1 = sqrt((halfEdge->getPrevHalfEdge()->getOrigin()->getCoordinate() - intersecton) * (halfEdge->getPrevHalfEdge()->getOrigin()->getCoordinate() - intersecton));
				auto dist2 = sqrt((halfEdge->getNextHalfEdge()->getOrigin()->getCoordinate() - intersecton) * (halfEdge->getNextHalfEdge()->getOrigin()->getCoordinate() - intersecton));

				if (dist1 < dist2) 
					iterVert = halfEdge->getPrevHalfEdge()->getOrigin();
				else 
					iterVert = halfEdge->getNextHalfEdge()->getOrigin();
				iterVert->setCoordinate(intersecton);
				iterVert->setConstraint(DoublyConnectedList::Vertex::Constraints::LINE);
				iterVert->updateConnections();
			}
			if (!AddConstraint(domain, lineConstraint->getEndCoordinates(), DoublyConnectedList::Vertex::Constraints::LINE))
				return false;
		}
	}
	return true;
}
void BasicQuadMesh::ReturnToCartesian(std::shared_ptr<MeshData::Domain> domain) {
	std::vector<std::pair<double, double>> nodes;
	for (auto corner : domain->getShapePoints())
		nodes.push_back({ corner->getCoordinates().xCoord, corner->getCoordinates().yCoord });
	for (auto vertex : domain->getDCEL()->getVertices())
		vertex->setCoordinate(MapFromNeutralCoordinate(vertex->getCoordinate().xCoord, vertex->getCoordinate().yCoord,
			nodes, [&](double x, double y)
			{return std::vector<double>({
				-0.25 * (1 - x) * (1 - y) * (1 + x + y),
				0.5 * (1 - x) * (1 + x) * (1 - y),
				-0.25 * (1 + x) * (1 - y) * (1 - x + y),
				0.5 * (1 + x) * (1 + y) * (1 - y),
				-0.25 * (1 + x) * (1 + y) * (1 - x - y),
				0.5 * (1 - x) * (1 + x) * (1 + y),
				-0.25 * (1 - x) * (1 + y) * (1 + x - y),
				0.5 * (1 - x) * (1 + y) * (1 - y) }); }));

	// update face propertiess
	for (auto face : domain->getDCEL()->getFaces()) {
		face->updateProperties();
	}
}
void BasicQuadMesh::FindOptimumEdgeLength(const std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	this->m_EdgeDivideNum.clear();
	std::vector<long long> constraintLengthsDomains;
	for (auto domain : domains) {
		// this vector holds the constreaint distances for EDGE 1
		std::vector<double> constraintLengths1;
		// this vector holds the constreaint distances for EDGE 2
		std::vector<double> constraintLengths2;
		// this vector holds the constreaint distances for EDGE 3
		std::vector<double> constraintLengths3;
		// this vector holds the constreaint distances for EDGE 4
		std::vector<double> constraintLengths4;

		unsigned short edgeID = 1;
		auto addConstraintLength = [&](double length) {
			switch (edgeID) {
			case 1:
				constraintLengths1.push_back(length);
				break;
			case 2:
				constraintLengths2.push_back(length);
				break;
			case 3:
				constraintLengths3.push_back(length);
				break;
			case 4:
				constraintLengths4.push_back(length);
				break;
			}
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
		// optimum edge length candidate for edge 1
		auto edgeLength1 = ApproximateGCD(constraintLengths1);
		// optimum edge length candidate for edge 2
		auto edgeLength2 = ApproximateGCD(constraintLengths2);
		// optimum edge length candidate for edge 3
		auto edgeLength3 = ApproximateGCD(constraintLengths3);
		// optimum edge length candidate for edge 4
		auto edgeLength4 = ApproximateGCD(constraintLengths4);

		// find the least common divison numbers for egde1-3.
		auto candidateDivideNumberForEdge13 = std::lround(ApproximateLCM({ domain->getEdges().at(0)->getLength() / edgeLength1, domain->getEdges().at(2)->getLength() / edgeLength3 }));
		// find the least common divison numbers for egde2-4.
		auto candidateDivideNumberForEdge24 = std::lround(ApproximateLCM({ domain->getEdges().at(1)->getLength() / edgeLength2, domain->getEdges().at(3)->getLength() / edgeLength4 }));

		edgeLength1 = domain->getEdges().at(0)->getLength() / candidateDivideNumberForEdge13;
		edgeLength2 = domain->getEdges().at(1)->getLength() / candidateDivideNumberForEdge24;
		edgeLength3 = domain->getEdges().at(2)->getLength() / candidateDivideNumberForEdge13;
		edgeLength4 = domain->getEdges().at(3)->getLength() / candidateDivideNumberForEdge24;

		if (!(this->m_OptimumLength > edgeLength1))
			edgeLength1 = edgeLength1 / ceil(edgeLength1 / this->m_OptimumLength);
		if (!(this->m_OptimumLength > edgeLength2))
			edgeLength2 = edgeLength2 / ceil(edgeLength2 / this->m_OptimumLength);
		if (!(this->m_OptimumLength > edgeLength3))
			edgeLength3 = edgeLength3 / ceil(edgeLength3 / this->m_OptimumLength);
		if (!(this->m_OptimumLength > edgeLength4))
			edgeLength4 = edgeLength4 / ceil(edgeLength4 / this->m_OptimumLength);

		double divideNumberForEdge13 = domain->getEdges().at(0)->getLength() / edgeLength1 < domain->getEdges().at(2)->getLength() / edgeLength3 ? domain->getEdges().at(2)->getLength() / edgeLength1 : domain->getEdges().at(0)->getLength() / edgeLength3;
		double divideNumberForEdge24 = domain->getEdges().at(1)->getLength() / edgeLength2 < domain->getEdges().at(3)->getLength() / edgeLength4 ? domain->getEdges().at(3)->getLength() / edgeLength2 : domain->getEdges().at(1)->getLength() / edgeLength4;

		// TODO if there are more than one domain than change this part to have compatible divide numbers at the intersection edges.
		// add the divide numbers to the vector.
		this->m_EdgeDivideNum.push_back({ divideNumberForEdge13 , divideNumberForEdge24 });
	}
}
void BasicQuadMesh::TriangleToQuad(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {
		// Get the generated DCEL.
		auto DCEL = domain->getDCEL();
		// Generate dual graph.
		auto graph = this->GenerateGraph(domain);
		auto numberOfVertices = domain->getDCEL()->getFaces().size() - 1;
		std::vector<boost::graph_traits< my_graph >::vertex_descriptor > mate(
			numberOfVertices);
		// Find the matching that has maximum total weight of the grqph.
		maximum_weighted_matching(graph, &mate[0]);
		std::unordered_set<int> vertices;

		// Iterate over all of the found matchings.
		for (size_t i = 0; i < mate.size(); i++) {
			// If the matching already proessed or if there is no matchinf continue.
			if (vertices.find(i) != vertices.end() || (mate[i] > (mate.size() - 1)))
				continue;
			// Get the faces that has matching.
			auto face1 = graph[i].face;
			auto face2 = graph[mate[i]].face;

			vertices.insert(mate[i]);

			// Find the common edge between them to delete.
			std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge;
			auto hEdge = face1->getHalfEdge();
			do {
				if (hEdge->getTwinHalfEdge()->getIncidentFace() == face2) {
					halfEdge = hEdge;
					break;
				}
				hEdge = hEdge->getNextHalfEdge();
			} while (hEdge != face1->getHalfEdge());

			// If the edge is found delete it.
			if(halfEdge)
				DCEL->deleteEdge(halfEdge);
		}
	}
}
my_graph BasicQuadMesh::GenerateGraph(std::shared_ptr<MeshData::Domain> domain) {
	auto DCEL = domain->getDCEL();
	std::unordered_set<std::shared_ptr<DoublyConnectedList::HalfEdge>> hEdges;
	std::unordered_map<std::shared_ptr<DoublyConnectedList::Face>, vertex_t> vertices;

	// Assing the weight of the given edge.
	auto assignWeight = [this](std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) {
		// If the edge is line constraint then assign really low weight value to ensure that this edge won't be deleted.
		if (halfEdge->getOrigin()->constriantType() == DoublyConnectedList::Vertex::Constraints::LINE &&
			halfEdge->getTwinHalfEdge()->getOrigin()->constriantType() == DoublyConnectedList::Vertex::Constraints::LINE)
			return -1000000.0;

		std::vector<double> internalAngles;

		// Find the angle between the given halfEdge and the previous of it.
		auto getAngle = [](std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge) {
			auto angle1 = halfEdge->getAngle();
			auto prevHalfEdge = halfEdge->getPrevHalfEdge();
			auto angle2 = prevHalfEdge->getAngle();
			auto angle = PI - angle1 + angle2;
			while (angle > 2 * PI)
				angle -= 2 * PI;

			return angle * 180.0 / PI;
		};

		// Calculate the internal angles of the possible quad. 
		internalAngles.push_back(getAngle(halfEdge) + getAngle(halfEdge->getTwinHalfEdge()->getNextHalfEdge()));
		internalAngles.push_back(getAngle(halfEdge->getNextHalfEdge()) + getAngle(halfEdge->getTwinHalfEdge()));
		internalAngles.push_back(getAngle(halfEdge->getNextHalfEdge()->getNextHalfEdge()));
		internalAngles.push_back(getAngle(halfEdge->getTwinHalfEdge()->getNextHalfEdge()->getNextHalfEdge()));

		// If one of the internal angle of the quad that is going to be formed when the edge is delete is higher than the 140 degrees don't delete it.
		// In order to ensure that the edge won't be deleted assing really low value.
		std::sort(internalAngles.begin(), internalAngles.end());

		if (internalAngles[3] > 145.0)
			return -10000000.0;

		// Assing weight of the edge by looking at the standard deviation of the internal angles of candidate quad.
		auto stdDeviation = this->StandardDeviation(internalAngles);
		return 200000.0 /pow((1 + stdDeviation),2 );
	};

	// The number of vertices of the graph will be equal to the number of faces except the external face.
	my_graph graph;
	for (auto face : DCEL->getFaces()) {
		if (face->isExternal())
			continue;
		vertex_t newVertex = boost::add_vertex(graph);
		graph[newVertex].face = face;
		vertices[face] = newVertex;
	}
	for (auto hEdge : DCEL->getHalfEdges()) {
		auto twinHEdge = hEdge->getTwinHalfEdge();
		if (hEdge->getIncidentFace()->isExternal() || twinHEdge->getIncidentFace()->isExternal() ||
			(hEdges.find(hEdge) != hEdges.end()) || (hEdges.find(twinHEdge) != hEdges.end()))
			continue;
		hEdges.insert(hEdge);
		hEdges.insert(twinHEdge);
		boost::add_edge(vertices[hEdge->getIncidentFace()], vertices[twinHEdge->getIncidentFace()], EdgeProperty(assignWeight(hEdge)), graph);
	}
	return graph;
}
bool BasicQuadMesh::CheckQualityOfMesh(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {

	}
	return true;
}

void BasicQuadMesh::ClearDomain(std::vector<std::shared_ptr<MeshData::Domain>>& domains) {
	for (auto domain : domains) {
		domain->reset();
	}
}

DoublyConnectedList::Vertex::Coordinates BasicQuadMesh::RayIntersection(const DoublyConnectedList::Vertex::Coordinates& origin1, DoublyConnectedList::Vertex::Coordinates& direction1,
	const DoublyConnectedList::Vertex::Coordinates& origin2, DoublyConnectedList::Vertex::Coordinates& direction2) {
	// normalize the direction vectors
	auto dir1 = direction1.normalize();
	auto dir2 = direction2.normalize();
	auto offsetOrigin1 = origin1 - dir1;
	auto offsetOrigin2 = origin2 - dir2;
	// Caulculate the distance between the intersection point and the origin 2 by solving the intersection
	// equation between 2 rays.
	double distanceBetweenTheIntersectionAndOrigin2;
	if (abs(dir1.xCoord) < 10e-15)
		distanceBetweenTheIntersectionAndOrigin2 = (offsetOrigin1.xCoord - offsetOrigin2.xCoord) / dir2.xCoord;
	else
		distanceBetweenTheIntersectionAndOrigin2 = (offsetOrigin1.yCoord + dir1.yCoord * (offsetOrigin2.xCoord - offsetOrigin1.xCoord) / dir1.xCoord - offsetOrigin2.yCoord) /
		(dir2.yCoord - dir2.xCoord * dir1.yCoord / dir1.xCoord);

	auto result = offsetOrigin2 + dir2 * distanceBetweenTheIntersectionAndOrigin2;
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
double BasicQuadMesh::ApproximateGCD(const std::vector<double>& nums){

	auto meanSin = [&nums](double period) {
		double sum = 0.0;
		for (auto num : nums)
			sum += sin(2.0 * PI * num / period) / nums.size();
		return sum;
	};

	auto meanCos = [&nums](double period) {
		double sum = 0.0;
		for (auto num : nums)
			sum += cos(2.0 * PI * num / period) / nums.size();
		return sum;
	};

	auto gcdAppeal = [&meanCos, &meanSin](double period) {
		return (1.0 - 0.5 * sqrt(pow(meanSin(period), 2) + pow(meanCos(period) - 1, 2)));
	};

	double iterator = *std::min_element(nums.begin(), nums.end());

	while (iterator >= 0) {
		if (gcdAppeal(iterator) > 0.99)
			break;
		iterator -= 0.000001;
	}

	return iterator;
}
double BasicQuadMesh::ApproximateLCM(const std::vector<double>& nums) {
	double multiply = 1.0;
	for (auto num : nums)
		multiply *= num;
	return multiply / ApproximateGCD(nums);
}
double BasicQuadMesh::StandardDeviation(const std::vector<double> nums) {
	double sum = std::accumulate(nums.begin(), nums.end(), 0.0);
	double mean = sum / nums.size();

	std::vector<double> diff(nums.size());
	std::transform(nums.begin(), nums.end(), diff.begin(), [mean](double x) { return x - mean; });
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	return std::sqrt(sq_sum / nums.size());
}