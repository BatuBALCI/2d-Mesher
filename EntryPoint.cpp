#include "mesh.h"
typedef DoublyConnectedList::Vertex::Coordinates coord;
int main()
{
	std::vector<std::vector<double>> verts = { {0.0 ,0.0}, {1.0 ,0.0} , {2.0 ,0.0} , {0.0 ,1.0}, {1.0 ,1.0} , {2.0 ,1.0} , {0.0 ,2.0}, {1.0 ,2.0} , {2.0 ,2.0} };
	std::vector<std::vector<int>> edges = { {0 ,1}, {1 ,2}, {3 ,4}, {4 ,5}, {6, 7}, {7 ,8}, {0 ,3}, {1 ,4}, {2 ,5}, {3 ,6}, {4 ,7}, {5 ,8} };

	auto vertex = std::make_shared<DoublyConnectedList::Vertex>(2, 1.49);

	auto a = DoublyConnectedList::DCEL(verts, edges);
	auto vertices = a.getVertices();
	auto hEdges = a.getHalfEdges();
	auto faces = a.getFaces();

	auto hEdge = hEdges[20];
	a.deleteEdge(hEdge);
	vertices = a.getVertices();
	hEdges = a.getHalfEdges();
	faces = a.getFaces();

	a.addEdge(4, 8);
	vertices = a.getVertices();
	hEdges = a.getHalfEdges();
	faces = a.getFaces();

	a.addVertex(2.0, 1.5);
	vertices = a.getVertices();
	hEdges = a.getHalfEdges();
	faces = a.getFaces();

	a.addVertex(1.75, 1.75);
	vertices = a.getVertices();
	hEdges = a.getHalfEdges();
	faces = a.getFaces();

	a.addVertex(0.5, 0.5, { 0,1,3,4 });
	vertices = a.getVertices();
	hEdges = a.getHalfEdges();
	faces = a.getFaces();

	a.addVertex(0.5, 0.75, { 3,4 });
	vertices = a.getVertices();
	hEdges = a.getHalfEdges();
	faces = a.getFaces();

	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(10.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(10.0, 10.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(0, 10.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	auto cons1 = std::make_shared<MeshData::Domain::EdgeConstraint>(std::vector<double>{0.2});
	auto cons2 = std::make_shared<MeshData::Domain::EdgeConstraint>(std::vector<double>{ 0.3, 0.8 });
	auto cons3 = std::make_shared<MeshData::Domain::EdgeConstraint>(std::vector<double>{ 0.3, 0.70 });
	auto cons4 = std::make_shared<MeshData::Domain::EdgeConstraint>(std::vector<double>{ 0.5 });
	edge1->addEdgeConstraint(cons1);
	edge2->addEdgeConstraint(cons2);
	edge3->addEdgeConstraint(cons3);
	edge4->addEdgeConstraint(cons4);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(5.0, 2.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(10.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(10.0, 5.0, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(10.0, 10.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(5.0, 12.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 10.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 5.0, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	//domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(4.65, 6.78));
	//domain->addLineConstraint(std::make_shared<MeshData::Domain::LineConstraint>(0.6, 8.5, 7.65, 9.78));
	domain->setEdgeLength(1.0);

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();

	mesh.Mesh(domains);
	auto res = a.findPoint(vertex->getCoordinate());
	std::shared_ptr<DoublyConnectedList::Vertex> close;
	if(res)
		close = res->getClosestPoint(vertex->getCoordinate());
	a.ExportVTKFormat("asd.vtk");
}