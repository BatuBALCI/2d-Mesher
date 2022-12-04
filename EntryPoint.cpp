#include "DoublyConnectedEdgeList.h"
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

	auto res = a.findPoint(vertex);
	std::shared_ptr<DoublyConnectedList::Vertex> close;
	if(res)
		close = res->getClosestPoint(vertex);
}