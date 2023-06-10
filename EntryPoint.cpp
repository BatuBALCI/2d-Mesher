#include "mesh.h"
typedef DoublyConnectedList::Vertex::Coordinates coord;

void spanWithExtremeReversedSkew(double edgeLength, std::function<double(double)> stdFunc) {
	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(15.0, 15.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 0.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(42.5, 7.5, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 15.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(15.0, 15.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(7.5, 7.5, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(25.0, 7.5));
	domain->setEdgeLength(edgeLength);

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();
	mesh.SetStdDeviationFunc(stdFunc);
	mesh.Mesh(domains);
	for (size_t i = 0; i < domains.size(); i++)
		domains[i]->getDCEL()->ExportVTKFormat("spanWithExtremeReversedSkew_" + std::to_string(i+1) + "_" + std::to_string(edgeLength) + ".vtk");
}
void spanWithExtremeParallelSkew(double edgeLength, std::function<double(double)> stdFunc) {
	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(-15.0, 15.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 0.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(42.5, 7.5, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(10.0, 15.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(-15.0, 15.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(-7.5, 7.5, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(17.5, 7.5));
	domain->setEdgeLength(edgeLength);

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();
	mesh.SetStdDeviationFunc(stdFunc);

	mesh.Mesh(domains);
	for (size_t i = 0; i < domains.size(); i++)
		domains[i]->getDCEL()->ExportVTKFormat("spanWithExtremeParallelSkew" + std::to_string(i + 1) + "_" + std::to_string(edgeLength) + ".vtk");
}
void spanWithExtremeCurvedPath(double edgeLength, std::function<double(double)> stdFunc) {
	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(30.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(30.0, 15.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(0.0, 15.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(15.0, 10.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(30.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(30.0, 7.5, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(30.0, 15.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(15.0, 25.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 15.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 7.5, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(25.0, 15.0));
	domain->setEdgeLength(edgeLength);

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();
	mesh.SetStdDeviationFunc(stdFunc);

	mesh.Mesh(domains);
	for (size_t i = 0; i < domains.size(); i++)
		domains[i]->getDCEL()->ExportVTKFormat("spanWithExtremeCurvedPath" + std::to_string(i + 1) + "_" + std::to_string(edgeLength) + ".vtk");
}
void spanWithExtremeCurvedPathAndExtremeReverseSkew(double edgeLength, std::function<double(double)> stdFunc) {
	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(15.0, 15.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 5.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(42.5, 7.5, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 20.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(15.0, 15.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(7.5, 7.5, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(25.0, 12.5));
	domain->setEdgeLength(edgeLength);

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();
	mesh.SetStdDeviationFunc(stdFunc);
	mesh.Mesh(domains);
	for (size_t i = 0; i < domains.size(); i++)
		domains[i]->getDCEL()->ExportVTKFormat("spanWithExtremeCurvedPathAndExtremeReverseSkew" + std::to_string(i + 1) + "_" + std::to_string(edgeLength) + ".vtk");
}
void spanWithExtremeReverseSkewAndLineConstraint(double edgeLength, std::function<double(double)> stdFunc) {
	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(15.0, 15.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 0.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(42.5, 7.5, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(35.0, 15.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 15.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(15.0, 15.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(7.5, 7.5, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(25.0, 7.5));
	domain->setEdgeLength(edgeLength);

	domain->addLineConstraint(std::make_shared<MeshData::Domain::LineConstraint>(4.0, 1.2, 46.0, 1.2));
	domain->addLineConstraint(std::make_shared<MeshData::Domain::LineConstraint>(16.0, 13.2, 34.0, 13.2));

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();
	mesh.SetStdDeviationFunc(stdFunc);
	mesh.Mesh(domains);
	for (size_t i = 0; i < domains.size(); i++)
		domains[i]->getDCEL()->ExportVTKFormat("spanWithExtremeReverseSkewAndLineConstraint" + std::to_string(i + 1) + "_" + std::to_string(edgeLength) + ".vtk");
}
void spanWithCrossLineConstraint(double edgeLength, std::function<double(double)> stdFunc) {
	std::vector<std::shared_ptr<MeshData::Domain>> domains;

	auto domain = std::make_shared<MeshData::Domain>();
	auto corner1 = std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0);
	auto corner2 = std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0);
	auto corner3 = std::make_shared<MeshData::Domain::Corner>(50.0, 15.0, 1.0, 1.0);
	auto corner4 = std::make_shared<MeshData::Domain::Corner>(0.0, 15.0, -1.0, 1.0);
	auto edge1 = std::make_shared<MeshData::Domain::Edge>(corner1, corner2);
	auto edge2 = std::make_shared<MeshData::Domain::Edge>(corner2, corner3);
	auto edge3 = std::make_shared<MeshData::Domain::Edge>(corner3, corner4);
	auto edge4 = std::make_shared<MeshData::Domain::Edge>(corner4, corner1);
	domain->addCorner(corner1);
	domain->addCorner(corner2);
	domain->addCorner(corner3);
	domain->addCorner(corner4);
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 0.0, -1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 0.0, 0.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 0.0, 1.0, -1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 7.5, 1.0, 0.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(50.0, 15.0, 1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(25.0, 15.0, 0.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 15.0, -1.0, 1.0));
	domain->addShapePoint(std::make_shared<MeshData::Domain::Corner>(0.0, 7.5, -1.0, 0.0));
	domain->addEdge(edge1);
	domain->addEdge(edge2);
	domain->addEdge(edge3);
	domain->addEdge(edge4);
	domain->addPointConstriant(std::make_shared<MeshData::Domain::PointConstraint>(25.0, 7.5));

	domain->addLineConstraint(std::make_shared<MeshData::Domain::LineConstraint>(4, 12.8, 16.6, 2.2));
	domain->addLineConstraint(std::make_shared<MeshData::Domain::LineConstraint>(33.4, 2.2, 46.0, 12.8));


	domain->setEdgeLength(edgeLength);

	domains.push_back(domain);

	auto mesh = BasicQuadMesh();
	mesh.SetStdDeviationFunc(stdFunc);
	mesh.Mesh(domains);
	for (size_t i = 0; i < domains.size(); i++)
		domains[i]->getDCEL()->ExportVTKFormat("spanWithCrossLineConstraint" + std::to_string(i + 1) + "_" + std::to_string(edgeLength) + ".vtk");
}

int main()
{
	spanWithExtremeReversedSkew(5.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReversedSkew(4.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReversedSkew(3.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReversedSkew(2.0, [](double stdDeviation) {return 20000000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReversedSkew(1.0, [](double stdDeviation) {return 200000000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReversedSkew(0.9, [](double stdDeviation) {return 200000000 / pow((1 + stdDeviation), 2); });
	
	spanWithExtremeParallelSkew(5.0, [](double stdDeviation) {return 100.0 - stdDeviation;});
	spanWithExtremeParallelSkew(4.0, [](double stdDeviation) {return 80.0 - stdDeviation;});
	spanWithExtremeParallelSkew(3.0, [](double stdDeviation) {return 80.0 - stdDeviation;});
	spanWithExtremeParallelSkew(2.0, [](double stdDeviation) {return 156.0 - stdDeviation;});
	spanWithExtremeParallelSkew(1.0, [](double stdDeviation) {return 156.0 - stdDeviation;});
	spanWithExtremeParallelSkew(0.9, [](double stdDeviation) {return 156.0 - stdDeviation;});
	spanWithExtremeParallelSkew(0.8, [](double stdDeviation) {return 156.0 - stdDeviation;});
	spanWithExtremeParallelSkew(0.7, [](double stdDeviation) {return 156.0 - stdDeviation;});
	
	spanWithExtremeCurvedPath(5.0, [](double stdDeviation) {return 100.0 - stdDeviation; });
	spanWithExtremeCurvedPath(4.0, [](double stdDeviation) {return 156.0 - stdDeviation; });
	spanWithExtremeCurvedPath(3.0, [](double stdDeviation) {return 156.0 - stdDeviation; });
	spanWithExtremeCurvedPath(2.0, [](double stdDeviation) {return 100.0 - stdDeviation; });
	spanWithExtremeCurvedPath(1.0, [](double stdDeviation) {return 156.0 - stdDeviation; });
	spanWithExtremeCurvedPath(0.9, [](double stdDeviation) {return 156.0 - stdDeviation; });
	spanWithExtremeCurvedPath(0.8, [](double stdDeviation) {return 156.0 - stdDeviation; });
	spanWithExtremeCurvedPath(0.7, [](double stdDeviation) {return 156.0 - stdDeviation; });
	spanWithExtremeCurvedPath(0.6, [](double stdDeviation) {return 156.0 - stdDeviation; });
	
	spanWithExtremeCurvedPathAndExtremeReverseSkew(5.0, [](double stdDeviation) {return 29.0 - stdDeviation; });
	spanWithExtremeCurvedPathAndExtremeReverseSkew(4.0, [](double stdDeviation) {return 50.0 - stdDeviation; });
	spanWithExtremeCurvedPathAndExtremeReverseSkew(3.0, [](double stdDeviation) {return 50.0 - stdDeviation; });
	spanWithExtremeCurvedPathAndExtremeReverseSkew(2.0, [](double stdDeviation) {return 65.0 - stdDeviation; });
	spanWithExtremeCurvedPathAndExtremeReverseSkew(1.0, [](double stdDeviation) {return 200000000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeCurvedPathAndExtremeReverseSkew(0.9, [](double stdDeviation) {return 26.0 - stdDeviation; });
	
	spanWithCrossLineConstraint(5.0, [](double stdDeviation){ return 60.0 - stdDeviation; });
	spanWithCrossLineConstraint(4.0, [](double stdDeviation){ return 60.0 - stdDeviation; });
	spanWithCrossLineConstraint(3.0, [](double stdDeviation){ return 40.0 - stdDeviation; });
	spanWithCrossLineConstraint(2.0, [](double stdDeviation){ return 80.0 - stdDeviation; });
	spanWithCrossLineConstraint(1.0, [](double stdDeviation){ return 80.0 - stdDeviation; });
	spanWithCrossLineConstraint(0.9, [](double stdDeviation){ return 80.0 - stdDeviation; });
	spanWithCrossLineConstraint(0.8, [](double stdDeviation){ return 156.0 - stdDeviation; });
	spanWithCrossLineConstraint(0.7, [](double stdDeviation){ return 60.0 - stdDeviation; });
	spanWithCrossLineConstraint(0.6, [](double stdDeviation){ return 60.0 - stdDeviation; });
	
	spanWithExtremeReverseSkewAndLineConstraint(5.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReverseSkewAndLineConstraint(4.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReverseSkewAndLineConstraint(3.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReverseSkewAndLineConstraint(2.0, [](double stdDeviation) {return 20000000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReverseSkewAndLineConstraint(1.0, [](double stdDeviation) {return 200000 / pow((1 + stdDeviation), 2); });
	spanWithExtremeReverseSkewAndLineConstraint(0.9, [](double stdDeviation) {return 32.0 - stdDeviation; });

}