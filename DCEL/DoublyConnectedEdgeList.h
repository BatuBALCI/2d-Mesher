#pragma once
#define pi 3.14159265358979323846
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <math.h>
#include <functional>
#include <fstream>

namespace DoublyConnectedList
{
	class Face;
	class Vertex;
	class HalfEdge;
	class DCEL;
	
	class Vertex
	{
	public:
		struct Coordinates
		{
			double xCoord;
			double yCoord;
		};
		Vertex() = default;
		Vertex(double x, double y);
		Vertex(double x, double y, int ID);
		void setID(int ID);
		const std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>& getHalfEdge();
		int getID();
		void addHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge);
		void angleSortEdges();
		void prevAndNextAssignments();
		const Coordinates& getCoordinate();
	private:
		int m_ID;
		// Holds the half edges that this vertex is their origin.
		std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>> m_HalfEdges;
		// Holds the coordinate info of the vertex
		struct Coordinates m_Coordinates;

		friend class DoublyConnectedList::DCEL;
		void deleteEdgeFromList(std::shared_ptr<DoublyConnectedList::HalfEdge> edge);
	};

	class HalfEdge
	{
	public:
		HalfEdge() = default;
		HalfEdge(std::shared_ptr<DoublyConnectedList::Vertex> point1, 
			std::shared_ptr<DoublyConnectedList::Vertex> point2);
		void setNextHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> next);
		void setPrevHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> prev);
		void setTwinHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> twin);
		void setIncidentFace(std::shared_ptr<DoublyConnectedList::Face> incidentFace);
		std::shared_ptr<HalfEdge> getNextHalfEdge();
		std::shared_ptr<HalfEdge> getPrevHalfEdge();
		std::shared_ptr<HalfEdge> getTwinHalfEdge();
		std::shared_ptr<DoublyConnectedList::Face> getIncidentFace();
		std::shared_ptr<DoublyConnectedList::Vertex> getOrigin();
		double getAngle();
		double getLength();
		double determinant(const std::vector<double>& point1, const std::vector<double>& point2);
		bool isPointOnLeft(std::shared_ptr<DoublyConnectedList::Vertex> point);
		bool isPointOn(std::shared_ptr<DoublyConnectedList::Vertex> point);
		std::shared_ptr<DoublyConnectedList::Face> faceAssignmentToEdges(std::shared_ptr<DoublyConnectedList::HalfEdge> itself);
	private:
		std::shared_ptr<DoublyConnectedList::HalfEdge> m_Next = nullptr;
		std::shared_ptr<DoublyConnectedList::HalfEdge> m_Prev = nullptr;
		std::shared_ptr<DoublyConnectedList::HalfEdge> m_Twin = nullptr;
		std::shared_ptr<DoublyConnectedList::Face> m_IncidentFace = nullptr;
		std::shared_ptr<DoublyConnectedList::Vertex> m_Origin = nullptr;
		double m_Angle;
		double m_Length;
		// Determines the angle with respect to the x axis of a segment
		// of coordinates dx and dy
		void calculateProperties(std::shared_ptr<DoublyConnectedList::Vertex> point1,
			std::shared_ptr<DoublyConnectedList::Vertex> point2);
	};
	
	class Face
	{
	public:
		Face() = default;
		double calculateArea();
		double calculatePerimeter();
		void calculateEdgeCount();
		int getEdgeCount();
		void setHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge);
		std::shared_ptr<DoublyConnectedList::HalfEdge> getHalfEdge();
		bool isInside(std::shared_ptr<DoublyConnectedList::Vertex> point);
		std::shared_ptr<DoublyConnectedList::HalfEdge> isOn(std::shared_ptr<DoublyConnectedList::Vertex> point);
		std::shared_ptr<DoublyConnectedList::Vertex> getClosestPoint(std::shared_ptr<DoublyConnectedList::Vertex> vert);
		void setIfExternal(bool isExt);
		bool isExternal();
		void clearFaceAssignments();
	private:
		std::shared_ptr<DoublyConnectedList::HalfEdge> m_HalfEdgeComponent = nullptr;
		bool m_IsTheFaceExternal = false;
		double m_Area;
		double m_Perimeter;
		int m_EdgeCount;
	};

	class DCEL
	{
	public:
		DCEL();
		DCEL(std::vector<std::vector<double>> vertexInput, std::vector<std::vector<int>> edgeInput);
		size_t getNumberOfVertices();
		size_t getNumberOfHalfEdges();
		size_t getNumberOfFaces();
		const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& getVertices();
		const std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>& getHalfEdges();
		const std::vector<std::shared_ptr<DoublyConnectedList::Face>>& getFaces();
		void updateVertexIds();
		std::shared_ptr<DoublyConnectedList::Face> findPoint(std::shared_ptr<DoublyConnectedList::Vertex> point);
		void addEdge(int vertId1, int vertId2);
		void deleteEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> edge);
		void addVertex(double xCoord, double yCoord, std::vector<int> verticesToConnect = {});
		void ExportVTKFormat(std::string filename);
	private:
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> m_Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>> m_HalfEdges;
		std::vector<std::shared_ptr<DoublyConnectedList::Face>> m_Faces;

		void buildDCEL(std::vector<std::vector<double>> vertexInput, std::vector<std::vector<int>> edgeInput);
		void deleteEdgeFromList(std::shared_ptr<DoublyConnectedList::HalfEdge> edge);
		void deleteVertexFromList(int ID);
		void deleteFaceFromList(std::shared_ptr<DoublyConnectedList::Face> face);
		void addVertexOnTheEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> edge, std::shared_ptr<DoublyConnectedList::Vertex> vertex);
		void addVertexInsideTheFace(std::shared_ptr<DoublyConnectedList::Face> face, std::shared_ptr<DoublyConnectedList::Vertex> vertex, std::vector<int> verticesToConnect);
		std::pair<std::shared_ptr<DoublyConnectedList::HalfEdge>, std::shared_ptr<DoublyConnectedList::HalfEdge>> 
			createHalfEdge(std::shared_ptr<DoublyConnectedList::Vertex> vertex1, std::shared_ptr<DoublyConnectedList::Vertex> vertex2);
	};
}