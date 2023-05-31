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

			inline Coordinates operator+(const Coordinates& cor) const { return { xCoord + cor.xCoord, yCoord + cor.yCoord }; };
			inline Coordinates operator-(const Coordinates& cor) const { return { xCoord - cor.xCoord, yCoord - cor.yCoord }; };
			inline double operator*(const Coordinates& cor) const { return { xCoord * cor.xCoord + yCoord * cor.yCoord }; };
			inline Coordinates operator*(const double& length) const { return { xCoord * length, yCoord * length }; };
			inline Coordinates operator/(const double& length) const { return { xCoord / length, yCoord / length }; };
			inline Coordinates operator=(const Coordinates& cor) { this->xCoord = cor.xCoord; this->yCoord = cor.yCoord; return *this; };
			inline Coordinates normalize() { return *this/sqrt(this->xCoord * this->xCoord + this->yCoord * this->yCoord); };

		};
		enum Constraints {
			NONE,
			POINT,
			LINE,
			EDGE
		};
		Vertex() = default;
		Vertex(double x, double y);
		Vertex(double x, double y, int ID);
		void setID(int ID);
		void setConstraint(Constraints cons);
		void clearConstraint();
		Constraints constriantType();
		const std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>& getHalfEdge() const;
		void clearHalfEdge();
		int getID() const;
		void addHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge);
		void angleSortEdges();
		void prevAndNextAssignments();
		void setCoordinate(DoublyConnectedList::Vertex::Coordinates coordinate);
		const Coordinates& getCoordinate() const;
		bool operator==(const Vertex& vertex) const { return this->m_ID == vertex.getID(); };
		bool operator==(std::shared_ptr<Vertex> vertex) {  this->getID() == vertex->getID(); }
	private:
		int m_ID;
		// Holds the half edges that this vertex is their origin.
		std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>> m_HalfEdges;
		// Holds the coordinate info of the vertex
		struct Coordinates m_Coordinates;

		friend class DoublyConnectedList::DCEL;
		Constraints m_ConstraintType = Constraints::NONE;
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
		std::shared_ptr<HalfEdge> getNextHalfEdge() const;
		std::shared_ptr<HalfEdge> getPrevHalfEdge() const;
		std::shared_ptr<HalfEdge> getTwinHalfEdge() const;
		std::shared_ptr<DoublyConnectedList::Face> getIncidentFace() const;
		std::shared_ptr<DoublyConnectedList::Vertex> getOrigin() const;
		double getAngle() const;
		double getLength() const;
		void updateProperties();
		double determinant(const std::vector<double>& point1, const std::vector<double>& point2);
		bool isPointOnLeft(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates);
		bool isPointOn(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates);
		std::shared_ptr<DoublyConnectedList::Face> faceAssignmentToEdges(std::shared_ptr<DoublyConnectedList::HalfEdge> itself);
		bool operator==(std::shared_ptr<HalfEdge> halfEdge) { return this->getOrigin()->getID() == halfEdge->getOrigin()->getID(); }
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
		void updateProperties();
		void calculateEdgeCount();
		int getEdgeCount() const;
		void setHalfEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> halfEdge);
		std::shared_ptr<DoublyConnectedList::HalfEdge> getHalfEdge() const;
		bool isInside(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates);
		std::shared_ptr<DoublyConnectedList::HalfEdge> isOn(const DoublyConnectedList::Vertex::Coordinates& pointCooridnate);
		std::shared_ptr<DoublyConnectedList::Vertex> getClosestPoint(const DoublyConnectedList::Vertex::Coordinates& pointCooridnate);
		void setIfExternal(bool isExt);
		bool isExternal();
		void clearFaceAssignments();
		bool operator==(std::shared_ptr<Face> face){ return this->getHalfEdge()->getOrigin()->getID() == face->getHalfEdge()->getOrigin()->getID(); }
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
		DCEL(const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& vertexInput, const std::vector<std::vector<int>>& edgeInput);
		size_t getNumberOfVertices() const;
		size_t getNumberOfHalfEdges() const;
		size_t getNumberOfFaces() const;
		const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& getVertices() const;
		const std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>>& getHalfEdges() const;
		const std::vector<std::shared_ptr<DoublyConnectedList::Face>>& getFaces() const;
		void updateVertexIds();
		std::shared_ptr<DoublyConnectedList::Face> findPoint(const DoublyConnectedList::Vertex::Coordinates& pointCoordinates);
		void addEdge(int vertId1, int vertId2);
		void deleteEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> edge);
		void addVertex(double xCoord, double yCoord, std::vector<int> verticesToConnect = {});
		void addVertex(std::shared_ptr<DoublyConnectedList::Vertex> vertex, std::vector<int> verticesToConnect = {});
		void ExportVTKFormat(std::string filename);
	private:
		std::vector<std::shared_ptr<DoublyConnectedList::Vertex>> m_Vertices;
		std::vector<std::shared_ptr<DoublyConnectedList::HalfEdge>> m_HalfEdges;
		std::vector<std::shared_ptr<DoublyConnectedList::Face>> m_Faces;

		void buildDCEL(std::vector<std::vector<double>> vertexInput, std::vector<std::vector<int>> edgeInput);
		void buildDCEL(const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& vertexInput, const std::vector<std::vector<int>>& edgeInput);
		void deleteEdgeFromList(std::shared_ptr<DoublyConnectedList::HalfEdge> edge);
		void deleteVertexFromList(int ID);
		void deleteFaceFromList(std::shared_ptr<DoublyConnectedList::Face> face);
		void addVertexOnTheEdge(std::shared_ptr<DoublyConnectedList::HalfEdge> edge, std::shared_ptr<DoublyConnectedList::Vertex> vertex);
		void addVertexInsideTheFace(std::shared_ptr<DoublyConnectedList::Face> face, std::shared_ptr<DoublyConnectedList::Vertex> vertex, std::vector<int> verticesToConnect);
		std::pair<std::shared_ptr<DoublyConnectedList::HalfEdge>, std::shared_ptr<DoublyConnectedList::HalfEdge>> 
			createHalfEdge(std::shared_ptr<DoublyConnectedList::Vertex> vertex1, std::shared_ptr<DoublyConnectedList::Vertex> vertex2);
	};
}