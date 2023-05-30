#pragma once
#include "DoublyConnectedEdgeList.h"

namespace MeshData
{
	class Domain
	{
		class EdgeConstraint;
	public:
		Domain() = default;

		class Corner
		{
		public:
			Corner() = default;
			Corner(double x, double y, double ksi, double eta);
			std::shared_ptr<DoublyConnectedList::Vertex> getCornerVertex() const;
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates() const;
			bool operator==(const Corner& edge) const { return (this->m_CornerVertex == edge.getCornerVertex() && this->m_CornerVertex == edge.getCornerVertex()); }
		private:
			DoublyConnectedList::Vertex::Coordinates m_Coordinates;
			std::shared_ptr<DoublyConnectedList::Vertex> m_CornerVertex = nullptr;
		};

		class Edge
		{
		public:
			Edge() = default;
			Edge(std::shared_ptr<Corner> start, std::shared_ptr<Corner> end);
			std::shared_ptr<Corner> getStartCorner() const;
			std::shared_ptr<Corner> getEndCorner() const;
			void addEdgeConstraint(std::shared_ptr<EdgeConstraint> edgeConstraint);
			std::shared_ptr<EdgeConstraint> getEdgeConstraint() const;
			double getLength() const;
			double calculateLength();
			bool operator==(const Edge& edge) const { return (this->m_StartCorner == edge.getStartCorner() && this->m_EndCorner == edge.getEndCorner()); }
		private:
			std::shared_ptr<EdgeConstraint> m_EdgeConstraint = nullptr;
			std::shared_ptr<Corner> m_StartCorner = nullptr;
			std::shared_ptr<Corner> m_EndCorner = nullptr;
			double m_Length;
		};

		class PointConstraint
		{
		public:
			PointConstraint() = default;
			PointConstraint(double x, double y);
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates() const;
		private:
			DoublyConnectedList::Vertex::Coordinates m_Coordinates;
		};

		class LineConstraint
		{
		public:
			LineConstraint() = default;
			LineConstraint(double startX, double startY, double endX, double endY);
			const DoublyConnectedList::Vertex::Coordinates& getStartCoordinates() const;
			const DoublyConnectedList::Vertex::Coordinates& getEndCoordinates() const;
		private:
			DoublyConnectedList::Vertex::Coordinates m_StartCoord;
			DoublyConnectedList::Vertex::Coordinates m_EndCoord;
		};

		class EdgeConstraint
		{
		public:
			EdgeConstraint() = default;
			EdgeConstraint(const std::vector<double>& realtiveLocations);
			void addRelativeConstraintLocations(double relativeConstraintLocation);
			const std::vector<double>& getRelativeConstraintLocations() const;
		private:
			std::vector<double> m_RelativeConstraintLocations;
		};

		void addCorner(std::shared_ptr<Corner> corner);
		void addEdge(std::shared_ptr<Edge> edge);
		void addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint);
		void addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint);
		void setEdgeLength(double length);
		void setAspectRaito(double aspectRatio);
		void generateDCEL(const std::vector<std::shared_ptr<DoublyConnectedList::Vertex>>& vertexInput, const std::vector<std::vector<int>>& edgeInput);

		const std::vector<std::shared_ptr<Corner>>& getCorners() const;
		const std::vector<std::shared_ptr<Edge>>& getEdges() const;
		const std::vector<std::shared_ptr<PointConstraint>>& getPointConstriants() const;
		const std::vector<std::shared_ptr<LineConstraint>>& getLineConstraints() const;
		std::shared_ptr<DoublyConnectedList::DCEL> getDCEL();
		double getEdgeLength() const;
		double getAspectRaito() const;
	private:
		double m_EdgeLength = 1.0;
		double m_AspectRatio = 0.5;
		std::vector<std::shared_ptr<Corner>> m_Corners;
		std::vector<std::shared_ptr<Edge>> m_Edges;
		std::vector<std::shared_ptr<PointConstraint>> m_PointConstraints;
		std::vector<std::shared_ptr<LineConstraint>> m_LineConstraints;
		std::shared_ptr<DoublyConnectedList::DCEL> m_DCEL = nullptr;
	};
}
