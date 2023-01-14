#pragma once
#include "DoublyConnectedEdgeList.h"

namespace MeshData
{
	class Domain
	{
	public:
		Domain() = default;

		class Corner
		{
		public:
			Corner() = default;
			Corner(double x, double y);
			std::shared_ptr<DoublyConnectedList::Vertex> getCornerVertex();
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates();
		private:
			DoublyConnectedList::Vertex::Coordinates m_Coordinates;
			std::shared_ptr<DoublyConnectedList::Vertex> m_CornerVertex;;
		};

		class Edge
		{
		public:
			Edge() = default;
			Edge(std::shared_ptr<Corner> start, std::shared_ptr<Corner> end);
			std::shared_ptr<Corner> getStartCorner();
			std::shared_ptr<Corner> getEndCorner();
			double getLength();
			double calculateLength();
		private:
			std::shared_ptr<Corner> m_StartCorner;
			std::shared_ptr<Corner> m_EndCorner;
			double m_Length;
		};

		class PointConstraint
		{
		public:
			PointConstraint() = default;
			PointConstraint(double x, double y);
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates();
		private:
			DoublyConnectedList::Vertex::Coordinates m_Coordinates;
		};

		class LineConstraint
		{
		public:
			LineConstraint() = default;
			LineConstraint(double startX, double startY, double endX, double endY);
			const DoublyConnectedList::Vertex::Coordinates& getStartCoordinates();
			const DoublyConnectedList::Vertex::Coordinates& getEndCoordinates();
			void addPointConstraint(std::shared_ptr<PointConstraint> pointConstraint);
			const std::vector<std::shared_ptr<PointConstraint>>& getConstraints();
		private:
			DoublyConnectedList::Vertex::Coordinates m_StartCoord;
			DoublyConnectedList::Vertex::Coordinates m_EndCoord;
			std::vector<std::shared_ptr<PointConstraint>> m_PointConstraints;
		};

		class EdgeConstraint
		{
		public:
			EdgeConstraint() = default;
			EdgeConstraint(std::shared_ptr<Edge> edge);
			std::shared_ptr<Edge> getEdge();
			void addRelativeConstraintLocations(double relativeConstraintLocation);
			const std::vector<double>& getRelativeConstraintLocations();
		private:
			std::shared_ptr<Edge> m_Edge;
			std::vector<double> m_RelativeConstraintLocations;
		};

		void addCorner(std::shared_ptr<Corner> corner);
		void addEdge(std::shared_ptr<Edge> edge);
		void addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint);
		void addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint);
		void addEdgeConstraint(std::shared_ptr<EdgeConstraint> edgeConstraint);

		const std::vector<std::shared_ptr<Corner>>& getCorners();
		const std::vector<std::shared_ptr<Edge>>& getEdges();
		const std::vector<std::shared_ptr<PointConstraint>>& getPointConstriants();
		const std::vector<std::shared_ptr<LineConstraint>>& getLineConstraints();
		const std::vector<std::shared_ptr<EdgeConstraint>>& getEdgeConstraints();
		std::shared_ptr<DoublyConnectedList::DCEL> getDCEL();
		
		double m_EdgeLength;
	private:
		std::vector<std::shared_ptr<Corner>> m_Corners;
		std::vector<std::shared_ptr<Edge>> m_Edges;
		std::vector<std::shared_ptr<PointConstraint>> m_PointConstraints;
		std::vector<std::shared_ptr<LineConstraint>> m_LineConstraints;
		std::vector<std::shared_ptr<EdgeConstraint>> m_EdgeConstraints;
		std::shared_ptr<DoublyConnectedList::DCEL> m_DCEL;
	};
}
