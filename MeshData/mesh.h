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
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates();
		private:
			DoublyConnectedList::Vertex::Coordinates coordinates;
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
			std::shared_ptr<Corner> startCorner;
			std::shared_ptr<Corner> endCorner;
			double length;
		};

		class PointConstraint
		{
		public:
			PointConstraint() = default;
			PointConstraint(double x, double y);
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates();
		private:
			DoublyConnectedList::Vertex::Coordinates coordinates;
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
			DoublyConnectedList::Vertex::Coordinates startCoord;
			DoublyConnectedList::Vertex::Coordinates endCoord;
			std::vector<std::shared_ptr<PointConstraint>> pointConstraints;
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
			std::shared_ptr<Edge> edge;
			std::vector<double> relativeConstraintLocations;
		};

		void addCorner(std::shared_ptr<Corner> corner);
		void addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint);
		void addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint);

		const std::vector<std::shared_ptr<Corner>>& getCorners();
		const std::vector<std::shared_ptr<PointConstraint>>& getPointConstriants();
		const std::vector<std::shared_ptr<LineConstraint>>& getLineConstraints();
		const std::vector<std::shared_ptr<EdgeConstraint>>& getEdgeConstraints();
		std::shared_ptr<DoublyConnectedList::DCEL> getDCEL();
	private:
		std::vector<std::shared_ptr<Corner>> corners;
		std::vector<std::shared_ptr<PointConstraint>> pointConstraints;
		std::vector<std::shared_ptr<LineConstraint>> lineConstraints;
		std::vector<std::shared_ptr<EdgeConstraint>> edgeConstraints;
		std::shared_ptr<DoublyConnectedList::DCEL> DCEL;
	};
}
