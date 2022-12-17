#pragma once
#include "DoublyConnectedEdgeList.h"

namespace MeshData
{
	class Domain
	{
	public:
		Domain() = default;

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

		class Corner
		{
		public:
			Corner() = default;
			Corner(double x, double y);
			const DoublyConnectedList::Vertex::Coordinates& getCoordinates();
		private:
			DoublyConnectedList::Vertex::Coordinates coordinates;
		};

		void addCorner(std::shared_ptr<Corner> corner);
		void addPointConstriant(std::shared_ptr<PointConstraint> pointConstraint);
		void addLineConstraint(std::shared_ptr<LineConstraint> lineConstraint);

		const std::vector<std::shared_ptr<Corner>>& getCorners();
		const std::vector<std::shared_ptr<PointConstraint>>& getPointConstriants();
		const std::vector<std::shared_ptr<LineConstraint>>& getLineConstraints();
		std::shared_ptr<DoublyConnectedList::DCEL> getDCEL();
	private:
		std::vector<std::shared_ptr<Corner>> corners;
		std::vector<std::shared_ptr<PointConstraint>> pointConstraints;
		std::vector<std::shared_ptr<LineConstraint>> lineConstraints;
		std::shared_ptr<DoublyConnectedList::DCEL> DCEL;
	};
}