#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <cgogn/core/cmap/cmap0.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/rendering/drawer.h>

using CMap0 = cgogn::CMap0;
using CMap2 = cgogn::CMap2;
using Vec3 = Eigen::Vector3d;

class PivotingBall0
{
	private: 
		std::vector<Vec3> pointsPosition;
		std::vector<bool> pointsUsed;
		std::vector<Vec3> triangles;
		std::vector<uint32_t> front;
		double ballRadius; 

		CMap2* surface; 
		CMap2::VertexAttribute<Vec3>* surfacePositions;

		void push_to_front(uint32_t edgeStart, uint32_t edgeEnd, uint32_t edgeDirection);

		void push_triangle(uint32_t point0, uint32_t point1, uint32_t point2, bool pushEdge0, bool pushEdge1, bool pushEdge2);

		double edgePointDistance(Vec3 edgeStart, Vec3 edgeEnd, Vec3 point);

		Vec3 getEdgeNormal(Vec3 edgeStart, Vec3 edgeEnd, Vec3 otherPoint);

	public: 
		void Initialize
		(
			CMap0& pointSet,
			CMap0::VertexAttribute<Vec3>& pointSetPositions,
			CMap0::VertexAttribute<Vec3>& pointNormals,
			CMap2& surface,
			CMap2::VertexAttribute<Vec3>& surfacePositions
		);

		void SetRadius(double newRadius);

		bool FindSeed();

		bool FrontIsDone();

		void OneFrontIteration();

		void Complete(); 

		void Debug(std::unique_ptr<cgogn::rendering::DisplayListDrawer>& drawer);
};