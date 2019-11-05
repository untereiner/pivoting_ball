#pragma once

#include <cgogn/core/cmap/cmap0.h>
#include <cgogn/core/cmap/cmap2.h>
#include <Eigen/Dense>

using CMap0 = cgogn::CMap0;
using CMap2 = cgogn::CMap2;
using Vec3 = Eigen::Vector3d;

struct FrontEdge
{
	uint32_t startVertexId;
	uint32_t endVertexId;
	uint32_t oppositeVertexId;
	Vec3 ballCenter;
	FrontEdge* previous;
	FrontEdge* next;
};

class PivotingBallNaive1
{
private:

	bool tagPoints = true;

	float ballRadius;
	std::vector<Vec3> pointsPosition;
	std::vector<bool> pointsUsed;
	std::vector<Vec3> triangles;

	FrontEdge* frontFirst;
	FrontEdge* frontLast;

	bool FindSphere(Vec3 p0, Vec3 p1, Vec3 p2, Vec3& center);
	
	float OrientedAngleRad(Vec3 p, Vec3 q, Vec3 &axis);

	FrontEdge* AddEdgeToFront(uint32_t startVertexId, uint32_t endVertexId, uint32_t oppositeVertexId, Vec3 ball_center);

	void RemoveEdgeFromFront(FrontEdge* edge);

	void PushTriangle(uint32_t point0, uint32_t point1, uint32_t point2);

	FrontEdge* FindEdgeOnFront(uint32_t point0, uint32_t point1);

	void JoinOrGlueEdge(uint32_t point0, uint32_t point1, uint32_t oppositeVertexId, Vec3 ballCenter);

public:
	void getSurface
	(
		CMap0& pointSet,
		CMap0::VertexAttribute<Vec3>& pointSetPositions,
		CMap2& surface,
		CMap2::VertexAttribute<Vec3>& surfacePositions,
		CMap2::VertexAttribute<Vec3>& surfaceColors,
		float ballRadius
	);
};