#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <cgogn/core/cmap/cmap0.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/rendering/drawer.h>

using CMap0 = cgogn::CMap0;
using CMap2 = cgogn::CMap2;
using Vec3 = Eigen::Vector3d;

class PivotingBall1
{
private:
	using Vertex = uint32_t;

	std::list<CMap2::Edge> front;

	float ballRadius;
	uint32_t pointCount; 
	std::vector<Vec3> pointPositions; 
	std::vector<Vec3> pointNormals; 
	std::vector<std::list<std::list<CMap2::Edge>::iterator>> pointFrontEdges;

	CMap2* surface; 
	CMap2::VertexAttribute<Vec3>* surfacePositions;
	CMap2::VertexAttribute<Vertex> surfaceVertexes;

	void GetCircumscribedCircle(Vec3 p0, Vec3 p1, Vec3 p2, Vec3& center, double& radius);

	bool IsOriented(Vec3 normal, Vec3 normal0, Vec3 normal1, Vec3 normal2);

	bool GetBallCenter(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3& center, Vertex* order);

	void AddFrontEdge(CMap2::Edge edge);

	void RemoveFrontEdge(std::list<CMap2::Edge>::iterator edge);

	std::list<CMap2::Edge>::iterator FindFrontEdge(CMap2::Edge edge);

	void JoinOrGlueEdge(CMap2::Edge edge);

	CMap2::Face AddTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2);

	bool IsEmpty(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3 ballCenter);

	std::vector<Vertex> GetNeighbors(Vec3 position, float radius);

public:

	void Initialize
	(
		CMap0& pointSet,
		CMap0::VertexAttribute<Vec3>& pointSetPositions,
		CMap0::VertexAttribute<Vec3>& pointNormals,
		CMap2& surface,
		CMap2::VertexAttribute<Vec3>& surfacePositions
	);

	void SetRadius(double radius);

	bool FindSeed();

	bool FrontIsDone();

	void OneFrontIteration();

	void Complete();

	void Debug(std::unique_ptr<cgogn::rendering::DisplayListDrawer>& drawer);
};