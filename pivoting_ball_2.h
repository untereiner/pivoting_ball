#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <cgogn/core/cmap/cmap0.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/rendering/drawer.h>
#include <nanoflann.hpp>

using CMap0 = cgogn::CMap0;
using CMap2 = cgogn::CMap2;
using Vec3 = Eigen::Vector3d;

typedef Eigen::Matrix<double, Eigen::Dynamic, 3> PointMatrix;
typedef nanoflann::KDTreeEigenMatrixAdaptor<PointMatrix> KDTree;

class PivotingBall2
{
private:
	using Vertex = uint32_t;

	std::list<CMap2::Edge> untriedEdgeList; 
	std::list<CMap2::Edge> triedEdgeList;
	std::list<CMap2::Edge> sewedEdgeList;

	double ballRadius;
	uint32_t pointCount; 
	std::vector<Vec3> pointPositions; 
	std::vector<Vec3> pointNormals; 

	std::vector<std::list<std::list<CMap2::Edge>::iterator>> pointUntriedEdges;
	std::vector<std::list<std::list<CMap2::Edge>::iterator>> pointTriedEdges;
	std::vector<std::list<std::list<CMap2::Edge>::iterator>> pointSewedEdges;

	CMap2* surface; 
	CMap2::VertexAttribute<Vec3>* surfacePositions;
	CMap2::VertexAttribute<Vertex> surfaceVertexes;

	PointMatrix pointsMatrix;
	std::vector<std::pair<Eigen::Index, double>> searchResult;
	std::vector<Vertex> neighbors;
	KDTree* kdtree;

	void GetCircumscribedCircle(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3& center, double& radius);

	Vec3 Normal(Vertex vertex0, Vertex vertex1, Vertex vertex2);

	bool WellOriented(Vertex vertex0, Vertex vertex1, Vertex vertex2);

	void Reorder(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vertex* order);

	bool GetBallCenter(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3& center);

	void AddTriedEdge(CMap2::Edge edge);
	void RemoveTriedEdge(std::list<CMap2::Edge>::iterator edge);
	std::list<CMap2::Edge>::iterator FindTriedEdge(Vertex startVertex, Vertex endVertex);
	std::list<CMap2::Edge>::iterator FindOppositeTriedEdge(CMap2::Edge edge);

	void AddUntriedEdge(CMap2::Edge edge);
	void RemoveUntriedEdge(std::list<CMap2::Edge>::iterator edge);
	std::list<CMap2::Edge>::iterator FindUntriedEdge(Vertex startVertex, Vertex endVertex);
	std::list<CMap2::Edge>::iterator FindOppositeUntriedEdge(CMap2::Edge edge);

	void AddSewedEdge(CMap2::Edge edge);
	void RemoveSewedEdge(std::list<CMap2::Edge>::iterator edge);
	std::list<CMap2::Edge>::iterator FindSewedEdge(Vertex startVertex, Vertex endVertex);

	void JoinOrGlueEdge(CMap2::Edge edge);

	bool ValidTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2);

	CMap2::Face AddTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2);

	bool IsEmpty(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3 ballCenter);

	void GetNeighbors(Vec3 position, float radius);

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