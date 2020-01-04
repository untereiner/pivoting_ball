#include "pivoting_ball_2.h"

#define COMPARISON_EPSILON	1e-10
#define IN_BALL_THRESHOLD	1e-7

template <typename T> int sign(T val) 
{
	return (T(0) < val) - (val < T(0));
}

void PivotingBall2::GetCircumscribedCircle(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3& center, double& radius)
{
	Vec3 p0 = pointPositions[vertex0];
	Vec3 p1 = pointPositions[vertex1];
	Vec3 p2 = pointPositions[vertex2];

	Vec3 d10 = p1 - p0;
	Vec3 d20 = p2 - p0;
	Vec3 d01 = p0 - p1;
	Vec3 d12 = p1 - p2;
	Vec3 d21 = p2 - p1;
	Vec3 d02 = p0 - p2;

	double norm01 = d01.norm();
	double norm12 = d12.norm();
	double norm02 = d02.norm();

	double norm01C12 = d01.cross(d12).norm();

	double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	double beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	center = alpha * p0 + beta * p1 + gamma * p2;
	radius = (norm01 * norm12 * norm02) / (2 * norm01C12);
}

/*
Returns the unnormalized normal of the face formed by 3 points
*/
Vec3 PivotingBall2::Normal(Vertex vertex0, Vertex vertex1, Vertex vertex2)
{
	Vec3 p0 = pointPositions[vertex0];
	Vec3 p1 = pointPositions[vertex1];
	Vec3 p2 = pointPositions[vertex2];
	Vec3 v10 = p1 - p0;
	Vec3 v20 = p2 - p0;
	return v10.cross(v20);
}

bool PivotingBall2::WellOriented(Vertex vertex0, Vertex vertex1, Vertex vertex2)
{
	Vec3 normal = Normal(vertex0, vertex1, vertex2).normalized();

	int count = 0;
	count = pointNormals[vertex0].dot(normal) < 0 ? count + 1 : count;
	count = pointNormals[vertex1].dot(normal) < 0 ? count + 1 : count;
	count = pointNormals[vertex2].dot(normal) < 0 ? count + 1 : count;
	return count <= 1;
}

/*
Check if the normal of the face formed by three vertices points toward the same direction as the normals of the vertices 
Reorder the three vertices if they are not in the correct winding
*/
void PivotingBall2::Reorder(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vertex* order)
{
	if (WellOriented(vertex0,vertex1,vertex2))
	{
		order[0] = vertex0;
		order[1] = vertex1;
		order[2] = vertex2;
	}
	else
	{
		order[0] = vertex1;
		order[1] = vertex0;
		order[2] = vertex2;
	}
}

/*
Given three vertices, calculate where the center of the ball should be 
*/
bool PivotingBall2::GetBallCenter(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3& center)
{
	Vec3 normal = Normal(vertex0, vertex1, vertex2).normalized();

	Vec3 circleCenter; 
	double circleRadius; 
	GetCircumscribedCircle(vertex0, vertex1, vertex2, circleCenter, circleRadius);
	double squaredDistance = ballRadius * ballRadius - circleRadius * circleRadius;
	if (0.0 < squaredDistance)
	{
		double distance = sqrt(fabs(squaredDistance));
		center = circleCenter + distance * normal;
		return true; 
	}
	else
	{
		return false; 
	}
}

/*
Add an edge
*/
void PivotingBall2::AddTriedEdge(CMap2::Edge edge)
{
	triedEdgeList.push_back(edge);

	auto iterator = triedEdgeList.end();
	--iterator;

	CMap2::Vertex startVertex2 = CMap2::Vertex(edge.dart);
	Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointTriedEdges[startVertex0].push_back(iterator);
}

/*
Remove an edge
*/
void PivotingBall2::RemoveTriedEdge(std::list<CMap2::Edge>::iterator edge)
{
	CMap2::Vertex startVertex2 = CMap2::Vertex(edge->dart);
	Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointTriedEdges[startVertex0].remove(edge);

	triedEdgeList.erase(edge);
}

/*
Search an edge
*/
std::list<CMap2::Edge>::iterator PivotingBall2::FindTriedEdge(Vertex startVertex, Vertex endVertex)
{
	std::list<std::list<CMap2::Edge>::iterator>& edges = pointTriedEdges[startVertex];
	auto iterator = edges.begin();
	while (iterator != edges.end())
	{
		Vertex otherEndVertex = surfaceVertexes[CMap2::Vertex(surface->phi1((*iterator)->dart))];
		if (otherEndVertex == endVertex)
		{
			return *iterator;
		}
		++iterator;
	}

	return triedEdgeList.end();
}

/*
Search the opposite of an edge
*/
std::list<CMap2::Edge>::iterator PivotingBall2::FindOppositeTriedEdge(CMap2::Edge edge)
{
	Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge.dart)];
	Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge.dart))];
	return FindTriedEdge(endVertex,startVertex);
}

/*
Add an edge to the front
*/
void PivotingBall2::AddUntriedEdge(CMap2::Edge edge)
{
	untriedEdgeList.push_back(edge);
	
	auto iterator = untriedEdgeList.end();
	--iterator; 

	CMap2::Vertex startVertex2 = CMap2::Vertex(edge.dart);
	Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointUntriedEdges[startVertex0].push_back(iterator);
}

/*
Remove an edge from the front
*/
void PivotingBall2::RemoveUntriedEdge(std::list<CMap2::Edge>::iterator edge)
{
	CMap2::Vertex startVertex2 = CMap2::Vertex(edge->dart);
	Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointUntriedEdges[startVertex0].remove(edge);

	untriedEdgeList.erase(edge); 
}

/*
Search an edge on the front
*/
std::list<CMap2::Edge>::iterator PivotingBall2::FindUntriedEdge(Vertex startVertex, Vertex endVertex)
{
	std::list<std::list<CMap2::Edge>::iterator>& frontEdges = pointUntriedEdges[startVertex];
	auto iterator = frontEdges.begin();
	while (iterator != frontEdges.end())
	{
		Vertex otherEndVertex = surfaceVertexes[CMap2::Vertex(surface->phi1((*iterator)->dart))];
		if (otherEndVertex == endVertex)
		{
			return *iterator;
		}
		++iterator;
	}

	return untriedEdgeList.end();
}

/*
Search the opposite of an edge on the front
*/
std::list<CMap2::Edge>::iterator PivotingBall2::FindOppositeUntriedEdge(CMap2::Edge edge)
{
	Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge.dart)];
	Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge.dart))];
	return FindUntriedEdge(endVertex,startVertex);
}

/*
Add an edge to the front
*/
void PivotingBall2::AddSewedEdge(CMap2::Edge edge)
{
	sewedEdgeList.push_back(edge);

	auto iterator = sewedEdgeList.end();
	--iterator;

	CMap2::Vertex startVertex2 = CMap2::Vertex(edge.dart);
	Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointSewedEdges[startVertex0].push_back(iterator);
}

/*
Remove an edge from the front
*/
void PivotingBall2::RemoveSewedEdge(std::list<CMap2::Edge>::iterator edge)
{
	CMap2::Vertex startVertex2 = CMap2::Vertex(edge->dart);
	Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointSewedEdges[startVertex0].remove(edge);

	sewedEdgeList.erase(edge);
}

/*
Search an edge on the front
*/
std::list<CMap2::Edge>::iterator PivotingBall2::FindSewedEdge(Vertex startVertex, Vertex endVertex)
{
	std::list<std::list<CMap2::Edge>::iterator>& edges = pointSewedEdges[startVertex];
	auto iterator = edges.begin();
	while (iterator != edges.end())
	{
		Vertex otherEndVertex = surfaceVertexes[CMap2::Vertex(surface->phi1((*iterator)->dart))];
		if (otherEndVertex == endVertex)
		{
			return *iterator;
		}
		++iterator;
	}

	return sewedEdgeList.end();
}

/*
Adding a new edge, we first search for the opposite edge on the front:
- If the opposite edge is found, then they are glued together and the opposite is removed from the front
- If no edge is no found, add the new edge to the front
*/
void PivotingBall2::JoinOrGlueEdge(CMap2::Edge edge)
{
	std::list<CMap2::Edge>::iterator triedEdge = FindOppositeTriedEdge(edge);
	if (triedEdge != triedEdgeList.end())
	{
		surface->sew_faces(*triedEdge, edge);
		AddSewedEdge(*triedEdge);
		RemoveTriedEdge(triedEdge);
	}
	else
	{
		std::list<CMap2::Edge>::iterator untriedEdge = FindOppositeUntriedEdge(edge);
		if (untriedEdge != untriedEdgeList.end())
		{
			surface->sew_faces(*untriedEdge, edge);
			AddSewedEdge(*untriedEdge);
			RemoveUntriedEdge(untriedEdge);
		}
		else
		{
			AddUntriedEdge(edge);
		}
	}
}

bool PivotingBall2::ValidTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2)
{
	return FindSewedEdge(vertex0, vertex1) == sewedEdgeList.end()
		&& FindSewedEdge(vertex1, vertex2) == sewedEdgeList.end()
		&& FindSewedEdge(vertex2, vertex0) == sewedEdgeList.end()
		&& FindSewedEdge(vertex1, vertex0) == sewedEdgeList.end()
		&& FindSewedEdge(vertex2, vertex1) == sewedEdgeList.end()
		&& FindSewedEdge(vertex0, vertex2) == sewedEdgeList.end()
		&& FindTriedEdge(vertex0, vertex1) == triedEdgeList.end()
		&& FindTriedEdge(vertex1, vertex2) == triedEdgeList.end()
		&& FindTriedEdge(vertex2, vertex0) == triedEdgeList.end()
		&& FindUntriedEdge(vertex0, vertex1) == untriedEdgeList.end()
		&& FindUntriedEdge(vertex1, vertex2) == untriedEdgeList.end()
		&& FindUntriedEdge(vertex2, vertex0) == untriedEdgeList.end();
}

CMap2::Face PivotingBall2::AddTriangle(Vertex vertex0, Vertex vertex1, Vertex vertex2)
{
	CMap2::Face face = surface->add_face(3);

	CMap2::Edge edge0 = CMap2::Edge(face.dart);
	CMap2::Edge edge1 = CMap2::Edge(surface->phi1(edge0.dart));
	CMap2::Edge edge2 = CMap2::Edge(surface->phi1(edge1.dart));

	surfacePositions->operator[](edge0.dart) = pointPositions[vertex0];
	surfacePositions->operator[](edge1.dart) = pointPositions[vertex1];
	surfacePositions->operator[](edge2.dart) = pointPositions[vertex2];

	surfaceVertexes[edge0.dart] = vertex0;
	surfaceVertexes[edge1.dart] = vertex1;
	surfaceVertexes[edge2.dart] = vertex2;

	JoinOrGlueEdge(edge0);
	JoinOrGlueEdge(edge1);
	JoinOrGlueEdge(edge2);

	return face; 
}

bool PivotingBall2::IsEmpty(Vertex vertex0, Vertex vertex1, Vertex vertex2, Vec3 ballCenter)
{
	bool isEmpty = true;

	double squaredRadius = this->ballRadius * this->ballRadius; 
	double queryPoint[3] = { ballCenter[0], ballCenter[1], ballCenter[2] };
	nanoflann::SearchParams params;
	uint32_t resultCount = kdtree->index->radiusSearch(&queryPoint[0], squaredRadius, searchResult, params);

	//cgogn_log_info("radiusSearch: ") << ballCenter[0] << "," << ballCenter[1] << "," << ballCenter[2] << " : " << radius << " : " << resultCount;

	for (uint32_t i = 0; i < resultCount; i++)
	{
		Vertex vertex = Vertex(searchResult[i].first);
		
		//double distance = (pointPositions[vertex] - ballCenter).norm(); 
		//cgogn_log_info("distance: ") << distance;

		if (vertex != vertex0 && vertex != vertex1 && vertex != vertex2)
		{
			isEmpty = false;
		}
	}

	return isEmpty;
}

void PivotingBall2::GetNeighbors(Vec3 position, float radius)
{
	double squaredRadius = radius * radius;
	double queryPoint[3] = { position[0], position[1], position[2] };
	nanoflann::SearchParams params;
	uint32_t resultCount = kdtree->index->radiusSearch(&queryPoint[0], squaredRadius, searchResult, params);

	neighbors.resize(resultCount);
	for (uint32_t i = 0; i < resultCount; i++)
	{
		Vertex vertex = Vertex(searchResult[i].first);
		neighbors[i] = vertex; 
	}
}

void PivotingBall2::Initialize
(
	CMap0& points,
	CMap0::VertexAttribute<Vec3>& pointPositions,
	CMap0::VertexAttribute<Vec3>& pointNormals,
	CMap2& surface,
	CMap2::VertexAttribute<Vec3>& surfacePositions
)
{
	pointCount = 0;
	points.foreach_cell([&](CMap0::Vertex vertex)
	{
		pointCount++;
	});
	this->pointPositions.resize(pointCount);
	this->pointNormals.resize(pointCount);
	this->pointUntriedEdges.resize(pointCount);
	this->pointTriedEdges.resize(pointCount);
	this->pointSewedEdges.resize(pointCount);
	Vertex current = 0;
	points.foreach_cell([&](CMap0::Vertex vertex)
	{
		this->pointPositions[current] = pointPositions[vertex];
		this->pointNormals[current] = pointNormals[vertex];
		current++;
	});

	this->surface = &surface;
	this->surfacePositions = &surfacePositions;
	this->ballRadius = ballRadius;

	surfaceVertexes = surface.add_attribute<Vertex, CMap2::Vertex>("vertexes");

	cgogn_log_info("pointCount: ") << pointCount;

	pointsMatrix.resize(pointCount, 3);
	for (int i = 0; i < pointCount; i++)
	{
		Vec3 position = this->pointPositions[i];
		pointsMatrix(i, 0) = position[0];
		pointsMatrix(i, 1) = position[1];
		pointsMatrix(i, 2) = position[2];
	}
	kdtree = new KDTree(3, std::cref(pointsMatrix), pointCount);
	kdtree->index->buildIndex(); 
	searchResult.resize(pointCount);
	neighbors.resize(pointCount);
}

void PivotingBall2::SetRadius(double radius)
{
	this->ballRadius = radius;
}

bool PivotingBall2::FindSeed()
{
	double neighborhoodSize = 1.3;

	bool found = false;

	for (Vertex vertex0 = 0; vertex0 < pointCount; vertex0++)
	{
		Vec3 position0 = pointPositions[vertex0];

		GetNeighbors(position0, ballRadius * neighborhoodSize);

		if (3 <= neighbors.size())
		{
			for (size_t j = 0; j < neighbors.size() && !found; j++)
			{
				Vertex vertex1 = neighbors[j];
				if (vertex1 != vertex0)
				{
					for (size_t k = 0; k < neighbors.size() && !found; k++)
					{
						if (j != k)
						{
							Vertex vertex2 = neighbors[k];
							if (vertex2 != vertex0)
							{
								Vec3 center;
								Vertex order[3];
								Reorder(vertex0, vertex1, vertex2, order);
								if (GetBallCenter(order[0], order[1], order[2], center))
								{
									if (IsEmpty(order[0], order[1], order[2], center))
									{
										if (ValidTriangle(order[0], order[1], order[2]))
										{
											AddTriangle(order[0], order[1], order[2]);
											found = true;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	return found; 
}

bool PivotingBall2::FrontIsDone()
{
	return untriedEdgeList.empty();
}

void PivotingBall2::OneFrontIteration()
{
	auto edge = untriedEdgeList.begin(); 

	Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge->dart)];
	Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge->dart))];
	Vertex oppositeVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(surface->phi1(edge->dart)))];

	bool oneFound = false;
	Vertex bestOrder[3];
	double bestAngle = M_PI;

	Vec3 edgeStartPosition = pointPositions[startVertex];
	Vec3 edgeEndPosition = pointPositions[endVertex];
	Vec3 edgeOppositePosition = pointPositions[oppositeVertex];

	Vec3 edgeMiddle = (edgeStartPosition + edgeEndPosition) / 2;
	Vec3 ballCenter;
	if (GetBallCenter(startVertex, endVertex, oppositeVertex, ballCenter))
	{
		Vec3 diff1 = 100 * (edgeStartPosition - edgeMiddle);
		Vec3 diff2 = 100 * (ballCenter - edgeMiddle);

		Vec3 y = diff1.cross(diff2).normalized();
		Vec3 normal = diff2.cross(y).normalized();
		Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>(normal, edgeMiddle);

		Vec3 zeroAngle = ((Vec3)(edgeOppositePosition - edgeMiddle)).normalized();
		zeroAngle = plane.projection(zeroAngle).normalized();

		GetNeighbors(edgeMiddle, ballRadius * 2);
		for (size_t t = 0; t < neighbors.size(); t++)
		{
			Vertex newVertex = neighbors[t];
			if (newVertex != startVertex && newVertex != endVertex && newVertex != oppositeVertex)
			{
				Vec3 position = pointPositions[newVertex];
				if (plane.absDistance(position) <= ballRadius)
				{
					Vec3 center;
					Vertex order[3];
					if (Normal(startVertex, endVertex, newVertex).norm() > COMPARISON_EPSILON)
					{
						Reorder(startVertex, endVertex, newVertex, order);
						if (GetBallCenter(order[0], order[1], order[2], center))
						{
							if (IsEmpty(order[0], order[1], order[2], center))
							{
								Vec3 projectedCenter = plane.projection(center);
								double cosAngle = zeroAngle.dot(projectedCenter.normalized());
								if (fabs(cosAngle) > 1)
									cosAngle = sign<double>(cosAngle);
								double angle = acos(cosAngle);

								if (!oneFound || angle < bestAngle)
								{
									if (ValidTriangle(order[0], order[1], order[2]))
									{
										bestOrder[0] = order[0];
										bestOrder[1] = order[1];
										bestOrder[2] = order[2];
										bestAngle = angle;
										oneFound = true;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	if (oneFound)
	{
		AddTriangle(bestOrder[0], bestOrder[1], bestOrder[2]);
	}
	else
	{
		AddTriedEdge(*edge);
		RemoveUntriedEdge(edge);
	}
}

void PivotingBall2::Complete()
{
	if (FindSeed())
	{
		while (!FrontIsDone())
		{
			OneFrontIteration(); 
		}
	}
	else
	{
		cgogn_log_error("PIVOTING BALL: No seed found for this ball radius in this point cloud");
	}

	if (!surface->is_well_embedded<CMap2::Vertex>())
	{
		cgogn_log_error("PIVOTING BALL: is_well_embedded returned false");
	}

	if (!surface->check_map_integrity())
	{
		cgogn_log_error("PIVOTING BALL: check_map_integrity returned false");
	}
}

void PivotingBall2::Debug(std::unique_ptr<cgogn::rendering::DisplayListDrawer>& drawer)
{
	drawer->point_size(2.0);
	drawer->begin(GL_LINES);
	drawer->color3f(1.0, 1.0, 0.0);
	auto untriedEdge = untriedEdgeList.begin();
	while (untriedEdge != untriedEdgeList.end())
	{
		CMap2::Edge edge = *untriedEdge;
		++untriedEdge;

		Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge.dart)];
		Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge.dart))];

		Vec3 startPosition = pointPositions[startVertex];
		Vec3 endPosition = pointPositions[endVertex];

		if (untriedEdge == untriedEdgeList.end())
		{
			drawer->color3f(1.0, 0.0, 1.0);
		}
		drawer->vertex3f(startPosition[0], startPosition[1], startPosition[2]);
		drawer->vertex3f(endPosition[0], endPosition[1], endPosition[2]);
	}
	drawer->end(); 
}
