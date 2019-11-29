#include "pivoting_ball_naive1.h"

#define COMPARISON_EPSILON	1e-10
#define IN_BALL_THRESHOLD	1e-7

template <typename T> int sign(T val) 
{
	return (T(0) < val) - (val < T(0));
}

void GetCircumscribedCircle(Vec3 p0, Vec3 p1, Vec3 p2, Vec3& center, double& radius)
{
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

bool IsOriented(Vec3 normal, Vec3 normal0, Vec3 normal1, Vec3 normal2)
{
	int count = 0;
	count = normal0.dot(normal) < 0 ? count + 1 : count;
	count = normal1.dot(normal) < 0 ? count + 1 : count;
	count = normal2.dot(normal) < 0 ? count + 1 : count;
	return count <= 1;
}

bool PivotingBallNaive1::GetBallCenter(CMap0::Vertex index0, CMap0::Vertex index1, CMap0::Vertex index2, Vec3& center, CMap0::Vertex* order)
{
	bool status = false;

	Vec3 p0 = pointPositions->operator[](index0);
	Vec3 p1 = pointPositions->operator[](index1);
	Vec3 p2 = pointPositions->operator[](index2);
	order[0] = index0;
	order[1] = index1;
	order[2] = index2;

	Vec3 v10 = p1 - p0;
	Vec3 v20 = p2 - p0;
	Vec3 normal = v10.cross(v20);

	if (normal.norm() > COMPARISON_EPSILON)
	{
		normal.normalize();
		if (!IsOriented(normal, pointNormals->operator[](index0), pointNormals->operator[](index1), pointNormals->operator[](index2)))
		{
			auto temp = p0; 
			p0 = p1;
			p1 = temp;
			order[0] = index1;
			order[1] = index0;

			v10 = p1 - p0;
			v20 = p2 - p0;
			normal = v10.cross(v20);
			normal.normalize();
		}
	
		Vec3 circleCenter; 
		double circleRadius; 
		GetCircumscribedCircle(p0, p1, p2, circleCenter, circleRadius);
		double squaredDistance = ballRadius * ballRadius - circleRadius * circleRadius;
		if (0.0 < squaredDistance)
		{
			double distance = sqrt(fabs(squaredDistance));
			center = circleCenter + distance * normal;
			status = true;
		}
	}
	return status;
}

void PivotingBallNaive1::AddFrontEdge(CMap2::Edge edge)
{
	front.push_back(edge);
	/*
	auto iterator = front.end(); 
	--iterator; */

	/*
	CMap2::Vertex startVertex2 = CMap2::Vertex(edge.dart);
	CMap0::Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointFrontEdges[startVertex0].push_back(iterator);*/
}

void PivotingBallNaive1::RemoveFrontEdge(std::list<CMap2::Edge>::iterator edge)
{
	/*
	CMap2::Vertex startVertex2 = CMap2::Vertex(edge->dart);
	CMap0::Vertex startVertex0 = surfaceVertexes[startVertex2];
	pointFrontEdges[startVertex0].remove(edge);*/

	front.erase(edge); 
}

std::list<CMap2::Edge>::iterator PivotingBallNaive1::FindFrontEdge(CMap2::Edge edge)
{
	CMap0::Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge.dart)];
	CMap0::Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge.dart))];
	/*
	std::list<std::list<CMap2::Edge>::iterator>& frontEdges = pointFrontEdges[endVertex];
	auto iterator = frontEdges.begin();
	while (iterator != frontEdges.end())
	{
		CMap0::Vertex otherEndVertex = surfaceVertexes[CMap2::Vertex(surface->phi1((*iterator)->dart))];
		if (otherEndVertex.dart == startVertex.dart)
		{
			return *iterator;
		}
		++iterator;
	}*/
	/*
	if (!vertex.dart.is_nil())
	{
		bool found = false;
		auto iterator = front.begin();

		int count = 0;
		surface->foreach_incident_edge(vertex, [&](CMap2::Edge edge)
		{
			CMap0::Vertex otherStartVertex = surfaceVertexes[CMap2::Vertex(edge.dart)];
			CMap0::Vertex otherEndVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge.dart))];

			if (otherEndVertex.dart == startVertex.dart || otherStartVertex.dart == startVertex.dart)
			{
				found = true;

				while (iterator != front.end() && iterator->dart != edge.dart && iterator->dart != surface->phi2(edge.dart))
				{
					++iterator;
				}
			}
			count++;

		});

		cgogn_log_error("count: ") << std::to_string(count);

		if (found)
			return iterator;
	}*/

	//return front.end();

	auto iterator = front.begin();
	while (iterator != front.end())
	{
		CMap0::Vertex otherStartVertex = surfaceVertexes[CMap2::Vertex(iterator->dart)];
		CMap0::Vertex otherEndVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(iterator->dart))];
		if (otherStartVertex.dart == endVertex.dart && otherEndVertex.dart == startVertex.dart)
		{
			return iterator;
		}
		++iterator;
	}
	return iterator;

}

void PivotingBallNaive1::JoinOrGlueEdge(CMap2::Edge edge)
{
	std::list<CMap2::Edge>::iterator frontEdge = FindFrontEdge(edge);
	if (frontEdge == front.end())
	{
		AddFrontEdge(edge);
	}
	else
	{
		surface->sew_faces(*frontEdge, edge);
		RemoveFrontEdge(frontEdge);
	}
}

CMap2::Face PivotingBallNaive1::AddTriangle(CMap0::Vertex vertex0, CMap0::Vertex vertex1, CMap0::Vertex vertex2)
{
	CMap2::Face face = surface->add_face(3);

	CMap2::Edge edge0 = CMap2::Edge(face.dart);
	CMap2::Edge edge1 = CMap2::Edge(surface->phi1(edge0.dart));
	CMap2::Edge edge2 = CMap2::Edge(surface->phi1(edge1.dart));

	surfacePositions->operator[](edge0.dart) = pointPositions->operator[](vertex0);
	surfacePositions->operator[](edge1.dart) = pointPositions->operator[](vertex1);
	surfacePositions->operator[](edge2.dart) = pointPositions->operator[](vertex2);

	surfaceVertexes[edge0.dart] = vertex0;
	surfaceVertexes[edge1.dart] = vertex1;
	surfaceVertexes[edge2.dart] = vertex2;

	JoinOrGlueEdge(edge0);
	JoinOrGlueEdge(edge1);
	JoinOrGlueEdge(edge2);

	return face; 
}

bool PivotingBallNaive1::IsEmpty(CMap0::Vertex vertex0, CMap0::Vertex vertex1, CMap0::Vertex vertex2, Vec3 ballCenter)
{
	bool isEmpty = true;

	points->foreach_cell([&](CMap0::Vertex vertex)
	{
		if (vertex.dart != vertex0.dart && vertex.dart != vertex1.dart && vertex.dart != vertex2.dart)
		{
			Vec3 position = pointPositions->operator[](vertex);
			Vec3 dist = (position - ballCenter);
			if (dist.norm() < ballRadius)
				isEmpty = false;
		}
	}); 

	return isEmpty;
}

std::vector<CMap0::Vertex> PivotingBallNaive1::GetNeighbors(Vec3 position, float radius)
{
	std::vector<CMap0::Vertex> neighbors;
	points->foreach_cell([&](CMap0::Vertex index1)
	{
		Vec3 position1 = pointPositions->operator[](index1);
		if ((position1 - position).norm() < radius)
		{
			neighbors.push_back(index1);
		}
	}); 
	return neighbors;
}

void PivotingBallNaive1::Initialize
(
	CMap0& points,
	CMap0::VertexAttribute<Vec3>& pointPositions,
	CMap0::VertexAttribute<Vec3>& pointNormals,
	CMap2& surface,
	CMap2::VertexAttribute<Vec3>& surfacePositions,
	float ballRadius
)
{
	this->points = &points;
	this->pointPositions = &pointPositions;
	this->pointNormals = &pointNormals;

	this->surface = &surface;
	this->surfacePositions = &surfacePositions;
	this->ballRadius = ballRadius;

	/*pointFrontEdges = points.add_attribute<std::vector<std::list<CMap2::Edge>::iterator>, CMap0::Vertex>("vertexes");

	points.foreach_cell([&](CMap0::Vertex vertex)
	{
		pointFrontEdges[vertex] = std::vector<std::list<CMap2::Edge>::iterator>();
	});*/

	surfaceVertexes = surface.add_attribute<CMap0::Vertex, CMap2::Vertex>("vertexes");
}

bool PivotingBallNaive1::FindSeed()
{
	double neighborhoodSize = 1.3;

	bool found = false;

	points->foreach_cell([&](CMap0::Vertex vertex0)
	{
		Vec3 position0 = pointPositions->operator[](vertex0);

		std::vector<CMap0::Vertex> neighbors = GetNeighbors(position0, ballRadius * neighborhoodSize);

		if (3 <= neighbors.size())
		{
			for (size_t j = 0; j < neighbors.size() && !found; j++)
			{
				CMap0::Vertex vertex1 = neighbors[j];
				if (vertex1.dart != vertex0.dart)
				{
					for (size_t k = 0; k < neighbors.size() && !found; k++)
					{
						if (j != k)
						{
							CMap0::Vertex vertex2 = neighbors[k];
							if (vertex2.dart != vertex0.dart)
							{
								Vec3 center;
								CMap0::Vertex order[3];
								if (GetBallCenter(vertex0, vertex1, vertex2, center, order))
								{
									if (IsEmpty(vertex0, vertex1, vertex2, center))
									{
										AddTriangle(order[0], order[2], order[1]);
										found = true;
									}
								}
							}
						}
					}
				}
			}
		}
	}); 

	return found; 
}

void PivotingBallNaive1::OneFrontIteration()
{
	auto edge = front.begin();

	CMap0::Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge->dart)];
	CMap0::Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge->dart))];
	CMap0::Vertex oppositeVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(surface->phi1(edge->dart)))];

	Vec3 edgeStartPosition = pointPositions->operator[](startVertex);
	Vec3 edgeEndPosition = pointPositions->operator[](endVertex);
	Vec3 edgeOppositePosition = pointPositions->operator[](oppositeVertex);

	Vec3 edgeMiddle = (edgeStartPosition + edgeEndPosition) / 2;
	Vec3 ballCenter;
	CMap0::Vertex order[3];
	if (!GetBallCenter(startVertex, endVertex, oppositeVertex, ballCenter, order))
	{
		exit(1);
	}

	Vec3 diff1 = 100 * (edgeStartPosition - edgeMiddle);
	Vec3 diff2 = 100 * (ballCenter - edgeMiddle);

	Vec3 y = diff1.cross(diff2).normalized();
	Vec3 normal = diff2.cross(y).normalized();
	Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>(normal, edgeMiddle);

	Vec3 zeroAngle = ((Vec3)(edgeOppositePosition - edgeMiddle)).normalized();
	zeroAngle = plane.projection(zeroAngle).normalized();

	bool oneFound = false;
	CMap0::Vertex bestOrder[3];
	double bestAngle = M_PI;

	std::vector<CMap0::Vertex> neighbors = GetNeighbors(edgeMiddle, ballRadius * 2);
	for (size_t t = 0; t < neighbors.size(); t++)
	{
		CMap0::Vertex newVertex = neighbors[t];
		if (newVertex.dart != startVertex.dart && newVertex.dart != endVertex.dart && newVertex.dart != oppositeVertex.dart)
		{
			Vec3 position = pointPositions->operator[](newVertex);
			if (plane.absDistance(position) <= ballRadius)
			{
				Vec3 center;
				CMap0::Vertex order[3];
				if (GetBallCenter(startVertex, endVertex, newVertex, center, order))
				{
					if (IsEmpty(startVertex, endVertex, newVertex, center))
					{
						Vec3 Vij = edgeEndPosition - edgeStartPosition;
						Vec3 Vik = position - edgeStartPosition;
						Vec3 faceNormal = Vik.cross(Vij).normalized();
						if (!IsOriented(faceNormal, pointNormals->operator[](startVertex), pointNormals->operator[](endVertex), pointNormals->operator[](newVertex)))
						{
							Vec3 projectedCenter = plane.projection(center);
							double cosAngle = zeroAngle.dot(projectedCenter.normalized());
							if (fabs(cosAngle) > 1)
								cosAngle = sign<double>(cosAngle);
							double angle = acos(cosAngle);

							if (!oneFound || angle < bestAngle)
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

	if (oneFound)
	{
		AddTriangle(bestOrder[0], bestOrder[2], bestOrder[1]);
	}
	else
	{
		RemoveFrontEdge(edge);
	}
}

void PivotingBallNaive1::AllFrontIteration()
{
	while (!front.empty())
	{
		OneFrontIteration(); 
	}
}

void PivotingBallNaive1::Complete()
{
	if (FindSeed())
	{
		AllFrontIteration();
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

void PivotingBallNaive1::Debug(std::unique_ptr<cgogn::rendering::DisplayListDrawer>& drawer)
{
	drawer->point_size(2.0);
	drawer->begin(GL_LINES);
	drawer->color3f(1.0, 1.0, 0.0);
	auto frontEdge = front.begin();
	while (frontEdge != front.end())
	{
		CMap2::Edge edge = *frontEdge; 

		CMap0::Vertex startVertex = surfaceVertexes[CMap2::Vertex(edge.dart)];
		CMap0::Vertex endVertex = surfaceVertexes[CMap2::Vertex(surface->phi1(edge.dart))];

		Vec3 startPosition = pointPositions->operator[](startVertex);
		Vec3 endPosition = pointPositions->operator[](endVertex);

		drawer->vertex3f(startPosition[0], startPosition[1], startPosition[2]);
		drawer->vertex3f(endPosition[0], endPosition[1], endPosition[2]);

		++frontEdge;
	}
	drawer->end(); 
}
