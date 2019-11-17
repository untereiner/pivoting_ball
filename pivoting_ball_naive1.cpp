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

bool PivotingBallNaive1::GetBallCenter(CMap0::Vertex index0, CMap0::Vertex index1, CMap0::Vertex index2, Vec3& center, CMap0::Vertex* sequence)
{
	bool status = false;

	Vec3 p0 = pointPositions->operator[](index0);
	Vec3 p1 = pointPositions->operator[](index1);
	Vec3 p2 = pointPositions->operator[](index2);
	sequence[0] = index0;
	sequence[1] = index1; 
	sequence[2] = index2; 

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
			sequence[0] = index1;
			sequence[1] = index0;

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

void PivotingBallNaive1::AddEdgeToFront(CMap0::Vertex startVertex, CMap0::Vertex endVertex, CMap0::Vertex oppositeVertex)
{
	FrontEdge edge; 
	edge.startVertex = startVertex; 
	edge.endVertex = endVertex;
	edge.oppositeVertex = oppositeVertex;
	front.push_back(edge);
}

void PivotingBallNaive1::RemoveEdgeFromFront(std::list<FrontEdge>::iterator edge)
{
	front.erase(edge); 
}

void PivotingBallNaive1::PushTriangle(CMap0::Vertex vertex0, CMap0::Vertex vertex1, CMap0::Vertex vertex2)
{
	Vec3 position0 = pointPositions->operator[](vertex0);
	Vec3 position1 = pointPositions->operator[](vertex1);
	Vec3 position2 = pointPositions->operator[](vertex2);

	auto face = surface->add_face(3);
	surfacePositions->operator[](face.dart) = position0;
	surfacePositions->operator[](surface->phi1(face.dart)) = position1;
	surfacePositions->operator[](surface->phi1(surface->phi1(face.dart))) = position2;
}

std::list<FrontEdge>::iterator PivotingBallNaive1::FindEdgeOnFront(CMap0::Vertex vertex0, CMap0::Vertex vertex1)
{
	auto iterator = front.begin();
	while (iterator != front.end())
	{
		if ((iterator->startVertex.dart == vertex0.dart && iterator->endVertex.dart == vertex1.dart)
			|| (iterator->startVertex.dart == vertex1.dart && iterator->endVertex.dart == vertex0.dart))
		{
			return iterator;
		}
		++iterator;
	}
	return iterator; 
}

void PivotingBallNaive1::JoinOrGlueEdge(CMap0::Vertex vertex0, CMap0::Vertex vertex1, CMap0::Vertex oppositeVertexId)
{
	auto edge = FindEdgeOnFront(vertex0, vertex1);
	if (edge == front.end())
	{
		AddEdgeToFront(vertex0, vertex1, oppositeVertexId);
	}
	else
	{
		RemoveEdgeFromFront(edge); 
	}
}

bool PivotingBallNaive1::IsEmpty(CMap0::Vertex vertex0, CMap0::Vertex vertex1, CMap0::Vertex vertex2, Vec3 ballCenter)
{
	points->foreach_cell([&](CMap0::Vertex vertex)
	{
		if (vertex.dart != vertex0.dart && vertex.dart != vertex1.dart && vertex.dart != vertex2.dart)
		{
			Vec3 position = pointPositions->operator[](vertex);
			Vec3 dist = (position - ballCenter);
			if (dist.norm() < ballRadius)
				return false;
		}
	}); 

	return true;
}

std::vector<CMap0::Vertex> PivotingBallNaive1::GetNeighbors(Vec3 position, float radius)
{
	std::vector<CMap0::Vertex> indices;
	points->foreach_cell([&](CMap0::Vertex index1)
	{
		Vec3 position1 = pointPositions->operator[](index1);
		if ((position1 - position).norm() < radius)
		{
			indices.push_back(index1);
		}
	}); 
	return indices; 
}

bool PivotingBallNaive1::FindSeed()
{
	double neighborhoodSize = 1.3;

	bool found = false;

	points->foreach_cell([&](CMap0::Vertex index0)
	{
		Vec3 position0 = pointPositions->operator[](index0);

		std::vector<CMap0::Vertex> indices = GetNeighbors(position0, ballRadius * neighborhoodSize);

		if (3 <= indices.size())
		{
			for (size_t j = 0; j < indices.size() && !found; j++)
			{
				CMap0::Vertex index1 = indices[j];
				if (index1.dart != index0.dart)
				{
					for (size_t k = 0; k < indices.size() && !found; k++)
					{
						if (j != k)
						{
							CMap0::Vertex index2 = indices[k];
							if (index2.dart != index0.dart)
							{
								Vec3 center;
								CMap0::Vertex sequence[3];
								if (GetBallCenter(index0, index1, index2, center, sequence))
								{
									if (IsEmpty(index0, index1, index2, center))
									{
										PushTriangle(index0, index1, index2);
										AddEdgeToFront(index0, index1, index2);
										AddEdgeToFront(index1, index2, index0);
										AddEdgeToFront(index2, index0, index1);
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

void PivotingBallNaive1::FinishFront()
{
	while (!front.empty())
	{
		auto edge = front.begin();

		CMap0::Vertex startVertex = edge->startVertex;
		CMap0::Vertex endVertex = edge->endVertex;
		CMap0::Vertex oppositeVertex = edge->oppositeVertex;

		Vec3 edgeStartPosition = pointPositions->operator[](startVertex);
		Vec3 edgeEndPosition = pointPositions->operator[](endVertex);
		Vec3 edgeOppositePosition = pointPositions->operator[](oppositeVertex);

		Vec3 edgeMiddle = (edgeStartPosition + edgeEndPosition) / 2;
		Vec3 ballCenter;
		CMap0::Vertex sequence[3];
		if (!GetBallCenter(startVertex, endVertex, oppositeVertex, ballCenter, sequence))
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
		CMap0::Vertex bestSequence[3];
		double bestAngle = M_PI;

		std::vector<CMap0::Vertex> indices = GetNeighbors(edgeMiddle, ballRadius * 2);
		for (size_t t = 0; t < indices.size(); t++)
		{
			CMap0::Vertex newVertex = indices[t];
			if (newVertex.dart != startVertex.dart && newVertex.dart != endVertex.dart && newVertex.dart != oppositeVertex.dart)
			{
				Vec3 position = pointPositions->operator[](newVertex);
				if (plane.absDistance(position) <= ballRadius)
				{
					Vec3 center;
					CMap0::Vertex sequence[3];
					if (GetBallCenter(startVertex, endVertex, newVertex, center, sequence))
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
									bestSequence[0] = sequence[0];
									bestSequence[1] = sequence[1];
									bestSequence[2] = sequence[2];
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
			PushTriangle(bestSequence[0], bestSequence[2], bestSequence[1]);

			JoinOrGlueEdge(bestSequence[0], bestSequence[2], bestSequence[1]);
			JoinOrGlueEdge(bestSequence[2], bestSequence[1], bestSequence[0]);
		}

		RemoveEdgeFromFront(edge);
	}
}

void PivotingBallNaive1::getSurface
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
	
	if (!FindSeed())
	{
		exit(1); 
	}
	FinishFront(); 
}