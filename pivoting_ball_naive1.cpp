#include "pivoting_ball_naive1.h"
/*
Taken from https://github.com/cdcseacave/VCG/blob/master/vcg/complex/algorithms/create/ball_pivoting.h
*/
bool Inferior(Vec3 p0, Vec3 p1)
{
	return (p0[2] != p1[2]) ? (p0[2]<p1[2]) :
		   (p0[1] != p1[1]) ? (p0[1]<p1[1]) :
		   (p0[0]<p1[0]);
}

/*
Taken from https://github.com/cdcseacave/VCG/blob/master/vcg/complex/algorithms/create/ball_pivoting.h
*/
bool PivotingBallNaive1::FindSphere(Vec3 p0, Vec3 p1, Vec3 p2, Vec3& center)
{
	Vec3 p[3];

	if (Inferior(p0,p1) && Inferior(p0,p2))
	{
		p[0] = p0;
		p[1] = p1;
		p[2] = p2;
	}
	else if (Inferior(p1,p0) && Inferior(p1,p2))
	{
		p[0] = p1;
		p[1] = p2;
		p[2] = p0;
	}
	else {
		p[0] = p2;
		p[1] = p0;
		p[2] = p1;
	}

	Vec3 q1 = p[1] - p[0];
	Vec3 q2 = p[2] - p[0];

	Vec3 up = q1.cross(q2);
	float uplen = up.norm();

	if (uplen < 0.001*q1.norm()*q2.norm())
	{
		return false;
	}
	up /= uplen;

	float a11 = q1.dot(q1);
	float a12 = q1.dot(q2);
	float a22 = q2.dot(q2);

	float m = 4 * (a11*a22 - a12 * a12);
	float l1 = 2 * (a11*a22 - a22 * a12) / m;
	float l2 = 2 * (a11*a22 - a12 * a11) / m;

	center = q1 * l1 + q2 * l2;
	float radius = center.norm();
	if (radius > ballRadius)
	{
		return false; 
	}

	float height = sqrt(ballRadius*ballRadius - radius * radius);
	center += p[0] + up * height;

	return true;
}

/*
Taken from https://github.com/cdcseacave/VCG/blob/master/vcg/complex/algorithms/create/ball_pivoting.h
*/
float PivotingBallNaive1::OrientedAngleRad(Vec3 p, Vec3 q, Vec3 &axis) 
{
	p.normalize();
	q.normalize();
	Vec3 vec = p.cross(q);
	float angle = acos(p.dot(q));
	if (vec.dot(axis) < 0) angle = -angle;
	if (angle < 0) angle += 2 * M_PI;
	return angle;
}

FrontEdge* PivotingBallNaive1::AddEdgeToFront(uint32_t startVertexId, uint32_t endVertexId, uint32_t oppositeVertexId, Vec3 ballCenter)
{
	auto edge = new FrontEdge(); 

	edge->startVertexId = startVertexId; 
	edge->endVertexId = endVertexId; 
	edge->oppositeVertexId = oppositeVertexId;
	edge->ballCenter = ballCenter; 

	edge->previous = frontLast; 
	if (edge->previous == nullptr)
		frontFirst = edge;
	else
		edge->previous->next = edge;

	edge->next = nullptr; 
	frontLast = edge; 

	return edge; 
}

void PivotingBallNaive1::RemoveEdgeFromFront(FrontEdge* edge)
{
	if (edge->previous == nullptr)
		frontFirst = edge->next;
	else
		edge->previous->next = edge->next;

	if (edge->next == nullptr)
		frontLast = edge->previous;
	else
		edge->next->previous = edge->previous;

	delete(edge);
}

void PivotingBallNaive1::PushTriangle(uint32_t point0, uint32_t point1, uint32_t point2)
{
	Vec3 position0 = pointsPosition[point0];
	Vec3 position1 = pointsPosition[point1];
	Vec3 position2 = pointsPosition[point2];
	float red = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	float green = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	float blue = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	triangles.push_back(position0);
	triangles.push_back(position1);
	triangles.push_back(position2);
	triangles.push_back(Vec3(red, green, blue));
	
	if (tagPoints)
	{
		pointsUsed[point0] = true;
		pointsUsed[point1] = true;
		pointsUsed[point2] = true;
	}
}

FrontEdge* PivotingBallNaive1::FindEdgeOnFront(uint32_t point0, uint32_t point1)
{
	auto edge = frontFirst; 
	while (edge != nullptr)
	{
		if ((edge->startVertexId == point0 && edge->endVertexId == point1)
			|| (edge->startVertexId == point1 && edge->endVertexId == point0))
		{
			return edge; 
		}
		edge = edge->next; 
	}
	return nullptr; 
}

void PivotingBallNaive1::JoinOrGlueEdge(uint32_t point0, uint32_t point1, uint32_t oppositeVertexId, Vec3 ballCenter)
{
	FrontEdge* edge = FindEdgeOnFront(point0, point1); 
	if (edge == nullptr)
	{
		AddEdgeToFront(point0, point1, oppositeVertexId, ballCenter);
	}
	else
	{
		RemoveEdgeFromFront(edge); 
	}
}

void PivotingBallNaive1::getSurface
(
	CMap0& pointSet,
	CMap0::VertexAttribute<Vec3>& pointSetPositions,
	CMap2& surface,
	CMap2::VertexAttribute<Vec3>& surfacePositions,
	CMap2::VertexAttribute<Vec3>& surfaceColors,
	float ballRadius
)
{
	pointSet.foreach_cell([&](CMap0::Vertex vertex)
	{
		pointsPosition.push_back(pointSetPositions[vertex]);
		pointsUsed.push_back(false); 
	});

	this->ballRadius = ballRadius; 

	Vec3 ballCenter;
	if (!FindSphere(pointSetPositions[0], pointSetPositions[1], pointSetPositions[2], ballCenter))
	{
		exit(1); 
	}
	PushTriangle(0, 1, 2);
	AddEdgeToFront(0, 1, 2, ballCenter);
	AddEdgeToFront(1, 2, 0, ballCenter);
	AddEdgeToFront(2, 0, 1, ballCenter);

	while (frontFirst != nullptr && triangles.size() < 1000)
	{
		FrontEdge* edge = frontFirst;
		FrontEdge* previous = edge->previous; 
		FrontEdge* next = edge->next;

		Vec3 edgeStartPosition = pointsPosition[edge->startVertexId];
		Vec3 edgeEndPosition = pointsPosition[edge->endVertexId];
	
		Vec3 ballCenter = edge->ballCenter; 
		Vec3 edgeMiddlePosition = (edgeStartPosition + edgeEndPosition) / 2;
		Vec3 startPivot = ballCenter - edgeMiddlePosition;

		Vec3 axis = edgeEndPosition - edgeStartPosition;
		float axisSquareNorm = axis.squaredNorm(); 
		float limit = sqrt(ballRadius*ballRadius - axisSquareNorm / 4.0);
		axis = axis.normalized();

		uint32_t candidate = ~uint32_t(0); 
		Vec3 candidateCenter;
		float candidateAngle;

		for (uint32_t i = 0; i < pointsPosition.size(); i++)
		{
			if (i != edge->startVertexId
				&& i != edge->endVertexId
				&& i != edge->oppositeVertexId)
			{
				Vec3 position = pointsPosition[i];
				if ((position - edgeMiddlePosition).norm() <= limit + ballRadius)
				{
					Vec3 center;
					FindSphere(edgeStartPosition, edgeEndPosition, position, center);

					float angle = OrientedAngleRad(startPivot, center - edgeMiddlePosition, axis);

					if ((candidate == ~uint32_t(0)) || (angle < candidateAngle))
					{
						candidate = i;
						candidateCenter = center;
						candidateAngle = angle;
					}
				}
			}
		}

		if ((candidate != ~uint32_t(0)) && (candidateAngle < M_PI - 0.1))
		{
			if (!pointsUsed[candidate])
			{
				PushTriangle(edge->startVertexId, edge->endVertexId, candidate);
				JoinOrGlueEdge(edge->endVertexId, candidate, edge->startVertexId, candidateCenter);
				JoinOrGlueEdge(candidate, edge->startVertexId, edge->endVertexId, candidateCenter);
			}
		}

		RemoveEdgeFromFront(edge); 
	}


	for (uint32_t i = 0; i < triangles.size() / 4; i++)
	{
		auto color = triangles[i * 4 + 3];
		//drawer_->color3f(color[0], color[1], color[2]);
		auto position0 = triangles[i * 4];
		auto position1 = triangles[i * 4 + 1];
		auto position2 = triangles[i * 4 + 2];

		auto face = surface.add_face(3);
		surfacePositions[face.dart] = position0;
		surfacePositions[surface.phi1(face.dart)] = position1;
		surfacePositions[surface.phi1(surface.phi1(face.dart))] = position2;
	}

}