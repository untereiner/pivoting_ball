#include "pivoting_ball_naive0.h"

void PivotingBallNaive0::push_to_front(uint32_t edgeStart, uint32_t edgeEnd, uint32_t edgeDirection)
{
	front.push_back(edgeStart);
	front.push_back(edgeEnd);
	front.push_back(edgeDirection);
}

void PivotingBallNaive0::push_triangle(uint32_t point0, uint32_t point1, uint32_t point2, bool pushEdge0, bool pushEdge1, bool pushEdge2)
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

	if (!pointsUsed[point0] || !pointsUsed[point1])
		push_to_front(point0, point1, point2);
	if (!pointsUsed[point1] || !pointsUsed[point2])
		push_to_front(point1, point2, point0);
	if (!pointsUsed[point0] || !pointsUsed[point2])
		push_to_front(point2, point0, point1);

	pointsUsed[point0] = true;
	pointsUsed[point1] = true;
	pointsUsed[point2] = true;
}

double PivotingBallNaive0::edgePointDistance(Vec3 edgeStart, Vec3 edgeEnd, Vec3 point)
{
	Vec3 start_end = edgeEnd - edgeStart;
	Vec3 start_point = point - edgeStart;
	Vec3 end_point = point - edgeEnd;

	if (start_point.dot(start_end) <= 0.0)
		return start_point.norm();

	if (end_point.dot(start_end) >= 0.0)
		return end_point.norm();

	return start_end.cross(start_point).norm() / start_end.norm();
}

Vec3 PivotingBallNaive0::getEdgeNormal(Vec3 edgeStart, Vec3 edgeEnd, Vec3 otherPoint)
{
	return ((edgeStart - otherPoint).normalized() + (edgeEnd - otherPoint).normalized()).normalized();
}

bool PivotingBallNaive0::FindSeed()
{
	push_triangle(0, 1, 2, true, true, true);
	return true; 
}

void PivotingBallNaive0::OneFrontIteration()
{
	uint32_t edgeDirection = front.back();
	front.pop_back();
	uint32_t edgeEnd = front.back();
	front.pop_back();
	uint32_t edgeStart = front.back();
	front.pop_back();

	Vec3 edgeStartPosition = pointsPosition[edgeStart];
	Vec3 edgeEndPosition = pointsPosition[edgeEnd];
	Vec3 thirdPoint = pointsPosition[edgeDirection];
	Vec3 edgeNormal = getEdgeNormal(edgeStartPosition, edgeEndPosition, thirdPoint).normalized();

	uint32_t bestIndex = UINT32_MAX;
	double bestDistance = std::numeric_limits<double>::max();
	for (uint32_t i = 0; i < pointsPosition.size(); i++)
	{
		if (i != edgeDirection && i != edgeStart && i != edgeEnd)
		{
			Vec3 point = pointsPosition[i];
			double distance = edgePointDistance(edgeStartPosition, edgeEndPosition, point);
			if (distance < bestDistance)
			{
				bestIndex = i;
				bestDistance = distance;
			}
		}
	}

	if (bestIndex != UINT32_MAX)
	{
		push_triangle(edgeStart, bestIndex, edgeEnd, false, true, true);
	}
}

void PivotingBallNaive0::AllFrontIteration()
{
	while (front.size() != 0)
	{
		OneFrontIteration(); 
	}
}

void PivotingBallNaive0::Initialize
(
	CMap0& pointSet, 
	CMap0::VertexAttribute<Vec3>& pointSetPositions,
	CMap0::VertexAttribute<Vec3>& pointNormals,
	CMap2& surface,
	CMap2::VertexAttribute<Vec3>& surfacePositions,
	float ballRadius
)
{
	pointSet.foreach_cell([&](CMap0::Vertex vertex)
	{
		pointsPosition.push_back(pointSetPositions[vertex]);
		pointsUsed.push_back(false);
	});

	this->surface = &surface;
	this->surfacePositions = &surfacePositions;
}

void PivotingBallNaive0::Complete()
{
	FindSeed();
	AllFrontIteration();

	for (uint32_t i = 0; i < triangles.size() / 4; i++)
	{
		auto color = triangles[i * 4 + 3];
		//drawer_->color3f(color[0], color[1], color[2]);
		auto position0 = triangles[i * 4];
		auto position1 = triangles[i * 4 + 1];
		auto position2 = triangles[i * 4 + 2];

		auto face = surface->add_face(3);
		surfacePositions->operator[](face.dart) = position0;
		surfacePositions->operator[](surface->phi1(face.dart)) = position1;
		surfacePositions->operator[](surface->phi1(surface->phi1(face.dart))) = position2;
	}

}

void PivotingBallNaive0::Debug(std::unique_ptr<cgogn::rendering::DisplayListDrawer>& drawer)
{
}