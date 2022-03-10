
#include <vector>
#include <stddef.h>

struct LvPoint
{
	constexpr LvPoint(const double &_x = 0, const double &_y = 0): x(_x), y(_y) {}
	double x = 0;
	double y = 0;
};


#include "clipper.hpp"
#include <cmath>
#define LVCLIPPERSCALE 10000.0f
struct LvBBox
{
	LvPoint min;
	LvPoint max;
};
double OccupiedPercent(std::vector<LvBBox> bboxs, std::vector<LvPoint> polygon)
{
	ClipperLib::Clipper c;

	ClipperLib::Path clip;
	size_t p_size = polygon.size();
	for (size_t nth_p = 0; nth_p < p_size; ++nth_p)
	{
		clip.push_back({
			(ClipperLib::cInt)(polygon[nth_p].x * LVCLIPPERSCALE),
			(ClipperLib::cInt)(polygon[nth_p].y * LVCLIPPERSCALE)
		});
	}
	if (ClipperLib::Orientation(clip)) ClipperLib::ReversePath(clip);
	c.AddPaths({clip}, ClipperLib::ptClip, true);


	size_t b_size = bboxs.size();
	for (size_t nth_b = 0; nth_b < b_size; ++nth_b)
	{
		ClipperLib::cInt xmin = (ClipperLib::cInt)(bboxs[nth_b].min.x * LVCLIPPERSCALE);
		ClipperLib::cInt ymin = (ClipperLib::cInt)(bboxs[nth_b].min.y * LVCLIPPERSCALE);
		ClipperLib::cInt xmax = (ClipperLib::cInt)(bboxs[nth_b].max.x * LVCLIPPERSCALE);
		ClipperLib::cInt ymax = (ClipperLib::cInt)(bboxs[nth_b].max.y * LVCLIPPERSCALE);

		ClipperLib::Path subj = {{xmin, ymin}, {xmin, ymax}, {xmax, ymax}, {xmax, ymin}};
		if (ClipperLib::Orientation(subj)) {ClipperLib::ReversePath(subj);}
		c.AddPaths({subj}, ClipperLib::ptSubject, true);
	}


	ClipperLib::Paths solution;
	if (c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero))
	{
		size_t s_size = solution.size();
		double subj_area = 0;
		for (size_t nth_s = 0; nth_s < s_size; ++nth_s)
		{
			double nth_area = ClipperLib::Area(solution[nth_s]);
			subj_area += nth_area;
		}
		double clip_area = ClipperLib::Area(clip);
		if (clip_area == 0) return 0;
		return fabs(subj_area / clip_area);
	}
	return 0;
}



// how to compile?
// g++ main.cpp clipper.cpp -o occupancy_demo
// ./occupancy_demo
#include <stdio.h>
int main(int argc, char const *argv[])
{
	printf("hello welcome to occupancy algo!!!\n");
	std::vector<LvBBox> bboxs;
	bboxs.push_back({{0, 0}, {0.5, 0.5}}); // box A
	bboxs.push_back({{0.5, 0.5}, {1, 1}}); // box B

	std::vector<LvPoint> polygons = {{0, 0}, {0, 1}, {1, 1}, {1, 0}}; // polygon

	printf("occupancy %f\n", OccupiedPercent(bboxs, polygons));
	return 0;
}

