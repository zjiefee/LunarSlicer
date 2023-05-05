#ifndef MULTI_MATERIAL_SEGMENTATION_H
#define MULTI_MATERIAL_SEGMENTATION_H

#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/voronoi_diagram.hpp>
#include <boost/geometry.hpp>
#include <vector>

#include "BoostInterface.hpp"
#include "SkeletalTrapezoidationGraph.h"
#include "slicer.h"
#include "utils/PolygonsSegmentIndex.h"
#include "utils/polygonUtils.h"
#include <spdlog/spdlog.h>

namespace cura
{
class MultiMaterialSegmentation
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
    using graph_t = SkeletalTrapezoidationGraph;
    using edge_t = STHalfEdge;
    using node_t = STHalfEdgeNode;
    using Segment = PolygonsSegmentIndex;

    class Line
    {
    public:
        std::vector<Point> points;
        std::vector<int> colors;
    };

    std::unordered_map<vd_t::edge_type*, edge_t*> vd_edge_to_he_edge;
    std::unordered_map<vd_t::vertex_type*, node_t*> vd_node_to_he_node;
    std::unordered_map<Point*, Point> mms_random_small_offset_point;

    int scale = 1;
    int no_painting_color = 0;
    int painting_color = 1;

public:
    static void multiMaterialSegmentationByPainting(Slicer* slicer, Slicer* color_slicer);

private:
    static coord_t pointToHash(Point& p)
    {
        return p.X * 1000000 + p.Y;
    };

    void paintingSlicerLayers(Slicer* slicer, Slicer* color_slicer);

    void paintingSlicerLayer(SlicerLayer& slicer_layer, SlicerLayer& color_slicer_layer);

    void paintingPolygonByColorLinePolygons(Polygons& polys, Polygons& color_line_polys, Polygons& color_polys, std::vector<Segment>& color_segments);

    void splitToOpenLineByCollinearSegments(Polygons& polys, Polygons& collinear_polys, std::vector<Line>& open_lines, int color);

    std::unordered_map<int, std::vector<SlicerSegment>> toAngleSlicerSegments(std::vector<SlicerSegment>& slicer_segments, int color);

    void addPaintingColorSegment(Polygons& polys, Polygon& poly, std::vector<Segment>& color_segments, Point& p1, Point& p2, std::unordered_map<int, std::vector<SlicerSegment>>& angle_slicer_segments);

    std::vector<SlicerSegment> linePaintingBySegments(Point& p1, Point& p2, double angle, std::vector<SlicerSegment>& segments);

    void addPointByRandomSmallOffsetMap(Polygon& poly, Point p);

    Polygons toVoronoiColorPolygons(std::vector<Segment>& colored_segments);

    void transferEdge(graph_t& graph, Point from, Point to, vd_t::edge_type& vd_edge, edge_t*& prev_edge, Point& start_source_point, Point& end_source_point, const std::vector<Point>& points, const std::vector<Segment>& segments);

    node_t& makeNode(graph_t& graph, vd_t::vertex_type& vd_node, Point p);

    std::vector<Point> discretize(const vd_t::edge_type& segment, const std::vector<Point>& points, const std::vector<Segment>& segments);

    Line linePolygonsIntersection(Point& p1, Point& p2, Polygons& line_polys);
};

} // namespace cura

#endif // MULTI_MATERIAL_SEGMENTATION_H
