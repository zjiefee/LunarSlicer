#include "MultiMaterialSegmentation.h"
#include "Application.h"
#include "BoostInterface.hpp"
#include "Scene.h"
#include "SkeletalTrapezoidation.h"
#include "Slice.h"
#include "boost/polygon/voronoi.hpp"
#include "utils/VoronoiUtils.h"
#include "utils/linearAlg2D.h"
#include "utils/polygonUtils.h"

namespace cura
{

class Slice;

void MultiMaterialSegmentation::multiMaterialSegmentationByPainting(Slicer* slicer, Slicer* color_slicer)
{
    MultiMaterialSegmentation mms;
    mms.paintingSlicerLayers(slicer, color_slicer);
}

void MultiMaterialSegmentation::paintingSlicerLayers(Slicer* slicer, Slicer* color_slicer)
{
    for (int i = 5; i < slicer->layers.size(); ++i)
    {
        color_slicer->layers.emplace_back();
        auto& color_layer = color_slicer->layers[i];
        auto& layer = slicer->layers[i];
        if (layer.segments_colors.size() <= 1)
        {
            continue;
        }

        paintingSlicerLayer(layer, color_layer);
    }
}

void MultiMaterialSegmentation::paintingSlicerLayer(SlicerLayer& slicer_layer, SlicerLayer& color_slicer_layer)
{
    Polygons color_polys;
    std::vector<Segment> color_segments;
    slicer_layer.polygons.print();
    std::cout << "[";
    for (int i = 0; i < slicer_layer.segments.size(); ++i)
    {
        if (slicer_layer.segments[i].color != 1) {
            continue;
        }
        auto segment = slicer_layer.segments[i];
        std::cout << "[";
        std::cout << segment.start.X << "," << segment.start.Y << ",";
        std::cout << segment.end.X << "," << segment.end.Y;
        std::cout << "],";
    }
    std::cout << "]" << std::endl;

    std::unordered_map<int, std::vector<SlicerSegment>> angle_slicer_segments = toAngleSlicerSegments(slicer_layer.segments, 1);

    Polygons& layer_polys = slicer_layer.polygons;
    layer_polys.scale(scale);

    for (int i = 0; i < layer_polys.size(); ++i)
    {
        Polygon color_poly;
        for (int j = 0; j < layer_polys[i].size(); ++j)
        {
            auto& p1 = layer_polys[i][j];
            auto& p2 = layer_polys[i][(j + 1) % layer_polys[i].size()];
            addPaintingColorSegment(color_polys, color_poly, color_segments, p1, p2, angle_slicer_segments);
        }
        color_polys.add(color_poly);
    }

    std::vector<Segment> test_segments;
    for (int i = 0; i < layer_polys[0].size(); ++i)
    {
        test_segments.emplace_back(&layer_polys, 0, i);
    }

    int color = 0;
    for (int i = 0; i < color_segments.size(); ++i)
    {
        if (color_segments[i].color == 1) {
            color++;
        }
    }
    std::cout << "cout:" << color << " - " << color_segments.size() << std::endl;

    color_polys.print();

    Polygons voronoi_color_polys;
    toVoronoiColorPolygons(color_segments, voronoi_color_polys);
    voronoi_color_polys.print();

    voronoi_color_polys = voronoi_color_polys.unionPolygons();
    voronoi_color_polys = voronoi_color_polys.intersection(slicer_layer.polygons);
    Polygons other_slicer_polys = slicer_layer.polygons.difference(voronoi_color_polys);
    color_slicer_layer.polygons = voronoi_color_polys;
    slicer_layer.polygons = other_slicer_polys;
}

std::unordered_map<int, std::vector<SlicerSegment>> MultiMaterialSegmentation::toAngleSlicerSegments(std::vector<SlicerSegment>& slicer_segments, int color)
{
    std::unordered_map<int, std::vector<SlicerSegment>> angle_slicer_segments;

    for (int i = 0; i < slicer_segments.size(); ++i)
    {
        auto segment = slicer_segments[i];
        segment.start = segment.start * scale;
        segment.end = segment.end * scale;
        if (segment.color != color)
        {
            continue;
        }
        int line_angle1 = angle((segment.end - segment.start));
//        int line_angle2 = angle((segment.start - segment.end));

        if (angle_slicer_segments.find(line_angle1) == angle_slicer_segments.end())
        {
            angle_slicer_segments[line_angle1] = std::vector<SlicerSegment>();
        }

        angle_slicer_segments[line_angle1].emplace_back(segment);

//        if (angle_slicer_segments.find(line_angle2) == angle_slicer_segments.end())
//        {
//            angle_slicer_segments[line_angle2] = std::vector<SlicerSegment>();
//        }
//
//        angle_slicer_segments[line_angle2].emplace_back(segment);
    }

    return angle_slicer_segments;
}

void MultiMaterialSegmentation::addPaintingColorSegment(Polygons& polys, Polygon& poly, std::vector<Segment>& color_segments, Point& p1, Point& p2, std::unordered_map<int, std::vector<SlicerSegment>>& angle_slicer_segments)
{
    Point const l1 = p2 - p1;
    double const l1_angled = angleDouble(l1);
    int l1_angle = static_cast<int>(l1_angled);
    std::cout << "[" << p1.X << "," << p1.Y << "," << p2.X << "," << p2.Y << "]" << std::endl;
    int const angle_range[3] = { -1, 0, 1 };

    AABB aabb;
    aabb.include(p1);
    aabb.include(p2);

    auto lineLineIntersection = [&l1_angled, &aabb, &p1, &p2](Point& p3, Point& p4)
    {
        Point const l2 = p4 - p3;
        double const l2_angled = angleDouble(l2);

        if (std::abs(l1_angled - l2_angled) > 1)
        {
            return false;
        }

        AABB aabb2;
        aabb2.include(p3);
        aabb2.include(p4);

        if (! aabb.contains(aabb2))
        {
            return false;
        }

        if (cross(p1 - p4, p3 - p4) * cross(p2 - p4, p3 - p4) < 0 || cross(p4 - p2, p1 - p2) * cross(p3 - p2, p1 - p2) > 0)
        {
            return false;
        }

        return true;
    };

    std::vector<SlicerSegment> intersect_segments;

    for (int i = 0; i < 3; ++i)
    {
        int l_angle_r = l1_angle + angle_range[i];
        if (l_angle_r < 0)
            l_angle_r += 360;
        if (l_angle_r > 360)
            l_angle_r -= 360;

        if (angle_slicer_segments.find(l_angle_r) == angle_slicer_segments.end())
        {
            continue;
        }

        auto& slicer_segments = angle_slicer_segments[l_angle_r];
        for (int j = 0; j < slicer_segments.size(); ++j)
        {
            if (lineLineIntersection(slicer_segments[j].start, slicer_segments[j].end))
            {
                intersect_segments.emplace_back(slicer_segments[j]);
            }
        }
    }

    if (intersect_segments.empty())
    {
        color_segments.emplace_back(&polys, polys.size(), poly.size(), 0);
        poly.add(p1);
    }
    else
    {
        std::vector<SlicerSegment> res = linePaintingBySegments(p1, p2, l1_angled, intersect_segments);
        for (int i = 0; i < res.size(); ++i)
        {
            color_segments.emplace_back(&polys, polys.size(), poly.size(), res[i].color);
            poly.add(res[i].start);
        }
    }
}

std::vector<SlicerSegment> MultiMaterialSegmentation::linePaintingBySegments(Point& p1, Point& p2, double angle, std::vector<SlicerSegment>& segments)
{
    std::vector<SlicerSegment> res;

    struct PointIndex
    {
        int value;
        int color_value = 0;
        int segment_index = 0;
        int segment_point_index = 0;
    };

    int len = segments.size() * 2 + 2;
    PointIndex point_indexes[len];

    int x_or_y = 0;
    int sort = 0;
    if ((45 < angle && angle < 135) || (225 < angle && angle < 315))
    {
        x_or_y = 0;
        sort = p1.X > p2.X;
        point_indexes[0].value = p1.X;
        point_indexes[0].color_value = 0;
        point_indexes[1].value = p2.X;
        point_indexes[1].color_value = 0;
    }
    else
    {
        x_or_y = 1;
        sort = p1.Y > p2.Y;
        point_indexes[0].value = p1.Y;
        point_indexes[0].color_value = 0;
        point_indexes[1].value = p2.Y;
        point_indexes[1].color_value = 0;
    }

    for (int i = 0; i < segments.size(); ++i)
    {
        auto segment = segments[i];
        int j = i * 2 + 2;
        point_indexes[j].segment_index = i;
        int segment_sort = x_or_y == 0 ? segment.start.X > segment.end.X : segment.start.Y > segment.end.Y;
        point_indexes[j].segment_point_index = sort ^ segment_sort;
        point_indexes[j].color_value = point_indexes[j].segment_point_index == 0 ? 1 : -1;
        point_indexes[j].value = x_or_y == 0 ? segment[point_indexes[j].segment_point_index].X : segment[point_indexes[j].segment_point_index].Y;

        point_indexes[j + 1].segment_index = i;
        point_indexes[j + 1].segment_point_index = ~point_indexes[j].segment_point_index;
        point_indexes[j + 1].color_value = -point_indexes[j].color_value;
        point_indexes[j + 1].value = x_or_y == 0 ? segment[point_indexes[j + 1].segment_point_index].X : segment[point_indexes[j + 1].segment_point_index].Y;
    }

    std::sort(point_indexes,
              point_indexes + len,
              [&sort](PointIndex& a, PointIndex& b)
              {
                  if (sort == 0)
                  {
                      return a.value < b.value;
                  }
                  else
                  {
                      return a.value > b.value;
                  }
              });

    int color = 0;
    bool is_start = false;
    bool last_color = false;
    Point* last_p;
    for (int i = 0; i < len; ++i)
    {
        color += point_indexes[i].color_value;
        assert(color >= 0);

        if (is_start)
        {
            if (point_indexes[i].color_value == 0)
            {
                res.emplace_back();
                res.back().start = *last_p;
                res.back().end = p2;
                res.back().color = color > 0 ? 1 : 0;
                break;
            }
            bool cur_color = color > 0;
            if (last_color != cur_color)
            {
                res.emplace_back();
                res.back().start = *last_p;
                res.back().end = segments[point_indexes[i].segment_index][point_indexes[i].segment_point_index];
                res.back().color = color > 0 ? 0 : 1;

                last_p = &segments[point_indexes[i].segment_index][point_indexes[i].segment_point_index];
                last_color = cur_color;
            }
        }
        else
        {
            if (point_indexes[i].color_value == 0)
            {
                is_start = true;
                last_p = &p1;
                last_color = color > 0;
            }
        }
    }

    for (auto iterator = res.begin(); iterator != res.end();)
    {
        if (iterator->start == iterator->end) {
            iterator = res.erase(iterator);
        } else {
            iterator++;
        }
    }

    return res;
}

void MultiMaterialSegmentation::toVoronoiColorPolygons(std::vector<Segment>& colored_segments, Polygons& voronoi_color_polys)
{
    vd_t vonoroi_diagram;
    construct_voronoi(colored_segments.begin(), colored_segments.end(), &vonoroi_diagram);

    for (int i = 0; i < vonoroi_diagram.cells().size(); ++i)
    {
        auto& cell = vonoroi_diagram.cells()[i];
        if (! cell.incident_edge() && !cell.contains_segment())
        {
            continue;
        }
        auto& segment = colored_segments[cell.source_index()];
        if (segment.color <= 0)
        {
            continue;
        }

        Polygon poly;
        auto from = segment.from();
        auto to = segment.to();

        auto* start = cell.incident_edge();
        bool is_circle = false;
        do
        {
            if (start->vertex0() == nullptr && start->vertex1()) {
                break;
            }
            start = start->prev();

            if (start == cell.incident_edge()) {
                is_circle = true;
            }
        } while (start != cell.incident_edge());

        if (!is_circle) {
            poly.add(to);
        }
        auto* next = start;
        do
        {
            if (next->vertex0() && next->vertex1() == nullptr) {
                break;
            }
            poly.add(Point(next->vertex1()->x(), next->vertex1()->y()));
            next = next->next();
        } while (next != start);
        if (!is_circle) {
            poly.add(from);
        }
        voronoi_color_polys.add(poly);
    }
}

void MultiMaterialSegmentation::transferEdge(graph_t& graph,
                                             Point from,
                                             Point to,
                                             vd_t::edge_type& vd_edge,
                                             edge_t*& prev_edge,
                                             Point& start_source_point,
                                             Point& end_source_point,
                                             const std::vector<Point>& points,
                                             const std::vector<Segment>& segments)
{
    auto he_edge_it = vd_edge_to_he_edge.find(vd_edge.twin());
    if (he_edge_it != vd_edge_to_he_edge.end())
    { // Twin segment(s) have already been made
        edge_t* source_twin = he_edge_it->second;
        assert(source_twin);
        auto end_node_it = vd_node_to_he_node.find(vd_edge.vertex1());
        assert(end_node_it != vd_node_to_he_node.end());
        node_t* end_node = end_node_it->second;
        for (edge_t* twin = source_twin;; twin = twin->prev)
        {
            if (! twin)
            {
                spdlog::warn("Encountered a voronoi edge without twin.");
                continue; // Prevent reading unallocated memory.
            }
            assert(twin);
            graph.edges.emplace_front(SkeletalTrapezoidationEdge());
            edge_t* edge = &graph.edges.front();
            edge->from = twin->to;
            edge->to = twin->from;
            edge->twin = twin;
            twin->twin = edge;
            edge->from->incident_edge = edge;

            if (prev_edge)
            {
                edge->prev = prev_edge;
                prev_edge->next = edge;
            }

            prev_edge = edge;

            if (prev_edge->to == end_node)
            {
                return;
            }

            if (! twin->prev)
            {
                spdlog::error("Discretized segment behaves oddly!");
                return;
            }

            assert(twin->prev); // Forth rib

            constexpr bool is_not_next_to_start_or_end = false; // Only ribs at the end of a cell should be skipped
        }
        assert(prev_edge);
    }
    else
    {
        std::vector<Point> discretized = discretize(vd_edge, points, segments);
        assert(discretized.size() >= 2);
        if (discretized.size() < 2)
        {
            spdlog::warn("Discretized Voronoi edge is degenerate.");
        }

        assert(! prev_edge || prev_edge->to);
        if (prev_edge && ! prev_edge->to)
        {
            spdlog::warn("Previous edge doesn't go anywhere.");
        }
        node_t* v0 = (prev_edge) ? prev_edge->to : &makeNode(graph, *vd_edge.vertex0(), from); // TODO: investigate whether boost:voronoi can produce multiple verts and violates consistency
        Point p0 = discretized.front();
        for (size_t p1_idx = 1; p1_idx < discretized.size(); p1_idx++)
        {
            Point p1 = discretized[p1_idx];
            node_t* v1;
            if (p1_idx < discretized.size() - 1)
            {
                graph.nodes.emplace_front(SkeletalTrapezoidationJoint(), p1);
                v1 = &graph.nodes.front();
            }
            else
            {
                v1 = &makeNode(graph, *vd_edge.vertex1(), to);
            }

            graph.edges.emplace_front(SkeletalTrapezoidationEdge());
            edge_t* edge = &graph.edges.front();
            edge->from = v0;
            edge->to = v1;
            edge->from->incident_edge = edge;

            if (prev_edge)
            {
                edge->prev = prev_edge;
                prev_edge->next = edge;
            }

            prev_edge = edge;
            p0 = p1;
            v0 = v1;

            if (p1_idx < discretized.size() - 1)
            { // Rib for last segment gets introduced outside this function!
                constexpr bool is_not_next_to_start_or_end = false; // Only ribs at the end of a cell should be skipped
            }
        }
        assert(prev_edge);
        vd_edge_to_he_edge.emplace(&vd_edge, prev_edge);
    }
}

MultiMaterialSegmentation::node_t& MultiMaterialSegmentation::makeNode(graph_t& graph, vd_t::vertex_type& vd_node, Point p)
{
    auto he_node_it = vd_node_to_he_node.find(&vd_node);
    if (he_node_it == vd_node_to_he_node.end())
    {
        graph.nodes.emplace_front(SkeletalTrapezoidationJoint(), p);
        node_t& node = graph.nodes.front();
        vd_node_to_he_node.emplace(&vd_node, &node);
        return node;
    }
    else
    {
        return *he_node_it->second;
    }
}


std::vector<Point> MultiMaterialSegmentation::discretize(const vd_t::edge_type& vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    /*Terminology in this function assumes that the edge moves horizontally from
    left to right. This is not necessarily the case; the edge can go in any
    direction, but it helps to picture it in a certain direction in your head.*/
    coord_t discretization_step_size = MM2INT(0.8) * scale;
    const AngleRadians transitioning_angle(10 / 180 * M_PI);

    const vd_t::cell_type* left_cell = vd_edge.cell();
    const vd_t::cell_type* right_cell = vd_edge.twin()->cell();
    Point start = VoronoiUtils::p(vd_edge.vertex0());
    Point end = VoronoiUtils::p(vd_edge.vertex1());

    bool point_left = left_cell->contains_point();
    bool point_right = right_cell->contains_point();
    if ((! point_left && ! point_right) || vd_edge.is_secondary()) // Source vert is directly connected to source segment
    {
        return std::vector<Point>({ start, end });
    }
    else if (point_left != point_right) // This is a parabolic edge between a point and a line.
    {
        Point p = VoronoiUtils::getSourcePoint(*(point_left ? left_cell : right_cell), points, segments);
        const Segment& s = VoronoiUtils::getSourceSegment(*(point_left ? right_cell : left_cell), points, segments);
        return VoronoiUtils::discretizeParabola(p, s, start, end, discretization_step_size, transitioning_angle);
    }
    else // This is a straight edge between two points.
    {
        /*While the edge is straight, it is still discretized since the part
        becomes narrower between the two points. As such it may need different
        beadings along the way.*/
        Point left_point = VoronoiUtils::getSourcePoint(*left_cell, points, segments);
        Point right_point = VoronoiUtils::getSourcePoint(*right_cell, points, segments);
        coord_t d = vSize(right_point - left_point);
        Point middle = (left_point + right_point) / 2;
        Point x_axis_dir = turn90CCW(right_point - left_point);
        coord_t x_axis_length = vSize(x_axis_dir);

        const auto projected_x = [x_axis_dir, x_axis_length, middle](Point from) // Project a point on the edge.
        {
            Point vec = from - middle;
            coord_t x = dot(vec, x_axis_dir) / x_axis_length;
            return x;
        };

        coord_t start_x = projected_x(start);
        coord_t end_x = projected_x(end);

        // Part of the edge will be bound to the markings on the endpoints of the edge. Calculate how far that is.
        float bound = 0.5 / tan((M_PI - transitioning_angle) * 0.5);
        coord_t marking_start_x = -d * bound;
        coord_t marking_end_x = d * bound;
        Point marking_start = middle + x_axis_dir * marking_start_x / x_axis_length;
        Point marking_end = middle + x_axis_dir * marking_end_x / x_axis_length;
        int direction = 1;

        if (start_x > end_x) // Oops, the Voronoi edge is the other way around.
        {
            direction = -1;
            std::swap(marking_start, marking_end);
            std::swap(marking_start_x, marking_end_x);
        }

        // Start generating points along the edge.
        Point a = start;
        Point b = end;
        std::vector<Point> ret;
        ret.emplace_back(a);

        // Introduce an extra edge at the borders of the markings?
        bool add_marking_start = marking_start_x * direction > start_x * direction;
        bool add_marking_end = marking_end_x * direction > start_x * direction;

        // The edge's length may not be divisible by the step size, so calculate an integer step count and evenly distribute the vertices among those.
        Point ab = b - a;
        coord_t ab_size = vSize(ab);
        coord_t step_count = (ab_size + discretization_step_size / 2) / discretization_step_size;
        if (step_count % 2 == 1)
        {
            step_count++; // enforce a discretization point being added in the middle
        }
        for (coord_t step = 1; step < step_count; step++)
        {
            Point here = a + ab * step / step_count; // Now simply interpolate the coordinates to get the new vertices!
            coord_t x_here = projected_x(here); // If we've surpassed the position of the extra markings, we may need to insert them first.
            if (add_marking_start && marking_start_x * direction < x_here * direction)
            {
                ret.emplace_back(marking_start);
                add_marking_start = false;
            }
            if (add_marking_end && marking_end_x * direction < x_here * direction)
            {
                ret.emplace_back(marking_end);
                add_marking_end = false;
            }
            ret.emplace_back(here);
        }
        if (add_marking_end && marking_end_x * direction < end_x * direction)
        {
            ret.emplace_back(marking_end);
        }
        ret.emplace_back(b);
        return ret;
    }
}

void MultiMaterialSegmentation::addPointByRandomSmallOffsetMap(Polygon& poly, Point p)
{
    if (scale < 10)
    {
        poly.add(p);
    }
    else
    {
        int mod = scale / 10 - 1;
        int x_random = rand() % mod + 1;
        int y_random = rand() % mod + 1;
        if (rand() / 2 == 1)
        {
            x_random = -x_random;
        }
        if (rand() / 2 == 1)
        {
            y_random = -y_random;
        }
        Point rp = Point(p.X + x_random, p.Y + y_random);
        poly.add(rp);
        mms_random_small_offset_point[&poly.back()] = p;
    }
}

} // namespace cura