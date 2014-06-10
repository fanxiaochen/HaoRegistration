#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Point>
#include <osg/ShapeDrawable>

#include "visualizer.h"

Visualizer::Visualizer()
    : viewer_(new osgViewer::Viewer()),
      scene_root_(new osg::Group())
{

}

Visualizer::~Visualizer()
{

}

void Visualizer::init()
{
    viewer_->setSceneData(scene_root_);
    viewer_->addEventHandler(new osgViewer::StatsHandler);
    viewer_->setCameraManipulator(new osgGA::TrackballManipulator);
// Use single thread here to avoid known issues under Linux
    viewer_->setThreadingModel(osgViewer::Viewer::SingleThreaded);
}

void Visualizer::drawImpl(PointCloud *point_cloud)
{
    osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array>  normals = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

    for (size_t i = 0, i_end = point_cloud->size(); i < i_end; i++) {
        const Point &point = point_cloud->at(i);
        vertices->push_back(osg::Vec3(point.x, point.y, point.z));
        normals->push_back(osg::Vec3(point.normal_x, point.normal_y, point.normal_z));
        colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 1.0));
    }

    size_t item_num = vertices->size();

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexData(osg::Geometry::ArrayData(vertices, osg::Geometry::BIND_PER_VERTEX));
    geometry->setNormalData(osg::Geometry::ArrayData(normals, osg::Geometry::BIND_PER_VERTEX));
    geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_PER_VERTEX));
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, item_num));
    geometry->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));

    osg::Geode *geode = new osg::Geode();
    geode->addDrawable(geometry);
    scene_root_->addChild(geode);

    return;
}

void Visualizer::drawSource(PointCloud *source)
{
    drawImpl(source);
    return;
}

void Visualizer::drawTarget(PointCloud *target)
{
    drawImpl(target);
    return;
}

void Visualizer::drawGraph(PointCloud *source)
{
    PointCloud::DeformationGraph* defo_graph = source->getDeformationGraph();
    GraphMap* graph_map = source->getGraphMap();
    
    for (PointCloud::DeformationGraph::NodeIt it(*defo_graph); it != lemon::INVALID; ++ it) {
        Point& point = source->at((*graph_map)[it]);
        osg::Geode *node = new osg::Geode();
        osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(
            new osg::Sphere(osg::Vec3(point.x, point.y, point.z), 1.0f));
        shape->setColor(osg::Vec4(0.8f, 0.8f, 0.4f, 1.0f));
        node->addDrawable(shape);
        scene_root_->addChild(node);
    } 
    
    return;
}


void Visualizer::visualize()
{
    viewer_->run();
}
