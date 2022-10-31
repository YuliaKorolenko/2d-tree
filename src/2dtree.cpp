
#include "primitives.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

const double INF = std::numeric_limits<double>::infinity();

Point::Point(double x, double y)
    : _x(x)
    , _y(y)
{
}

double Point::x() const
{
    return _x;
}
double Point::y() const
{
    return _y;
}
double Point::distance(const Point & point) const
{
    return std::hypot((_x - point.x()), (_y - point.y()));
}
bool Point::operator<(const Point & point) const
{
    if (_x == point.x()) {
        return _y < point.y();
    }
    return _x < point.x();
}
bool Point::operator>(const Point & point) const
{
    if (_x == point.x()) {
        return _y > point.y();
    }
    return _x > point.x();
}
bool Point::operator<=(const Point & point) const
{
    return !(operator>(point));
}
bool Point::operator>=(const Point & point) const
{
    return !(operator<(point));
}
bool Point::operator==(const Point & point) const
{
    if (_x == point.x()) {
        return _y == point.y();
    }
    return _x == point.x();
}
bool Point::operator!=(const Point & point) const
{
    return !(operator==(point));
}
std::ostream & operator<<(std::ostream & os, const Point & p)
{
    os << p.x() << " " << p.y();
    return os;
}

//Rect

Rect::Rect(const Point & _left_bottom, const Point & _right_top)
    : left_bottom(_left_bottom)
    , right_top(_right_top)
{
}

double Rect::xmin() const
{
    return left_bottom.x();
}
double Rect::ymin() const
{
    return left_bottom.y();
}
double Rect::xmax() const
{
    return right_top.x();
}
double Rect::ymax() const
{
    return right_top.y();
}

double Rect::distance(const Point & p) const
{
    if (contains(p))
        return 0;
    double answer;
    if (left_bottom.x() <= p.x() && p.x() <= right_top.x()) {
        answer = std::min(std::abs(left_bottom.y() - p.y()), std::abs(right_top.y() - p.y()));
    }
    else if (left_bottom.y() <= p.y() && right_top.y() >= p.y()) {
        answer = std::min(std::abs(left_bottom.x() - p.x()), std::abs(right_top.x() - p.x()));
    }
    else {
        answer = std::min({p.distance(left_bottom), p.distance(right_top), p.distance(Point(left_bottom.x(), right_top.y())), p.distance(Point(right_top.x(), left_bottom.y()))});
    }
    return answer;
}
bool Rect::contains(const Point & p) const
{
    return (xmin() <= p.x() && xmax() >= p.x() && ymin() <= p.y() && ymax() >= p.y());
}
bool Rect::intersects(const Rect & rect) const
{
    if (contains(Point(rect.xmin(), rect.ymin())) || contains(Point(rect.xmin(), rect.ymax())) || contains(Point(rect.xmax(), rect.ymin())) || contains(Point(rect.xmin(), rect.ymin()))) {
        return true;
    }
    else if (rect.contains(left_bottom) || rect.contains(right_top) || rect.contains(Point(left_bottom.x(), right_top.y())) || rect.contains(Point(right_top.x(), left_bottom.y()))) {
        return true;
    }
    else {
        return false;
    }
}

namespace rbtree {

PointSet::PointSet(const std::string & filename)
{
    std::ifstream file;
    file.open(filename);
    double x, y;
    while (file >> x) {
        file >> y;
        put(Point(x, y));
    }
}

bool PointSet::empty() const
{
    return set_rbtree.empty();
}

void PointSet::put(const Point & point)
{
    set_rbtree.insert(point);
}

std::size_t PointSet::size() const
{
    return set_rbtree.size();
}

bool PointSet::contains(const Point & point) const
{
    return (set_rbtree.find(point) != set_rbtree.end());
}

rbtree::PointSet::iterator PointSet::begin() const
{
    return iterator(set_rbtree.begin(), std::make_shared<std::set<Point>>(set_rbtree));
    //    return set_rbtree.begin();
}
rbtree::PointSet::iterator PointSet::end() const
{
    return iterator(set_rbtree.end(), std::make_shared<std::set<Point>>(set_rbtree));
}

std::optional<Point> PointSet::nearest(const Point & p) const
{
    Point near_point = *set_rbtree.begin();
    double near_distance = p.distance(near_point);
    for (const auto item : set_rbtree) {
        if (near_distance >= p.distance(item)) {
            near_distance = p.distance(item);
            near_point = item;
        }
    }
    return near_point;
}
std::pair<PointSet::iterator, PointSet::iterator> PointSet::range(const Rect & r) const
{
    std::shared_ptr<std::set<Point>> current_set = std::make_shared<std::set<Point>>();
    for (const auto i : PointSet::set_rbtree) {
        if (r.contains(i)) {
            current_set->insert(i);
        }
    }
    return {iterator(current_set->begin(), current_set),
            iterator(current_set->end(), current_set)};
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    std::shared_ptr<std::set<Point>> current_set = std::make_shared<std::set<Point>>();
    k = std::min(k, set_rbtree.size());
    for (size_t j = 0; j < k; j++) {
        Point near_point = Point(0, 0);
        double near_distance = INF;
        for (const auto item : PointSet::set_rbtree) {
            if (current_set->find(item) == current_set->end() && near_distance >= p.distance(item)) {
                near_distance = p.distance(item);
                near_point = item;
            }
        }
        current_set->insert(near_point);
    }
    //    auto pos = PointSet::set_of_storage.insert(current_set);
    return {iterator(current_set->begin(), current_set),
            iterator(current_set->end(), current_set)};
}

std::ostream & operator<<(std::ostream & os, const PointSet & p)
{
    for (const auto i : p) {
        os << i << "\n";
    }
    return os;
}
} // namespace rbtree

namespace kdtree {

PointSet::PointSet(const std::string & filename)
{
    std::ifstream file;
    file.open(filename);
    double x, y;
    while (file >> x) {
        file >> y;
        put(Point(x, y));
    }
}

bool PointSet::funct_for_put(std::shared_ptr<Node> & currentNode_prev, std::shared_ptr<Node> & currentNode, Point p)
{
    if (currentNode != nullptr) {
        currentNode_prev = currentNode;
    }
    else {
        std::shared_ptr<Node> right = std::make_shared<Node>(p, (currentNode_prev->rate + 1) % 2);
        currentNode = right;
        return true;
    }
    return false;
}

void PointSet::put(const Point & p)
{
    std::shared_ptr<Node> currentNode = Root;
    if (Root == nullptr) {
        Root = std::make_shared<Node>(p, 0);
        return;
    }
    while (true) {
        if (currentNode->value == p) {
            break;
        }
        currentNode->size++;
        if ((currentNode->rate == 0 && p.x() >= currentNode->value.x()) ||
            (currentNode->rate == 1 && p.y() >= currentNode->value.y())) {
            if (funct_for_put(currentNode, currentNode->right, p)) {
                break;
            }
        }
        else {
            if (funct_for_put(currentNode, currentNode->left, p)) {
                break;
            }
        }
    }
}

bool PointSet::empty() const
{
    if (Root == nullptr) {
        return true;
    }
    return false;
}
std::size_t PointSet::size() const
{
    if (Root == nullptr) {
        return 0;
    }
    return Root->size;
}

bool PointSet::contains(const Point & p) const
{
    if (empty()) {
        return false;
    }
    std::shared_ptr<Node> currentNode = Root;
    while (true) {
        if (currentNode->value == p) {
            return true;
        }
        if ((currentNode->rate == 0 && p.x() >= currentNode->value.x()) ||
            (currentNode->rate == 1 && p.y() >= currentNode->value.y())) {
            if (currentNode->right != nullptr) {
                currentNode = currentNode->right;
            }
            else {
                return false;
            }
        }
        else {
            if (currentNode->left != nullptr) {
                currentNode = currentNode->left;
            }
            else {
                return false;
            }
        }
    }
}

void PointSet::funct(std::set<Point> & set, Rect r, Rect rect_of_tree, const std::shared_ptr<Node> & currentNode) const
{
    if (currentNode == nullptr) {
        return;
    }
    if (!r.intersects(rect_of_tree)) {
        return;
    }
    if (r.contains(currentNode->value)) {
        set.insert(currentNode->value);
    }
    // for left
    funct(set, r, Rect(Point(rect_of_tree.xmin(), rect_of_tree.ymin()), Point(currentNode->rate == 0 ? currentNode->value.x() : rect_of_tree.xmax(), currentNode->rate == 1 ? currentNode->value.y() : rect_of_tree.ymax())), currentNode->left);
    // for right
    funct(set, r, Rect(Point(currentNode->rate == 0 ? currentNode->value.x() : rect_of_tree.xmin(), currentNode->rate == 1 ? currentNode->value.y() : rect_of_tree.ymin()), Point(rect_of_tree.xmax(), rect_of_tree.ymax())), currentNode->right);
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::print_iterator(const std::set<Point> & set) const
{
    std::shared_ptr<Node> current_root = std::make_shared<Node>(*set.begin(), 0);
    auto it = set.begin();
    std::shared_ptr<Node> first_element = current_root;
    while (++it != set.end()) {
        current_root->left = std::make_shared<Node>(*it, (current_root->rate + 1) % 2);
        current_root = current_root->left;
    }
    return {{first_element}, {}};
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::range(const Rect & r) const
{
    std::set<Point> set;
    funct(set, r, Rect(Point(-INF, -INF), Point(INF, INF)), Root);

    if (set.empty()) {
        return {};
    }
    return print_iterator(set);
}

void PointSet::funct_nearest(const std::shared_ptr<Node> & currentNode, Rect r, double & min, Point & min_point, Point p, std::set<Point> & set_point) const
{
    if (currentNode == nullptr) {
        return;
    }
    if (r.distance(p) >= min) {
        return;
    }
    else {
        if (set_point.find(currentNode->value) == set_point.end() && currentNode->value.distance(p) < min) {
            min = currentNode->value.distance(p);
            min_point = currentNode->value;
        }
    }
    // for left
    funct_nearest(currentNode->left,
                  Rect(Point(r.xmin(), r.ymin()),
                       Point(currentNode->rate == 0 ? currentNode->value.x() : r.xmax(),
                             currentNode->rate == 1 ? currentNode->value.y() : r.ymax())),
                  min,
                  min_point,
                  p,
                  set_point);
    // for right
    funct_nearest(currentNode->right, Rect(Point(currentNode->rate == 0 ? currentNode->value.x() : r.xmin(), currentNode->rate == 1 ? currentNode->value.y() : r.ymin()), Point(r.xmax(), r.ymax())), min, min_point, p, set_point);
}

std::optional<Point> PointSet::nearest(const Point & p) const
{
    std::set<Point> set_point;
    Point min_point = Point(INF, INF);
    double min = INF;
    funct_nearest(Root, Rect(Point(-INF, -INF), Point(INF, INF)), min, min_point, p, set_point);
    return min_point;
}

std::pair<PointSet::iterator, PointSet::iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    std::set<Point> set_point;
    k = std::min(PointSet::size(), k);
    if (k == 0) {
        return {{}, {}};
    }
    for (size_t i = 0; i < k; ++i) {
        Point min_point = Point(INF, INF);
        double min = INF;
        funct_nearest(Root, Rect(Point(-INF, -INF), Point(INF, INF)), min, min_point, p, set_point);
        set_point.insert(min_point);
    }
    return print_iterator(set_point);
}

} // namespace kdtree
