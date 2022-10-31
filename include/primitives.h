#pragma once
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

class Point
{
public:
    Point(double x, double y);
    Point();

    double x() const;
    double y() const;
    double distance(const Point &) const;

    bool operator<(const Point &) const;
    bool operator>(const Point &) const;
    bool operator<=(const Point &) const;
    bool operator>=(const Point &) const;
    bool operator==(const Point &) const;
    bool operator!=(const Point &) const;

    friend std::ostream & operator<<(std::ostream &, const Point &);

private:
    double _x, _y;
};

class Rect
{
public:
    Rect(const Point & left_bottom, const Point & right_top);

    double xmin() const;
    double ymin() const;
    double xmax() const;
    double ymax() const;
    double distance(const Point & p) const;

    bool contains(const Point & p) const;
    bool intersects(const Rect &) const;

private:
    Point left_bottom;
    Point right_top;
};

namespace rbtree {

class PointSet
{
public:
    class iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const Point *;   // or also value_type*
        using reference = const Point &; // or also value_type&

        iterator(std::set<Point>::iterator it,
                 std::shared_ptr<std::set<Point>> myCollection)
            : it(it)
            , myCollection(std::move(myCollection))
        {
        }

        iterator()
            : it(nullptr)
            , myCollection(nullptr)
        {
        }
        reference operator*() { return *it; }
        pointer operator->() { return &(*it); }

        iterator & operator++()
        {
            ++it;
            return *this;
        }

        iterator operator++(int)
        {
            iterator tmp = *this;
            ++it;
            return tmp;
        }

        friend bool operator==(const iterator & a, const iterator & b) { return a.it == b.it; };
        friend bool operator!=(const iterator & a, const iterator & b) { return a.it != b.it; };

    private:
        std::set<Point>::iterator it;
        std::shared_ptr<std::set<Point>> myCollection;
    };

    PointSet(const std::string & filename = {});

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    // second iterator points to an element out of range
    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    // second iterator points to an element out of range
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

private:
    std::set<Point> set_rbtree;
    //    mutable std::set<std::set<Point>> set_of_storage;
};

} // namespace rbtree

namespace kdtree {

class PointSet
{
private:
    struct Node
    {
        Point value;
        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;
        size_t size;
        size_t rate;
        Node(Point _value, size_t _rate)
            : value(_value)
            , size(1)
            , rate(_rate)
        {
        }
    };
    std::shared_ptr<Node> Root;
    bool funct_for_put(std::shared_ptr<Node> & currentNode_prev, std::shared_ptr<Node> & currentNode, Point p);
    void funct(std::set<Point> & set, Rect r, Rect rect_of_tree, const std::shared_ptr<Node> & currentNode) const;
    void funct_nearest(const std::shared_ptr<Node> & currentNode, Rect r, double & min, Point & min_point, Point p, std::set<Point> & set_point) const;

public:
    class iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const Point *;   // or also value_type*
        using reference = const Point &; // or also value_type&

        struct sset
        {
            std::shared_ptr<Node> value;
            bool left = false;
            bool right = false;
        };

        std::vector<sset> call;
        iterator(const std::shared_ptr<Node> & ptr = nullptr)
            : call({{nullptr}})
            , m_ptr(ptr)
        {
            if (ptr != nullptr) {
                call.push_back({ptr, ptr->left == nullptr, ptr->right == nullptr});
            }
        }

        reference operator*() { return m_ptr->value; }
        pointer operator->() { return &(m_ptr->value); }

        iterator & operator++()
        {
            if (!m_ptr) {
                return *this;
            }
            if (!call.back().left) {
                call.back().left = true;
                m_ptr = m_ptr->left;
                call.push_back({m_ptr, m_ptr->left == nullptr, m_ptr->right == nullptr});
            }
            else if (!call.back().right) {
                call.back().right = true;
                m_ptr = m_ptr->right;
                call.push_back({m_ptr, m_ptr->left == nullptr, m_ptr->right == nullptr});
            }
            else {
                while (call.back().value != nullptr && call.back().left && call.back().right) {
                    call.pop_back();
                    m_ptr = call.back().value;
                }
                operator++();
            }
            return *this;
        }

        iterator operator++(int)
        {
            iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        friend bool operator==(const iterator & a, const iterator & b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!=(const iterator & a, const iterator & b) { return a.m_ptr != b.m_ptr; };

    private:
        std::shared_ptr<Node> m_ptr;
    };

    PointSet(const std::string & filename = {});
    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<iterator, iterator> print_iterator(const std::set<Point> & set) const;
    std::pair<iterator, iterator> range(const Rect &) const;

    iterator begin() const
    {
        if (Root == nullptr) {
            return end();
        }
        return {Root};
    }
    iterator end() const
    {
        return {};
    }
    std::optional<Point> nearest(const Point & p) const;

    std::pair<iterator, iterator> nearest(const Point &, std::size_t) const;

    friend std::ostream & operator<<(std::ostream & s, const PointSet & p)
    {
        s << p.size();
        return s;
    }
};

} // namespace kdtree
