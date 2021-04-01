#include "grid.h"
#include <QVector2D>
#include <QGraphicsRectItem>
#include <execution>
#include <algorithm>
# include <cppitertools/slice.hpp>
template <typename T>
void Grid<T>::initialize(std::shared_ptr<DSR::DSRGraph> graph_,
                         std::shared_ptr<Collisions> collisions_,
                         bool read_from_file,
                         const std::string &file_name,
                         std::uint16_t num_threads)
{
    qDebug() << __FUNCTION__ << "FileName:" << QString::fromStdString(file_name);
    G = graph_;
    qInfo() << __FUNCTION__ <<  "World dimension: " << dim << TILE_SIZE;
    fmap.clear();
    // check that robot exists
    std::optional<int> robot_id = G->get_id_from_name(robot_name);
    if(not robot_id.has_value())
    {
        qInfo() << __FUNCTION__ << " No robot with name " << QString::fromStdString(robot_name);
        std::terminate();
    }

    if(read_from_file and not file_name.empty())
        readFromFile(file_name);

    else  // compute occupancy
    {
        std::thread threads[num_threads];
        std::vector<std::vector<float>> coordinates;
        std::vector<float> rot{0.0, 0.0, 0.0};
        for (float i = dim.left(); i < dim.right(); i += TILE_SIZE)
            for (float j = dim.bottom(); j < dim.top(); j += TILE_SIZE)
                coordinates.emplace_back(std::vector<float>{i,j, 10.f});
        std::cout << __FUNCTION__ << " Creating occupation grid with " << num_threads << " threads for "  << coordinates.size() << " positions" << std::endl;
        std::cout << __FUNCTION__ << " Estimated time: " << coordinates.size()*12/1000/60.f << " minutes"  << std::endl;
        auto start_time = Myclock::now();
        std::vector<std::vector<std::pair<Key,T>>> value_vector(num_threads);
        for(int &&k : iter::range(num_threads))
        {
            std::vector<std::vector<float>> my_coordinates(coordinates.begin()+k*coordinates.size()/num_threads, coordinates.begin()+(k+1)*coordinates.size()/num_threads-1);
            threads[k] = std::thread([this, collisions_, my_coordinates, &value_vector, k, rot]() {
                auto G_copy = G->G_copy();
                std::transform(my_coordinates.begin(), my_coordinates.end(), std::back_inserter(value_vector[k]),
                               [collisions_, &G_copy, rot](auto &pos) {
                                    auto[free, node_name] = collisions_->checkRobotValidStateAtTargetFast(G_copy, pos, rot);
                                    return std::make_pair(Key(pos[0], pos[1]), T{0, free, true, 1.f, node_name});
                               });
            });
        }
        for(auto &&k : iter::range(num_threads))
            threads[k].join();

        // add to grid results from threads
        uint32_t count = 0;  // add id now so they are unique
        for(auto &res : value_vector)
            for (auto &[key, val] : res)
            {
                val.id = count++;
                fmap.emplace(key, val);
            }

        auto duration = Myclock::now() - start_time;
        std::cout << __FUNCTION__ << " " << count << " elements inserted.  It took " << std::chrono::duration_cast<std::chrono::seconds>(duration).count() << "secs" << std::endl;

        ////////////////
        if(not file_name.empty())
            saveToFile(file_name);
    }
}


template <typename T>
std::tuple<bool, T &> Grid<T>::getCell(long int x, long int z)
{
    if( not dim.contains(QPointF(x,z)))
        return std::forward_as_tuple(false, T());
    else
        return std::forward_as_tuple(true, fmap.at(pointToGrid(x, z)));
}

template <typename T>
std::tuple<bool, T &> Grid<T>::getCell(const Key &k) //overladed version
{
//    if (not dim.contains(QPointF(k.x, k.z)))
//        return std::forward_as_tuple(false, T());
//    else
//        return std::forward_as_tuple(true, fmap.at(pointToGrid(k.x, k.z)));  // Key should already be correct
      try{ return std::forward_as_tuple(true, fmap.at(pointToGrid(k.x, k.z))); }
      catch(...){ qInfo() << __FUNCTION__ << " No key found in grid: (" << k.x << k.z << ")" ; return std::forward_as_tuple(false, T()) ;}
}

template <typename T>
typename Grid<T>::Key Grid<T>::pointToGrid(long int x, long int z) const
{
    int kx = (x - dim.left()) / TILE_SIZE;
    int kz = (z - dim.bottom()) / TILE_SIZE;
    return Key(dim.left() + kx * TILE_SIZE, dim.bottom() + kz * TILE_SIZE);
};

////////////////////////////////////////////////////////////////////////////////

template <typename T>
void Grid<T>::saveToFile(const std::string &fich)
{
    std::ofstream myfile;
    myfile.open(fich);
    for (auto &[k, v] : fmap)
        myfile << k << v << std::endl;

    myfile.close();
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements written to " << fich << std::endl;
}

template <typename T>
void Grid<T>::readFromFile(const std::string &fich)
{
    std::ifstream myfile(fich);
    std::string line;
    std::uint32_t count = 0;
    if (!myfile)
    {
        std::cout << fich << " No file found" << std::endl;
        std::terminate();
    }
    while ( std::getline (myfile, line) )
    {
        //std::cout << line << std::endl;
        std::stringstream ss(line);
        int x, z;
        bool free, visited;
        std::string node_name;
        ss >> x >> z >> free >> visited >> node_name;
        fmap.emplace(pointToGrid(x, z), T{count++, free, false, 1.f, node_name});
    }
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements read from " << fich << std::endl;
}


template <typename T>
bool Grid<T>::isFree(const Key &k)
{
    const auto &[success, v] = getCell(k);
    if(success)
        return v.free;
    else
        return false;
}

template <typename T>
void Grid<T>::setFree(const Key &k)
{
    auto &[success, v] = getCell(k);
    if(success)
        v.free = true;
}

template <typename T>
bool Grid<T>::cellNearToOccupiedCellByObject(const Key &k, const std::string &target_name)
{
    auto neigh = this->neighboors_8(k, true);
    for(const auto &[key, val] : neigh)
        if(val.free==false and val.node_name==target_name)
            return true;
    return false;
}

template <typename T>
void Grid<T>::setOccupied(const Key &k)
{
    auto &[success, v] = getCell(k);
    if(success)
        v.free = false;
}

template <typename T>
void Grid<T>::setCost(const Key &k,float cost)
{
    auto &[success, v] = getCell(k);
    if(success)
        v.cost = cost;
}

// if true area becomes free
template <typename T>
void Grid<T>::markAreaInGridAs(const QPolygonF &poly, bool free)
{
    const qreal step = TILE_SIZE / 4;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
        {
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
            {
                if (free)
                    setFree(pointToGrid(x, y));
                else
                    setOccupied(pointToGrid(x, y));
            }
        }
}

template <typename T>
void Grid<T>::modifyCostInGrid(const QPolygonF &poly, float cost)
{
    const qreal step = TILE_SIZE / 4.f;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
                setCost(pointToGrid(x, y),cost);
}

template <typename T>
std::tuple<bool, QVector2D> Grid<T>::vectorToClosestObstacle(QPointF center)
{
    QTime reloj = QTime::currentTime();
    qDebug()<<" reloj "<< reloj.restart();
    qDebug()<< "Computing neighboors of " << center;
    auto k = pointToGrid(center.x(),center.y());
    QVector2D closestVector;
    bool obstacleFound = false;

    auto neigh = neighboors_8(k, true);
    float dist = std::numeric_limits<float>::max();
    for (auto n : neigh)
    {
        if (n.second.free == false)
        {
            QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x,n.first.z)) ;
            if (vec.length() < dist)
            {
                dist = vec.length();
                closestVector = vec;
            }
            qDebug() << __FUNCTION__ << "Obstacle found";
            obstacleFound = true;
        }
    }

    if (!obstacleFound)
    {
        auto DistNeigh = neighboors_16(k, true);
        for (auto n : DistNeigh)
        {
            if (n.second.free == false)
            {
                QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x, n.first.z)) ;
                if (vec.length() < dist)
                {
                    dist = vec.length();
                    closestVector = vec;
                }
                obstacleFound = true;
            }
        }
    }
    return std::make_tuple(obstacleFound,closestVector);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
std::list<QPointF> Grid<T>::computePath(const QPointF &source_, const QPointF &target_)
{
    Key source = pointToGrid(source_.x(), source_.y());
    Key target = pointToGrid(target_.x(), target_.y());

    // Admission rules
    if (not dim.contains(target_))
    {
        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "out of limits " << dim << " Returning empty path";
        return std::list<QPointF>();
    }
    if (not dim.contains(source_))
    {
        qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
        return std::list<QPointF>();
    }
    if (source == target)
    {
        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
        return std::list<QPointF>();
    }
    const auto &[success, val] = getCell(source);
    if(not success)
    {
        qWarning() << "Could not find source position in Grid";
        return std::list<QPointF>();
    }

    // vector de distancias inicializado a UINT_MAX
    std::vector<uint32_t> min_distance(fmap.size(),std::numeric_limits<uint32_t>::max());
    // initialize source position to 0
    min_distance[val.id] = 0;
    // vector de pares<std::uint32_t,Key> initialized to (-1, Key())
    std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));
    // lambda to compare two vertices: a < b if a.id<b.id or
    auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y) {
        if (x.first <= y.first)
            return true;
            //else if(x.first == y.first)
            //	return std::get<T&>(getCell(x.second)).id <= std::get<T&>(getCell(y.second)).id;
        else
            return false;
    };

    // OPEN List
    std::set<std::pair<std::uint32_t, Key>, decltype(comp)> active_vertices(comp);
    active_vertices.insert({0, source});

    while (not active_vertices.empty())
    {
        Key where = active_vertices.begin()->second;
        if (where == target)
        {
            // qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point
            auto p = orderPath(previous, source, target);
            if (p.size() > 1)
                return p;
            else
                return std::list<QPointF>();
        }
        active_vertices.erase(active_vertices.begin());
        for (auto ed : neighboors_8(where))
        {
//				qDebug() << __FILE__ << __FUNCTION__ << "antes del if" << ed.first.x << ed.first.z << ed.second.id << fmap[where].id << min_distance[ed.second.id] << min_distance[fmap[where].id];
            if (min_distance[ed.second.id] > min_distance[fmap.at(where).id] + ed.second.cost)
            {
                active_vertices.erase({min_distance[ed.second.id], ed.first});
                min_distance[ed.second.id] = min_distance[fmap.at(where).id] + ed.second.cost;
                previous[ed.second.id] = std::make_pair(fmap.at(where).id, where);
                active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
                // active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
            }
        }
    }
    qDebug() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") not  found. Returning empty path";
    return std::list<QPointF>();
};

template <typename T>
std::vector<std::pair<typename Grid<T>::Key, T>> Grid<T>::neighboors(const Grid<T>::Key &k, const std::vector<int> &xincs,const std::vector<int> &zincs, bool all)
{
    std::vector<std::pair<Key, T>> neigh;
    // list of increments to access the neighboors of a given position
    for (auto &&[itx, itz] : iter::zip(xincs, zincs))
    {
        Key lk{k.x + itx, k.z + itz};
        const auto &[success, p] = getCell(lk);
        if(not success) continue;

        // check that incs are not both zero but have the same abs value, i.e. a diagonal
        if (itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)) and p.cost==1)
            p.cost = 1.41; 								// if neighboor in diagonal, cost is sqrt(2)

        if(all)
            neigh.emplace_back(std::make_pair(lk, p));
        else
            if (p.free)
                neigh.emplace_back(std::make_pair(lk, p));
    }
    return neigh;
}

template <typename T>
std::vector<std::pair<typename Grid<T>::Key, T>> Grid<T>::neighboors_8(const Grid<T>::Key &k, bool all)
{
    const int &I = TILE_SIZE;
    static const std::vector<int> xincs = {I, I, I, 0, -I, -I, -I, 0};
    static const std::vector<int> zincs = {I, 0, -I, -I, -I, 0, I, I};
    auto r = this->neighboors(k, xincs, zincs, all);
    return r;
}

template <typename T>
std::vector<std::pair<typename Grid<T>::Key, T>> Grid<T>::neighboors_16(const Grid<T>::Key &k, bool all)
{
    const int &I = TILE_SIZE;
    static const std::vector<int> xincs = {0,   I,   2*I,  2*I, 2*I, 2*I, 2*I, I, 0, -I, -2*I, -2*I,-2*I,-2*I,-2*I, -I};
    static const std::vector<int> zincs = {2*I, 2*I, 2*I,  I,   0 , -I , -2*I, -2*I,-2*I,-2*I,-2*I, -I, 0,I, 2*I, 2*I};
    return this->neighboors(k, xincs, zincs, all);
}

/**
 @brief Recovers the optimal path from the list of previous nodes
*/
template <typename T>
std::list<QPointF> Grid<T>::orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
{
    std::list<QPointF> res;
    Key k = target;
    std::uint32_t u = fmap.at(k).id;
    while (previous[u].first != (std::uint32_t)-1)
    {
        res.push_front(QPointF(k.x, k.z));
        u = previous[u].first;
        k = previous[u].second;
    }
    //qDebug() << __FILE__ << __FUNCTION__ << "Path length:" << res.size();  //exit point
    return res;
};

template <typename T>
inline double Grid<T>::heuristicL2(const Key &a, const Key &b) const
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
}

template <typename T>
void Grid<T>::draw(QGraphicsScene* scene)
{
    //clear previous points
    for (QGraphicsRectItem* item : scene_grid_points)
        scene->removeItem((QGraphicsItem*)item);

    scene_grid_points.clear();
    //create new representation
    std::string color;
    for( const auto &[key,value] : fmap)
    {
        if(value.free)
        {
            if (value.cost == 2.0) //affordance spaces
                color = "#FFFF00";
            else if (value.cost == 3.0) //lowvisited spaces
                color = "#FFBF00";
            else if (value.cost == 4.0) //mediumvisited spaces
                color = "#FF8000";
            else if (value.cost == 5.0) //highVisited spaces
                color = "#FF4000";
            else if (value.cost == 8.0) //zona social
                color = "#00BFFF";
            else if (value.cost == 10.0) //zona personal
                color = "#BF00FF";
            else
                color = "LightGreen";
        }
        else
            color = "#B40404";

        QColor my_color = QColor(QString::fromStdString(color));
        my_color.setAlpha(40);
        QGraphicsRectItem* aux = scene->addRect(-TILE_SIZE, -TILE_SIZE, TILE_SIZE, TILE_SIZE, QPen(my_color), QBrush(my_color));
        aux->setZValue(1);
        aux->setPos(key.x, key.z);
        scene_grid_points.push_back(aux);
    }
}

template <typename T>
void Grid<T>::clear()
{
    fmap.clear();
}

template <class T>
auto operator<<(std::ostream &os, const T &t) -> decltype(t.save(os), os)
{
    t.save(os);
    return os;
};
template <class T>
auto operator>>(std::istream &is, T &t) -> decltype(t.read(is), is)
{
    t.read(is);
    return is;
};
