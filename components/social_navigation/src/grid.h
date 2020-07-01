/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GRID_H
#define GRID_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <fstream>
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <limits>
#include <collisions.h>
using namespace std;

#define TILE_SIZE_ 250

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

// Map
struct TCellDefault
{
    std::uint32_t id;
    bool free;
    bool visited;
    float cost;

    // method to save the value
    void save(std::ostream &os) const {	os << free << " " << visited; };
    void read(std::istream &is) {	is >> free >> visited ;};
};

template <typename T = TCellDefault>

class Grid
{
public:
	struct Dimensions
	{
		int TILE_SIZE = 10;
		float HMIN = -2500, VMIN = -2500, WIDTH = 2500, HEIGHT = 2500;
	};

	struct Key
	{
		long int x;
		long int z;

	public:
		Key() : x(0), z(0){};
		Key(long int &&x, long int &&z) : x(std::move(x)), z(std::move(z)){};
		Key(long int &x, long int &z) : x(x), z(z){};
		Key(const long int &x, const long int &z) : x(x), z(z){};
		Key(const QPointF &p)
		{
			x = p.x();
			z = p.y();
		};
		bool operator==(const Key &other) const
		{
			return x == other.x && z == other.z;
		};
		void save(std::ostream &os) const { os << x << " " << z << " "; }; //method to save the keys
		void read(std::istream &is) { is >> x >> z; };					   //method to read the keys
	};

	struct KeyHasher
	{
		std::size_t operator()(const Key &k) const
		{
			using boost::hash_combine;
			using boost::hash_value;

			// Start with a hash value of 0    .
			std::size_t seed = 0;

			// Modify 'seed' by XORing and bit-shifting in one member of 'Key' after the other:
			hash_combine(seed, hash_value(k.x));
			hash_combine(seed, hash_value(k.z));
			return seed;
		};
	};

	using FMap = std::unordered_map<Key, T, KeyHasher>;

    //Grid(){};

	std::tuple<bool, T &> getCell(long int x, long int z)
	{
		if (!(x >= dim.HMIN and x < dim.HMIN + dim.WIDTH and z >= dim.VMIN and z < dim.VMIN + dim.HEIGHT))
			return std::forward_as_tuple(false, T());
		else
			return std::forward_as_tuple(true, fmap.at(pointToGrid(x, z)));
	}
	std::tuple<bool, T &> getCell(const Key &k) //overladed version
	{
		if (!(k.x >= dim.HMIN and k.x < dim.HMIN + dim.WIDTH and k.z >= dim.VMIN and k.z < dim.VMIN + dim.HEIGHT))
			return std::forward_as_tuple(false, T());
		else
			return std::forward_as_tuple(true, fmap.at(pointToGrid(k.x, k.z)));
	}
	T at(const Key &k) const { return fmap.at(k);};
	T &at(const Key &k) { return fmap.at(k);};
	typename FMap::iterator begin() { return fmap.begin(); };
	typename FMap::iterator end() { return fmap.end(); };
	typename FMap::const_iterator begin() const { return fmap.begin(); };
	typename FMap::const_iterator end() const { return fmap.begin(); };
	size_t size() const { return fmap.size(); };


//	void initialize(const Dimensions &dim_, T &&initValue)
//	{
//		dim = dim_;
//		fmap.clear();
//		for (int i = dim.HMIN; i < dim.HMIN + dim.WIDTH; i += dim.TILE_SIZE)
//			for (int j = dim.VMIN; j < dim.VMIN + dim.HEIGHT; j += dim.TILE_SIZE)
//            {
//                fmap.emplace(Key(i, j), initValue);
//            }
//
//		fmap_aux = fmap;
//
//		std::cout << "Grid::Initialize. Grid initialized to map size: " << fmap.size() << std::endl;
//	}



	void initialize(std::shared_ptr<Collisions> collisions_)
    {
	    qDebug()<<"Grid - " <<__FUNCTION__;

	    uint count = 0;
		dim.TILE_SIZE = int(TILE_SIZE_);
		dim.HMIN = std::min(collisions_->outerRegion.left(), collisions_->outerRegion.right());
		dim.WIDTH = std::max(collisions_->outerRegion.left(), collisions_->outerRegion.right()) - dim.HMIN;
		dim.VMIN = std::min(collisions_->outerRegion.top(), collisions_->outerRegion.bottom());
		dim.HEIGHT = std::max(collisions_->outerRegion.top(), collisions_->outerRegion.bottom()) - dim.VMIN;

        fmap.clear();
        fmap_aux.clear();

        qDebug()<<"Collisions - checkRobotValidStateAtTargetFast";

        for (int i = dim.HMIN; i < dim.HMIN + dim.WIDTH; i += dim.TILE_SIZE)
            for (int j = dim.VMIN; j < dim.VMIN + dim.HEIGHT; j += dim.TILE_SIZE)
            {
                bool free = collisions_->checkRobotValidStateAtTargetFast(std::vector<float>{(float)i,10.0,(float)j},std::vector<float>{0.0,0.0,0.0});
                fmap.emplace(Key(i, j), T{count++, free, false, 1.f});
            }

		collisions_->checkRobotValidStateAtTargetFast(std::vector<float>{0.0,10.0,0.0},std::vector<float>{0.0,0.0,0.0}); //para devolver el robot a la posición 0,0

		fmap_aux = fmap;

        std::cout << "Grid::Initialize. Grid initialized to map size: " << fmap.size() << std::endl;
    }

    FMap getMap() { return fmap_aux; }

	void resetGrid()
	{
	    fmap = fmap_aux;
	}

    template <typename Q>
	void insert(const Key &key, const Q &value)
	{
		fmap.insert(std::make_pair(key, value));
	}

	void clear()
	{
		fmap.clear();
	}

	void saveToFile(const std::string &fich)
	{
		std::ofstream myfile;
		myfile.open(fich);
		for (auto &[k, v] : fmap)
		{
			myfile << k << v << std::endl;
		}
		myfile.close();
		std::cout << fmap.size() << " elements written to " << fich << std::endl;
	}

	std::list<QPointF> computePath(const QPointF &source_, const QPointF &target_)
	{
		Key source = pointToGrid(source_.x(), source_.y());
		Key target = pointToGrid(target_.x(), target_.y());


		// Admission rules
		if (!(target.x >= dim.HMIN and target.x < dim.HMIN + dim.WIDTH and target.z >= dim.VMIN and target.z < dim.VMIN + dim.HEIGHT))
		{
			qDebug() << __FUNCTION__ << "Target out of limits. Returning empty path";
			return std::list<QPointF>();
		}
		if (!(source.x >= dim.HMIN and source.x < dim.HMIN + dim.WIDTH and source.z >= dim.VMIN and source.z < dim.VMIN + dim.HEIGHT))
		{
			qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
			return std::list<QPointF>();
		}
		if (source == target)
		{
			qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
			return std::list<QPointF>();
		}
		// vector de distancias inicializado a DBL_MAX
		std::vector<double> min_distance(fmap.size(),std::numeric_limits<double>::max());
		// std::uint32_t id with source value
		auto id = std::get<T &>(getCell(source)).id;
		// initialize source position to 0
		min_distance[id] = 0;
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

		while (!active_vertices.empty())
		{
			Key where = active_vertices.begin()->second;
			if (where == target)
			{
//				qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point
				auto p = orderPath(previous, source, target);
				if (p.size() > 1)
					return p;
				else
					return std::list<QPointF>();
			}
			active_vertices.erase(active_vertices.begin());

            const int &I = dim.TILE_SIZE;
            static const std::vector<int> xincs = {I, I, I, 0, -I, -I, -I, 0};
            static const std::vector<int> zincs = {I, 0, -I, -I, -I, 0, I, I};

			for (auto ed : neighboors(where,xincs,zincs))
			{
//				qDebug() << __FILE__ << __FUNCTION__ << "antes del if" << ed.first.x << ed.first.z << ed.second.id << fmap[where].id << min_distance[ed.second.id] << min_distance[fmap[where].id];
				if (min_distance[ed.second.id] > min_distance[fmap[where].id] + ed.second.cost)
				{
					active_vertices.erase({min_distance[ed.second.id], ed.first});
					min_distance[ed.second.id] = min_distance[fmap[where].id] + ed.second.cost;
					previous[ed.second.id] = std::make_pair(fmap[where].id, where);
					active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
					// active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
				}
			}
		}
		qDebug() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") not  found. Returning empty path";
		return std::list<QPointF>();
	};

	auto pointToGrid(long int x, long int z) const -> decltype(Key())
	{
		int kx = (x - dim.HMIN) / dim.TILE_SIZE;
		int kz = (z - dim.VMIN) / dim.TILE_SIZE;
		return Key(dim.HMIN + kx * dim.TILE_SIZE, dim.VMIN + kz * dim.TILE_SIZE);
	};

	void setFree(const Key &k)
	{
	    if((k.x >= dim.HMIN and k.x < dim.HMIN + dim.WIDTH and k.z >= dim.VMIN and k.z < dim.VMIN + dim.HEIGHT))
		    fmap.at(k).free = true;
	}
	void setOccupied(const Key &k)
	{
        if((k.x >= dim.HMIN and k.x < dim.HMIN + dim.WIDTH and k.z >= dim.VMIN and k.z < dim.VMIN + dim.HEIGHT))
            fmap.at(k).free = false;
	}
    void setCost(const Key &k,float cost)
	{
        if((k.x >= dim.HMIN and k.x < dim.HMIN + dim.WIDTH and k.z >= dim.VMIN and k.z < dim.VMIN + dim.HEIGHT))
            fmap.at(k).cost = cost;
	}

	// if true area becomes free
	void markAreaInGridAs(const QPolygonF &poly, bool free)
	{

		const qreal step = dim.TILE_SIZE / 4;
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
    void modifyCostInGrid(const QPolygonF &poly, float cost)
    {
        const qreal step = dim.TILE_SIZE / 4;
        QRectF box = poly.boundingRect();
        for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
            for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
            {
                if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
                {
                        setCost(pointToGrid(x, y),cost);
                }
            }
    }


	std::tuple<bool, QVector2D> vectorToClosestObstacle(QPointF center)
    {
        QTime reloj = QTime::currentTime();
        qDebug()<<" reloj "<< reloj.restart();

        qDebug()<< "Computing neighboors of " << center;

	    auto k = pointToGrid(center.x(),center.y());

//	    qDebug() << "point in grid "<< k.x << k.z;

	    QVector2D closestVector;
		bool obstacleFound = false;

        const int &I = dim.TILE_SIZE;
        static const std::vector<int> xincs = {I, I, I, 0, -I, -I, -I, 0};
        static const std::vector<int> zincs = {I, 0, -I, -I, -I, 0, I, I};
	    auto neigh = neighboors(k,xincs,zincs, true);

        float dist = std::numeric_limits<float>::max();

	    for (auto n : neigh)
	    {
            if (n.second.free == false)
            {
//                qDebug() << "Neigh "<< QPointF(n.first.x,n.first.z);
				QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x,n.first.z)) ;
                if (vec.length() < dist)
				{
					dist = vec.length();
					closestVector = vec;
				}

				qDebug()<< "Obstacle found";
				obstacleFound = true;
            }
        }

        if (!obstacleFound)
        {
            const int &I = dim.TILE_SIZE;
            static const std::vector<int> xincs = {0,   I,   2*I,  2*I, 2*I, 2*I, 2*I, I, 0, -I, -2*I, -2*I,-2*I,-2*I,-2*I, -I};
            static const std::vector<int> zincs = {2*I, 2*I, 2*I,  I,   0 , -I , -2*I, -2*I,-2*I,-2*I,-2*I, -I, 0,I, 2*I, 2*I};


            auto DistNeigh = neighboors(k,xincs,zincs, true);
            for (auto n : DistNeigh)
            {
                if (n.second.free == false)
                {
//                qDebug() << "Neigh "<< QPointF(n.first.x,n.first.z);
                    QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x,n.first.z)) ;
                    if (vec.length() < dist)
                    {
                        dist = vec.length();
                        closestVector = vec;
                    }

                    qDebug()<< "Obstacle found";
                    obstacleFound = true;
                }
            }

        }

        qDebug()<<" reloj "<< reloj.restart();

        return std::make_tuple(obstacleFound,closestVector);
    }


	std::vector<std::pair<Key, T>> neighboors(const Key &k, const std::vector<int> xincs,const std::vector<int> zincs, bool all = false)
	{
		std::vector<std::pair<Key, T>> neigh;
		// list of increments to access the neighboors of a given position


        for (auto &&[itx, itz] : iter::zip(xincs, zincs))
		{
			Key lk{k.x + itx, k.z + itz};
			try
			{
				T p = fmap.at(Key(lk.x, lk.z));
				T &p_aux = fmap_aux.at(Key(lk.x, lk.z));
				// check that incs are not both zero but have the same abs value, i.e. a diagonal
				if (itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)) and p.cost==1)
							p.cost = 1.41; 								// if neighboor in diagonal, cost is sqrt(2)
				if(all == false)
				{
					if (p.free and p_aux.free)
					{
						neigh.emplace_back(std::make_pair(lk, p));
					}
				}
				else
				{
					neigh.emplace_back(std::make_pair(lk, p));
				}
			}
			catch (const std::exception &e)
			{
				//std::cout << e.what() << " neighbour not found in grid " << lk.x << " " << lk.z << '\n';
			}
		}
		//qDebug() << neigh.size();
		return neigh;
	}

/*    void draw(std::shared_ptr<InnerViewer> viewer)
    {
        qDebug()<<"Grid - " <<__FUNCTION__;

        try	{ viewer->removeNode("IMV_fmap");} catch(const QString &s){	qDebug() << s; };
        try	{ viewer->addTransform_ignoreExisting("IMV_fmap","world");} catch(const QString &s){qDebug() << s; };

        auto normal = QVec::vec3(1,1,0);
        auto size =  QVec::vec3(50,50,50);

        try
        {
            uint i = 0;

            for( const auto &[key,value] : fmap)
            {

                QString item = "IMV_fmap_point_" + QString::number(i);
                if(value.free)
                {
                    if (value.cost == 2.0) //affordance spaces
                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FFFF00", size);
//                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FFFF00", size);
                    else if (value.cost == 3.0) //lowvisited spaces
                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FFBF00", size);
//                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FFFF00", size);
                    else if (value.cost == 4.0) //mediumvisited spaces
                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FF8000", size);
//                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FFFF00", size);
                    else if (value.cost == 5.0) //highVisited spaces
                            viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FF4000", size);
//                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#FFFF00", size);

					else if (value.cost == 8.0) //zona social
                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#00BFFF", size);

                    else if (value.cost == 10.0) //zona personal
                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#BF00FF", size);

                    else
                        viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), normal, "#E6E6E6", size); // Libre
                }

                else
                    viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(key.x, 10, key.z), QVec::vec3(1,1,0), "#B40404", size); //Ocupado

                i++;
            }


        }

        catch(const QString &s) {qDebug() << s;	}
    }*/

private:
	FMap fmap, fmap_aux;
	Dimensions dim;

	/**
		* @brief Recovers the optimal path from the list of previous nodes
		* 
		*/
	std::list<QPointF> orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
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

	inline double heuristicL2(const Key &a, const Key &b) const
	{
		return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
	}
};



#endif // GRID_H
