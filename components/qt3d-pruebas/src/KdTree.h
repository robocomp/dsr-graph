//
// Created by juancarlos on 21/9/21.
//

#ifndef QT3D_PRUEBAS_KDTREE_H
#define QT3D_PRUEBAS_KDTREE_H


#include <vector>
#include <memory>
#include <flann/flann.h>


class KdTree
{
public:

    typedef std::vector<size_t> IndiceVec;
    typedef std::vector<float> DistanceVec;
    typedef Eigen::Vector3f Point;

    KdTree(std::vector<std::tuple<float, float, float>>&& pointcloud)
        : m_dimension(3),
          m_dataset_size(pointcloud.size()),
          m_data(std::move(pointcloud)),
          m_flann_dataset(new flann::Matrix<float>((float *)m_data.data(),
            m_dataset_size, m_dimension)),
          m_flann_index(new flann::Index<flann::L2_Simple<float>>(
                  *m_flann_dataset, flann::KDTreeSingleIndexParams(10)))
    {
        m_flann_index->buildIndex();
    }


    std::pair<std::vector<IndiceVec>, std::vector<DistanceVec>> KNN(const std::vector<Point> &query, size_t k)
    {

        flann::Matrix<float> query_ ((float* ) query.data(), query.size(), m_dimension);
        flann::SearchParams param_(-1, 0.0);
        //    param.max_neighbors = -1;
        std::vector<std::vector<size_t>> indices_vec_(1);
        std::vector<std::vector<float>> dists_vec_(1);
        m_flann_index->knnSearch(query_, indices_vec_, dists_vec_, k, param_);

        //std::cout << "res: " <<res << " " << indices_vec_.size() << " " << dists_vec_.size() << std::endl;
        return { std::move(indices_vec_), std::move(dists_vec_)};
    }

    std::tuple<float, float, float> get_pos (size_t idx) {
        return m_data[idx];
    }

private:

    size_t m_dimension = 0;
    size_t m_dataset_size = 0;
    std::vector<std::tuple<float, float, float>> m_data;
    std::unique_ptr<flann::Matrix<float>> m_flann_dataset;
    std::unique_ptr<flann::Index<flann::L2_Simple<float>>> m_flann_index;

};


#endif //QT3D_PRUEBAS_KDTREE_H
