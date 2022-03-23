#pragma once


#include <pcl/kdtree/kdtree.h>
#include "log_duration.h"


namespace lidar_course {


template<typename PointT>
auto dbscan(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr,  const float eps = 0.25f, const size_t minPts = 4){

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud_ptr);

    const size_t UNDEFINED = 0;
    const size_t NOISE = 1;
    size_t cluster_id = NOISE;

    std::vector<size_t> labels(input_cloud_ptr->size(), UNDEFINED);

    {

        LOG_DURATION("Dbscan");

        for (size_t i = 0; i < input_cloud_ptr->size(); ++i){

            if (labels[i] != UNDEFINED){
                continue;
            }

            const auto point = input_cloud_ptr->points[i];

            std::vector<int> neighbours;
            std::vector<float> ndists;

            //kdtree.setEpsilon(eps);
            //kdtree.nearestKSearch(point, 4, neighbours, ndists);
        
            //std::sort(ndists.begin(), ndists.end());
       
            kdtree.radiusSearch(point, eps, neighbours, ndists); //RangeQuery

            if (neighbours.size() < minPts){ // Density check
                labels[i] = NOISE;
                continue;
            }

            cluster_id += 1;

            labels[i] = cluster_id;

            std::vector<int> bfs_queue;

            for (const int nbour_id : neighbours){
                if (nbour_id == i){
                    continue;
                }
                bfs_queue.push_back(nbour_id);
            }


            while (!bfs_queue.empty()){
                int index_q = bfs_queue.back();
                bfs_queue.pop_back();
                const auto point_q = input_cloud_ptr->points[index_q];

                if (labels[index_q] == NOISE){
                    labels[index_q] = cluster_id;
                }

                if (labels[index_q] != UNDEFINED){
                    continue;
                }

                labels[index_q] = cluster_id;

                std::vector<int> neighbours;
                std::vector<float> ndists;

                kdtree.radiusSearch(point_q, eps, neighbours, ndists); // RangeQuery

                if (neighbours.size() >= minPts){ // Density check
                    for (const int nbour_id : neighbours){
                        bfs_queue.push_back(nbour_id);
                    }
                }
            }
        }
    }

    auto labeled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

    for (size_t i = 0; i < input_cloud_ptr->size(); ++i){
        auto point = input_cloud_ptr->points[i];
        pcl::PointXYZL labeled_point;
        pcl::copyPoint(point, labeled_point);
        labeled_point.label = labels[i];
        labeled_cloud->push_back(labeled_point);
    }

  
    return labeled_cloud;
}

}