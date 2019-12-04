#ifndef GRID_CELL_IMPL_HPP
#define GRID_CELL_IMPL_HPP

#include <map>
#include <cmath>
#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <assert.h>

#include "common_types.hpp"
#include "../graph/graph.hpp"


namespace librav{
    class GridCellBase{
        public: 
            GridCellBase(int64_t id, int64_t col, int64_t row, OccupancyType occ):
                id_(id), occupancy_(occ){
                    coordinate_.x = col;
                    coordinate_.y = row;
                }
            ~GridCellBase(){};
        public:
            // Grid cell in simulated map
            int64_t id_;
            Position2i coordinate_;
            OccupancyType occupancy_;

            // Grid cell in real map
            Position2d position_;
            BoundingBox<int32_t> bbox_;

        public: 
            int64_t GetUniqueID() const {return this->id_;};

            void PrintCellInfo() const{
                std::string occ;
                switch(occupancy_)
                {
                    case OccupancyType::FREE:
                        occ = "free";
                        break;
                    case OccupancyType::UNKNOWN:
                        occ = "unknown";
                        break;
                    case OccupancyType::OCCUPIED:
                        occ = "occupied";
                        break;
                }
                std::cout << "Square cell id: " << this->id_ << ", coordinate: (" << this->coordinate_.x << "," << this->coordinate_.y << ")"
                                                << ", position: (" << this->position_.x << ","<< this->position_.y << ")"
                                                << ", occupancy: " << occ << std::endl;
            }

        public:
            void UpdateCellInfo(int32_t row_size, int32_t col_size, double cell_size, int32_t pixel_per_meter){
                int32_t vis_side_size = cell_size * pixel_per_meter;

                bbox_.x.min = coordinate_.x * vis_side_size;
                bbox_.x.max = bbox_.x.min + vis_side_size - 1;
                bbox_.y.min = coordinate_.y * vis_side_size;
                bbox_.y.max = bbox_.y.min + vis_side_size - 1;

                position_.x = coordinate_.x * vis_side_size + vis_side_size/2.0;
                position_.y = coordinate_.y * vis_side_size + vis_side_size/2.0;
            }
    };
}
#endif /* GRID_CELL_IMPL_HPP */