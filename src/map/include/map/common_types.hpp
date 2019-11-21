#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <cstdint>
#include <iostream>

namespace librav{
    enum class OccupancyType
    {
        FREE,
        OCCUPIED,
        UNKNOWN,
        INTERESTED 
    };

    template<typename T>
    struct value2d
    {
        value2d():x(0),y(0){}
        value2d(T _x, T _y): x(_x), y(_y){}

        T x;
        T y;

        // Check whether 2 value2d parameters are same
        bool operator==(const struct value2d &other) const
        {
            if(this->x == other.x && this->y == other.y){
                return true;
            }
            else{
                return false;
            }
        }

        // frined function: std::cout << pos << std::endl;
        friend std::ostream &operator<<(std::ostream &os, const struct value2d &pos)
        {
            os << pos.x << " , " << pos.y;
            return os;
        }    
    };

    using Position2i = value2d<int32_t>;
    using Position2d = value2d<double>;

    template<typename T>
    struct Range2D
    {
        T min;
        T max;
    };

    template<typename T>
    struct BoundingBox
    {
        Range2D<T> x;
        Range2D<T> y;
    };
}


#endif /* COMMON_TYPES_HPP */