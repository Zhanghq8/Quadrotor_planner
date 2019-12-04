#ifndef PRODUCT_AUTOMATON_HPP
#define PRODUCT_AUTOMATON_HPP

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>
#include <tuple>

#include "../map/square_grid.hpp"
#include "../graph/vertex.hpp"
#include "buchi_automaton.hpp"

namespace librav{

class SquareCell;
class ProductState
{   
    public:
        ProductState(int64_t _id):
            id_(_id),
            grid_vertex_(nullptr),
            buchi_vertex_(nullptr){};
        ~ProductState(){};

        int64_t id_;
        Vertex_t<SquareCell *> *  grid_vertex_;
        Vertex_t<BuchiState *, std::vector<int64_t>> * buchi_vertex_;

        
    public:
        int64_t GetUniqueID() const{return id_;};

};

class ProductAutomaton
{
    public:
        static std::shared_ptr<Graph_t<ProductState *>> BuildProductGraph(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph);
        static int64_t SetVirtualStartState(std::shared_ptr<Graph_t<ProductState *>> product_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, std::shared_ptr<Graph_t<SquareCell* >> grid_graph, int64_t grid_start_id);
};

class GetProductNeighbor
{
    public: 
        GetProductNeighbor(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph);
        ~GetProductNeighbor(){};
    
    private:
        std::shared_ptr<Graph_t<SquareCell *>> grid_graph_;
        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph_;

    public:
        void InfoCheck();
        std::vector<std::tuple<ProductState*, double>> operator()(ProductState* cell);
};
}


#endif /* PRODUCT_AUTOMATON_HPP */