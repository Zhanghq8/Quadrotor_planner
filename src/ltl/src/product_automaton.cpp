#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <bitset>

#include "ltl/product_automaton.hpp"

using namespace librav;

std::shared_ptr<Graph_t<ProductState *>> ProductAutomaton::BuildProductGraph(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph)
{
    std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    // Obtain all grid vertex
    std::vector<Vertex_t<SquareCell *> *> grid_vertices = GridGraph->GetAllVertex();

    //Obtain all buchi vertex
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = BuchiGraph->GetAllVertex();
    
    int num_buchi_states = buchi_vertices.size();
    std::vector<int64_t> alphabet_set = (*BuchiGraph->FindVertex(0)).state_->alphabet_set;

    int64_t node_2;

    for (auto gs = grid_vertices.begin(); gs!= grid_vertices.end(); gs++){
        // Find all neighbors
        std::vector<Vertex_t<SquareCell *> *> neighbor_from_grid = (*gs)->GetNeighbours();
        for (auto ngs = neighbor_from_grid.begin(); ngs != neighbor_from_grid.end(); ngs++){
            for (auto bs1 = buchi_vertices.begin(); bs1 != buchi_vertices.end(); bs1++){
                for (auto bs2 = buchi_vertices.begin(); bs2 != buchi_vertices.end(); bs2++){
                    std::vector<int64_t> buchi_transition = (*bs1)->GetEdgeCost(*bs2);
                    int32_t cell_bit_map = (*ngs)->state_->GetCellBitMap();
                    for(auto &e_buchi_trans: buchi_transition){
                        if(e_buchi_trans > alphabet_set.size()-1){
                            continue;
                        }
                        if(alphabet_set[e_buchi_trans] == cell_bit_map){
                            
                            int64_t node_1_id = (*gs)->state_->id_ * num_buchi_states + (*bs1)->state_->id_;
                            int64_t node_2_id = (*ngs)->state_->id_ * num_buchi_states + (*bs2) -> state_->id_;

                            ProductState* new_product_state_1 =  new ProductState(node_1_id);
                            new_product_state_1->grid_vertex_ = (*gs);
                            new_product_state_1->buchi_vertex_ = (*bs1);

                            ProductState* new_product_state_2 = new ProductState(node_2_id);
                            new_product_state_2->grid_vertex_ = (*ngs);
                            new_product_state_2->buchi_vertex_ = (*bs2);

                            product_graph->AddEdge(new_product_state_1,new_product_state_2,1.0);
                            break;

                            // ==================================== DEBUG ======================================================//
                            // std::cout << "Physical vertex transition is " << (*gs)->state_->id_ << " -> " << (*ngs) -> state_->id_ << std::endl;
                            // std::cout << "The dst bit map is " << cell_bit_map << std::endl;
                            // std::cout << "Buchi transition is " << (*bs1)->state_->id_ << " -> " << (*bs2)->state_->id_ << std::endl;
                            // std::cout << "The buchi transition is ";
                            // for (auto &e: buchi_transition){
                            //     std::cout << e << " ";
                            // }
                            // std::cout << std::endl;
                            // ==================================== DEBUG ======================================================//
                        }
                           
                    }
                                       
                }
            }
        }

    };

    return product_graph;
};

int64_t ProductAutomaton::SetVirtualStartState(std::shared_ptr<Graph_t<ProductState *>> product_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, std::shared_ptr<Graph_t<SquareCell* >> grid_graph, int64_t grid_start_id)
{
    int64_t virtual_start_id;

    // Buchi State
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph->GetAllVertex();
    int num_buchi_states = buchi_vertices.size();
    int64_t buchi_init_id = (*buchi_graph->FindVertex(0)).state_->init_state_idx_;
    Vertex_t<BuchiState *, std::vector<int64_t>> * buchi_init = buchi_graph->GetVertexFromID(buchi_init_id);

    // Grid State
    Vertex_t<SquareCell *> * grid_init = grid_graph->GetVertexFromID(grid_start_id);

    virtual_start_id = grid_init->state_->id_ * num_buchi_states + buchi_init->state_->id_;

    ProductState* virtual_start = new ProductState(virtual_start_id);
    virtual_start->grid_vertex_ = grid_init;
    virtual_start->buchi_vertex_ = buchi_init;

    product_graph->AddVertex(virtual_start);
    return virtual_start_id;
};

GetProductNeighbor::GetProductNeighbor(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph):
grid_graph_(GridGraph),
buchi_graph_(BuchiGraph){};


void GetProductNeighbor::InfoCheck()
{
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph_->GetAllVertex();
    for (auto &e: buchi_vertices){
        std::cout << e->state_->id_ << " ";
    }
    std::cout << std::endl;
}


std::vector<std::tuple<ProductState*, double>> GetProductNeighbor::operator()(ProductState* cell)
{
    std::vector<std::tuple<ProductState*, double>> neighbors;

    //Obtain all buchi vertex
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph_->GetAllVertex();
    
    int num_buchi_states = buchi_vertices.size();
    std::vector<int64_t> alphabet_set = (*buchi_graph_->FindVertex(0)).state_->alphabet_set;

    Vertex_t<SquareCell *>* current_grid_state =  cell->grid_vertex_;
    std::vector<Vertex_t<SquareCell *> *> neighbor_from_grid = current_grid_state->GetNeighbours();
    Vertex_t<BuchiState *, std::vector<int64_t>> * current_buchi_state = cell->buchi_vertex_;

    for (auto ngs = neighbor_from_grid.begin(); ngs != neighbor_from_grid.end(); ngs++){
        for (auto bs = buchi_vertices.begin(); bs != buchi_vertices.end(); bs++){
            std::vector<int64_t> buchi_transition = (current_buchi_state)->GetEdgeCost(*bs);
            int32_t cell_bit_map = (*ngs)->state_->GetCellBitMap();
            for(auto &e_buchi_trans: buchi_transition){
                if(e_buchi_trans > alphabet_set.size()-1){
                    continue;
                }
                if(alphabet_set[e_buchi_trans] == cell_bit_map){
                        
                    int64_t new_node_id = (*ngs)->state_->id_ * num_buchi_states + (*bs) -> state_->id_;
                    ProductState* new_product_state = new ProductState(new_node_id);
                    new_product_state->grid_vertex_ = (*ngs);
                    new_product_state->buchi_vertex_ = (*bs);

                    neighbors.emplace_back(new_product_state,1.0);
                
                    // //==================================== DEBUG ======================================================//
                    // std::cout << "Physical vertex transition is " << cell->grid_vertex_->state_->id_ << " -> " << (*ngs) -> state_->id_ << std::endl;
                    // std::cout << "The dst bit map is " << cell_bit_map << std::endl;
                    // std::cout << "Buchi transition is " << cell->buchi_vertex_->state_->id_ << " -> " << (*bs)->state_->id_ << std::endl;
                    // std::cout << "The buchi transition is ";
                    // for (auto &e: buchi_transition){
                    //     std::cout << e << " ";
                    // }
                    // std::cout << std::endl;
                    // //==================================== DEBUG ======================================================//
                    break;
                }
                           
            }
                                       
        }
    }
    return neighbors;
}

