#ifndef CBBA_IMPL_HPP
#define CBBA_IMPL_HPP

#include "cbba.hpp"
namespace librav{

	template<typename VehicleType>
	void CBBA::BundleConstruction(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team, TasksSet tasks){
		for(auto&agent: vehicle_team->auto_team_){
			agent->BundleRemove();
		}
		for(auto&agent: vehicle_team->auto_team_){
			agent->BundleAdd(tasks);
		}
	}

	template<typename VehicleType>
	bool CBBA::CheckConvergence(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team){
		bool flag = true;
		for(int64_t jj = 0; jj < vehicle_team->num_tasks_; jj++){
			int64_t winner = -1;
			double highest_bid = -1;

			for(auto& agent: vehicle_team->auto_team_){
				if(winner == -1 && highest_bid == -1){
					winner = agent->cbba_z_(jj);
					highest_bid = agent->cbba_y_(jj);
				}
				else{
					if(agent->cbba_z_(jj) != winner || agent->cbba_y_(jj) != highest_bid){
						flag = false;
						break;
					}
					else{
						continue;
					}
				}
				if(flag==false){break;}
			}
			if(flag == false){break;}
		}
		return flag;
	}

	template<typename VehicleType>
	void CBBA::Consensus(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team){
		//CBBA::neighbor_finder(agent);
		// sender: itself k
		// receiver: i
		// task : j
		for (auto &agent: vehicle_team->auto_team_){
			std::vector<int64_t> neighbors = agent->GetNeighborsIDs();
			// for (int i = 0; i < neighbors.size();i++){
			for(auto& neigb_idx: neighbors){
				AutoVehicle* neighbor = vehicle_team->GetVehicleFromID(neigb_idx);
				for (int j = 0; j < vehicle_team->auto_team_[0]->num_tasks_; j++){
					// Entries 1 to 4
					// if current agent k thinks that the winner of task j is itself k
					if (neighbor->cbba_history_.z_history_.back()(j) == neighbor->idx_){

						/***************************************** Entry 1 ***************************************/
						// Entry 1: Update or leave
						// If the receiver (neighbor) i thinks the winner of task j is also itself i
						if (agent->cbba_z_(j) == agent->idx_){
							// Update
							if (neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j) > EPS_){
								// std::cout << "case 1" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
							// Equal score: require to break the tie
							else if(std::fabs(neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j)) <= EPS_){
								// select the winner of task j as the agent with smaller index
								if (agent->cbba_z_(j) > neighbor->cbba_history_.z_history_.back()(j)){
									// std::cout << "case 2" << std::endl;
									agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
									agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
								}
							}
						}

						/***************************************** Entry 2 ***************************************/
						// Entry 2: Update
						// If the receiver i thinks the winner of task j is also agent k (sender)
						// Update
						else if(agent->cbba_z_(j) == neighbor->idx_){
							// std::cout << "case 3" << std::endl;
							agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
							agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
						}

						/***************************************** Entry 3 ***************************************/

						// Entry 3: Update or Leave
						// If the receiver i thinks the winner of task j is not k or itself i but other agent
						else if (agent->cbba_z_(j) >= 0){
							// Compare the iteration of task j, find which one is the least information of task j
							// Update
							int64_t winner_idx_ = agent->cbba_z_(j);
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// std::cout << "case 4" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
							// Update
							else if(neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j) > EPS_){
								// std::cout << "case 5" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
							// Equal scores: break the tie by selecting the winner as the agent with smaller index
							else if (std::fabs(neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j)) <= EPS_){
								if (agent->cbba_z_(j) > neighbor->cbba_history_.z_history_.back()(j)){
									// std::cout << "case 6" << std::endl;
									agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
									agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
								}
							}
						}

						/***************************************** Entry 4 ***************************************/
						// Entry 4: Update
						// If the agent i (receiver) has no idea about the winner
						else if(agent->cbba_z_(j) == -1){
							// std::cout << "case 7" << std::endl;
							agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
							agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
						}

						else{
							continue;
							// std::cout << "Unknown winner value 1" << std::endl;
						}
					}



					/*********************************************************************************************************/
					/*********************************************************************************************************/
					/*********************************************************************************************************/

					// Entries 5 to 8
					// If current agent i (sender) thinks the winner of task j is agent k (receiver)
					else if (neighbor->cbba_history_.z_history_.back()(j) == agent->idx_){

						/***************************************** Entry 5 ***************************************/
						// Entry 5
						// if agent i (receiver) also agree with agent k (sender): agent i thinks the winner of task j is also itself i
						// Leave
						if (agent->cbba_z_(j) == agent->idx_){
							continue;
							// std::cout << "Do nothing Entry 5" << std::endl;
						}
							

						/***************************************** Entry 6 ***************************************/
						// Entry 6
						// If agent i (receiver) thinks the winner of task j is agent k (sender)
						// Reset (Because the agent k will definitely be the first one to know its own information )
						else if (agent->cbba_z_(j) == neighbor->idx_){
							// std::cout << "case 7" << std::endl;
							agent->cbba_z_(j) = -1;
							agent->cbba_y_(j) = -1;
						}

						/***************************************** Entry 7 ***************************************/
						// Entry 7
						// If agent i thinks the winner of task j is not itself (agent k thinks), but other agent
						else if(agent->cbba_z_(j) >= 0){
							// Compare the iteration of agent k and i, find which one has the least information
							// Reset
							int64_t winner_idx_ = agent->cbba_z_(j);
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// agent k should have the updated information
								// std::cout << "case 8" << std::endl;
								agent->cbba_z_(j) = -1;
								agent->cbba_y_(j) = -1;
							}
						}

						/***************************************** Entry 8 ***************************************/
						// Entry 8
						else if(agent->cbba_z_(j) == -1){
							continue;
							// std::cout <<"Do nothing Entry 8" << std::endl;
						}
							

						else{
							continue;
							// std::cout << "Unknown winner value 2" << std::endl;
						}
							

					}

					/*********************************************************************************************************/
					/*********************************************************************************************************/
					/*********************************************************************************************************/
					// Entries 9 to 13
					// If agent k (sender) thinks the winner of task j is not itself k and not receiver i,but other agent
					else if (neighbor->cbba_history_.z_history_.back()(j) >= 0){
						/***************************************** Entry 9 ***************************************/
						// Entry 9
						// if agent i (receiver) thinks the winner of task j should be itself i
						if (agent->cbba_z_(j) == agent->idx_){
							// compare the iteration that agent k and i talk to the winner that agent k thinks
							// If agent k (sender) has the least information
							int64_t winner_idx_ = neighbor->cbba_history_.z_history_.back()(j);
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// Update
								if (neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j) > EPS_){
									// std::cout << "case 8" << std::endl;
									agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
									agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
								}
								
								// If we have a tie: break the tie by selecting the agent with smaller index as the winner
								else if (std::fabs(neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j)) <= EPS_){
									if (agent->cbba_z_(j) > neighbor->cbba_history_.z_history_.back()(j)){
										// Update
										// std::cout << "case 9" << std::endl;
										agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
										agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
									}
								}
							}
						}

						/***************************************** Entry 10 ***************************************/
						// Entry 10
						// if agent i (receiver) thinks the winner of task j is agent k (sender)
						else if (agent->cbba_z_(j) == neighbor->idx_){
							int64_t winner_idx_ = neighbor->cbba_history_.z_history_.back()(j);
							// Compare the iteration of agent k and i, which one has the least information about the winner that agent k thinks
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// std::cout << "case 10" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
							else{
								// Reset
								// std::cout << "case 11" << std::endl;
								agent->cbba_z_(j) = -1;
								agent->cbba_y_(j) = -1;
							}
						}

						/***************************************** Entry 11 ***************************************/
						// Entry 11
						// If agent i (receiver) agree with agent k and thinks the winner of task j is not i k, but other agent history_z(j)
						else if(agent->cbba_z_(j) == neighbor->cbba_history_.z_history_.back()(j)){
							// Update
							int64_t winner_idx_ = neighbor->cbba_history_.z_history_.back()(j);
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// std::cout << "case 12" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
						}

						/***************************************** Entry 12 ***************************************/
						// Entry 12
						// If agent i (receiver) thinks the winner of task j is not itself, agent k, the one that agent k thinks
						else if (agent->cbba_z_(j) >= 0){
							int64_t winner_idx_ = neighbor->cbba_history_.z_history_.back()(j);
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// Update
								if (neighbor->cbba_history_.iteration_neighb_history_.back()(agent->cbba_z_(j)) > agent->iteration_neighb_(agent->cbba_z_(j))){
									// std::cout << "case 13" << std::endl;
									agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
									agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
								}
								// Update
								if (neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j) > EPS_){
									// std::cout << "case 14" << std::endl;
									agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
									agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
								}
								else if (std::fabs(neighbor->cbba_history_.y_history_.back()(j) - agent->cbba_y_(j)) <= EPS_){
									if(agent->cbba_y_(j) > neighbor->cbba_history_.y_history_.back()(j)){
										// std::cout << "case 15" << std::endl;
										agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
										agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
									}
								}
								else{
									continue;
									// std::cout << "Should not be here Entry 12" << std::endl;
								}
									
							}
							else{
								if (neighbor->cbba_history_.iteration_neighb_history_.back()(agent->cbba_z_(j)) > agent->iteration_neighb_(agent->cbba_z_(j))){
									// Reset
									// std::cout << "case 15" << std::endl;
									agent->cbba_z_(j) = -1;
									agent->cbba_y_(j) = -1;
								}
							}
						}
						/***************************************** Entry 13 ***************************************/
						// Entry 13
						else if(agent->cbba_z_(j) == -1){
							// Update
							int64_t winner_idx_ = neighbor->cbba_history_.z_history_.back()(j);
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(winner_idx_) > agent->iteration_neighb_(winner_idx_)){
								// std::cout << "case 17" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
						}
						else{
							continue;
							// std::cout << "Unknown winner value Entry 13" << std::endl;
						}
							

					}

					/*********************************************************************************************************/
					/*********************************************************************************************************/
					/*********************************************************************************************************/
					// Entries 14 to 17
					else if (neighbor->cbba_history_.z_history_.back()(j) == -1){

						/***************************************** Entry 14 ***************************************/
						// Entry 14
						// Leave
						if (agent->cbba_z_(j) == agent->idx_){
							continue;
							// std::cout << "Do nothing Entry 14" << std::endl;
						}
							

						/***************************************** Entry 15 ***************************************/
						// Entry 15
						// Update
						else if (agent->cbba_z_(j) == neighbor->idx_){
							// std::cout << "case 18" << std::endl;
							agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
							agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
						}

						/***************************************** Entry 16 ***************************************/
						// Entry 16
						// Update
						else if (agent->cbba_z_(j) >= 0){
							// Update
							if (neighbor->cbba_history_.iteration_neighb_history_.back()(agent->cbba_z_(j)) > agent->iteration_neighb_(agent->cbba_z_(j))){
								// std::cout << "case 19" << std::endl;
								agent->cbba_z_(j) = neighbor->cbba_history_.z_history_.back()(j);
								agent->cbba_y_(j) = neighbor->cbba_history_.y_history_.back()(j);
							}
						}

						/***************************************** Entry 17 ***************************************/
						// Entry 17
						// Leave
						else if(agent->cbba_z_(j) == -1){
							continue;
							// std::cout<< "Do noting Entry 17" << std::endl;
						}
							

						else{
							continue;
							// std::cout << "Unknown winner value Entry 17" <<std::endl;
						}
							
					}

					else{
						continue;
						// std::cout << "Unknown winner value end of communicate" <<std::endl;
					}
						
				}

				for (int n = 0; n < vehicle_team->auto_team_.size(); n++){
					if (n != neighbor->idx_ && agent->iteration_neighb_(n) < neighbor->cbba_history_.iteration_neighb_history_.back()(n)){
						agent->iteration_neighb_(n) = neighbor->cbba_history_.iteration_neighb_history_.back()(n);
					}
				}
				agent->iteration_neighb_(neighbor->idx_) = agent->cbba_iter_;
			}	
		}
	}

	template<typename VehicleType>
	void CBBA::ConsensusBasedBundleAlgorithm(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team, TasksSet tasks){
		bool flag = false;
		// Init CBBA
		for(auto&agent: vehicle_team->auto_team_){
			agent->InitCBBA(tasks.tasks_.size());
		}
		vehicle_team->num_tasks_ = tasks.tasks_.size();
		
		
		while (flag != true){
			CBBA::Consensus(vehicle_team);
			for (auto&agent: vehicle_team->auto_team_){
				agent->cbba_history_.iteration_neighb_history_.push_back(agent->iteration_neighb_);
			}
			// std::cout << "After consensus " << std::endl;
			// for(auto& agent: vehicle_team->auto_team_){
			// 	std::cout << "Vehicle " << agent->idx_ << std::endl;
			// 	std::cout << "The info of winner is: " << std::endl;
			// 	std::cout << agent->cbba_z_ << std::endl;
			// 	std::cout << "The info of highest bid is " << std::endl;
			// 	std::cout << agent->cbba_y_ << std::endl;
			// 	std::cout << "The task assignment result is ";
			// 	for (auto& tt: agent->task_path_){
			// 		std::cout << tt << ", ";
			// 	} 
			// 	std::cout << std::endl;
			// 	std::cout << "========================================================" << std::endl;
			// }
			CBBA::BundleConstruction(vehicle_team,tasks);
			// std::cout << "After bundle construction " << std::endl;
			// for(auto& agent: vehicle_team->auto_team_){
			// 	std::cout << "Vehicle " << agent->idx_ << std::endl;
			// 	std::cout << "The info of winner is: " << std::endl;
			// 	std::cout << agent->cbba_z_ << std::endl;
			// 	std::cout << "The info of highest bid is " << std::endl;
			// 	std::cout << agent->cbba_y_ << std::endl;
			// 	std::cout << "The task assignment result is ";
			// 	for (auto& tt: agent->task_path_){
			// 		std::cout << tt << ", ";
			// 	} 
			// 	std::cout << std::endl;
			// 	std::cout << "========================================================" << std::endl;
			// }
			flag = CBBA::CheckConvergence(vehicle_team);
			if(flag == true){
				break;
			}
			for(auto& agent: vehicle_team->auto_team_){
				agent->cbba_iter_ ++;
			}
		}
		
		/* Debug of CBBA */
		std::cout << "The convergence of CBBA is achieved. " << std::endl;
		std::cout << "The number of iteration is " << vehicle_team->auto_team_.front()->cbba_iter_ << std::endl;
		for(auto& agent: vehicle_team->auto_team_){
			std::cout << "Vehicle " << agent->idx_ << std::endl;
			std::cout << "The info of winner is: " << std::endl;
			std::cout << agent->cbba_z_ << std::endl;
			std::cout << "The info of highest bid is " << std::endl;
			std::cout << agent->cbba_y_ << std::endl;
			std::cout << "The task assignment result is ";
			for (auto& tt: agent->task_path_){
				std::cout << tt << ", ";
			} 
			std::cout << std::endl;
			std::cout << "========================================================" << std::endl;
		}
	}
}
#endif /* CBBA_IMPL_HPP */

