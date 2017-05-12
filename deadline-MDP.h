#ifndef __DEADLINE__
#define __DEADLINE__

#define MAX(a,b)         (((a) > (b)) ? (a) : (b))
#define ITER 100

typedef enum{
	Simple_task = 0,
	Synchronization_task,
}task_type;

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <queue>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#include <unordered_map>
boost::mt19937 rng((unsigned)time(0));
boost::uniform_01<boost::mt19937&> u01(rng);

struct node {
	struct node **succ;
	struct node **pred;
	struct instance *instances = NULL;
	int num_instances;
	int num_succ = 0;
	int num_pred = 0;
	int pred_len, succ_len;
	int id;
	double est; // earliest start time
	double lst; // latest  start time = latest finish time - excution time
	double max_time, min_time, max_cost, min_cost;
	task_type type;
	double dl; // deadline
	double eet; //expected excution time
	double rt; // ready time
};

struct instance {
	double reliability;
	double time;
	double cost;
	int id;
	double pheromone;
};

struct solution {
	struct node *n;
	struct instance *inst;
	double st;
	double ct;
};

struct branch {
	double dl; // deadline
	double eet; //expected excution time
	double rt; // ready time
	struct node *head; 
	struct node *tail;
	struct node *start; // the start node of branch is one of child node of head
	int size; 
	double time; // required minimun time 
};

struct hashfunc {
	template<typename T, typename U>
	size_t operator()(const std::pair<T, U> &x) const {
		return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
	}
};

//void reproduct_topos(struct node ***sub_topos, struct node *nodes, int nodes_length, int **topos, int num_topos);

void deadline_MDP(struct node *nodes, int nodes_length, double time_constraint, int *topo, double *total, struct node *start, struct node *end);

void read_data(char *file_name, struct node **nodes_output, int *num_nodes, double *time_constraint, int ***topos, int *num_topos, struct node *start, struct node *end, double **prob);

void calculate_lst(struct node *start, struct node *end, int nodes_length, int *topo, struct solution *s, int *contained_nodes);

void calculate_est(struct node *start, struct node *end, int nodes_length, int *topo, struct solution *s, int *contained_nodes);

double calculate_est_and_lst(struct node *nodes, int nodes_length, int *topo, std::unordered_map<std::pair<int, int>, double, hashfunc> *func_map,
struct node *start, struct node *end, struct solution *s1, struct solution *s2);

double critical_path_makespan(struct node *start, struct node *end, int nodes_length, int *topo, struct solution *s, int *contained_nodes);

void differentiate_nodes(struct node *nodes, int nodes_length);

void partition_nodes(struct node *start, struct node *end, std::vector<struct branch *> *branchs, int nodes_length, int* topo);

void deadline_assignment(struct node *nodes, int nodes_length, int *topo, struct node *start, struct node *end,
	std::vector<struct branch *> *branchs, double time_constrain);

void planning(struct node *nodes, int nodes_length, int *topo, struct node *start, struct node *end,
	std::vector<struct branch *> *branchs, struct solution *s, double time_constrain);

void get_min_index(int *indexs, int k, struct node *head, int size, double *min_gap, int *best_indexs, struct branch *br);

double gap_evaluation(int *indexs, int size, struct node *head, struct branch *br);

int synchronization_task_arrange(struct node *n);

void rt_dl_arrange(struct node *start, struct node *end, std::vector<struct branch *> *branchs, int nodes_length, struct node *nodes, int *topo);

int find_contained_nodes(struct node *start, struct node *end, int *contained_nodes, int *topo);

void topo_deadline(struct node *nodes, int nodes_length, int **topos, int num_topos, struct node *start, struct node *end, double *topo_deadline_prob);

#endif