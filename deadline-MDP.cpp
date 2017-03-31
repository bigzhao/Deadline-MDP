#include "deadline-MDP.h"


int main()
{
	struct node *nodes = NULL/*, *start = NULL, *end = NULL*/;
	double time_constraint;
	int nodes_length, num_topos;
	int **topos = NULL;
	struct node *start = NULL, *end = NULL;
	start = (struct node *)malloc(sizeof(struct node));
	memset(start, 0, sizeof(struct node));
	end = (struct node *)malloc(sizeof(struct node));
	memset(end, 0, sizeof(struct node));
	double *prob = NULL;
	read_data("j601_1.txt", &nodes, &nodes_length, &time_constraint, &topos, &num_topos, start, end, &prob);
	for (int i = 1; i < num_topos; i++)
		prob[i] += prob[i - 1];
	double total = 0;
	int index;
	// boost random function
	//for (int i = 0; i < num_topos; i++) {
	//	for (int j = 0; j < nodes_length + 2; j++)
	//		printf("%d ", topos[i][j]);no 
	//	printf("\n");
	//}

	// eet 标识start end节点
	FILE *fp = NULL;
	fp = fopen("permute.txt", "w");
	fprintf(fp, "%d\n", ITER);
	printf("num:%d\n", num_topos);
	boost::uniform_int<> ui(0, num_topos - 1);   /*函数声明 0-100（包括100）的整数*/
	for (int i = 0; i < ITER; i++) {
		double p = u01();
		for (int j = 0; j < num_topos; j++) {
			if (p < prob[j]) {
				index = j;
				break;
			}
		}
		//index = ui(rng);
		fprintf(fp, "%d ", index);
		printf("index: %d ", index);
		deadline_MDP(nodes, nodes_length, time_constraint, topos[index], &total, start, end);
	}
	printf("%d times AVERAGE:%lf\n", ITER, total / ITER);
	fprintf(fp, "\n %d times runs.Average cost:%lf", ITER, total / ITER);
	fclose(fp);
	return 0;
}


void deadline_MDP(struct node *nodes, int nodes_length, double time_constraint, int *topo, double *total, struct node *start, struct node *end)
{
	std::vector<struct branch *> branchs;

	//int *contained_nodes = (int *)calloc(nodes_length + 2, sizeof(int));
	//contained_nodes[start->id] = 1;
	//contained_nodes[end->id] = 1;
	//printf("id: %d", (nodes + nodes_length - 1)->succ[0]->id);
	//find_contained_nodes(start, end, contained_nodes, topo);
	//for (int i = 0; i < nodes_length + 2; i++)
	//	printf("%d ", topo[i]);
	//printf("\n");
	//for (int i = 0; i < nodes_length; i++)
	//	printf("%d ", contained_nodes[i]);
	// start end 初始化结束
	//for (int i = 0; i < nodes_length; i++)
	//	printf("%d ", topo[i]);
	differentiate_nodes(nodes, nodes_length);

	//for (int i = 0; i < nodes_length; i++) {
	//	printf("%d ", i + 1);
	//	if (nodes[i].type == Simple_task)
	//		printf("Simple Task\n");
	//	else
	//		printf("Synchronization_task\n");
	//}
	partition_nodes(start, end, &branchs, nodes_length, topo);
	int branchs_length = branchs.size();
	//int *contained_nodes = (int *)calloc(nodes_length + 2, sizeof(int));
	//contained_nodes[(nodes + 5)->id] = 1;
	//contained_nodes[(nodes + 12)->id] = 1;
	//find_contained_nodes(nodes + 5, nodes + 12, contained_nodes);
	//for (i = 0; i < nodes_length + 2; i++)
	//	printf("%d ", contained_nodes[i]);
	//printf("\n");
	deadline_assignment(nodes, nodes_length, topo, start, end, &branchs, time_constraint);
	//for (int i = 0; i < branchs_length; i++) {
	//	if (topo[branchs[i]->head->id] && topo[branchs[i]->tail->id])
	//		printf("branch size : %d %d %lf %lf %lf\n", branchs[i]->size, branchs[i]->start->id + 1, branchs[i]->time, branchs[i]->eet, branchs[i]->dl);
	//}
	//for (i = 0; i < nodes_length; i++) {
	//	if (nodes[i].type == Synchronization_task)
	//		printf("Syn %d eet:%lf %lf\n",i+1, nodes[i].eet, nodes[i].dl);
	//}
	struct solution *s = (struct solution *)malloc(sizeof(struct solution) * (nodes_length + 2));  // +2 是因为start 和end
	planning(nodes, nodes_length, topo, start, end, &branchs, s, time_constraint);
	//calculate_est_and_lst(nodes, nodes_length, *topos);
	double sum_cost = 0;
	for (int i = 0; i < nodes_length; i++) {
		if (topo[nodes[i].id]) {
			//printf("id:%d time:%lf cost:%lf\n", nodes[i].id + 1, s[i].inst->time, s[i].inst->cost);
			sum_cost += s[i].inst->cost;
		}
	}
	printf("sum cost:%lf\n", sum_cost);
	(*total) += sum_cost;
	free(s);
}


void planning(struct node *nodes, int nodes_length, int *topo, struct node *start, struct node *end,
	std::vector<struct branch *> *branchs, struct solution *s, double time_constrain)
{
	std::queue<struct node*> q;
	struct node *operating_node = NULL;
	struct branch *br = NULL;   // represent a branch of the simple tasks;
	int i, j;
	double max_rt;
	int *push_on;    //记录队列状态，节点是否已经进队列
	int *dirty = NULL;
	struct instance *null_inst = (struct instance *)malloc(sizeof(struct instance));
	null_inst->time = 0;
	null_inst->cost = 0;
	dirty = (int *)malloc(sizeof(int)* (nodes_length + 2));
	push_on = (int *)malloc(sizeof(int)* (nodes_length + 2));
	memset(push_on, 0, sizeof(int)* (nodes_length + 2));
	memset(dirty, 0, sizeof(int)* (nodes_length + 2));
	
	q.push(start);
	start->rt = 0;
	s[start->id].st = 0;
	while (!q.empty()) {
		operating_node = q.front();
		//printf("%d\n", operating_node->id);

		q.pop();
		if (operating_node != start) {
			int state = 0;
			// 检查是否所有前驱都已经完成
			for (i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && !dirty[operating_node->pred[i]->id]) {
					q.push(operating_node);
					state = 1;
					break;
				}
			}
			// 如果重新入队则进入下一次循环
			if (state) {
				//std::cout << "hello " << operating_node->id + 1 << std::endl;
				continue;
			}
			max_rt = -1;
			for (i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && (operating_node->pred[i]->rt + s[operating_node->pred[i]->id].inst->time) > max_rt)
					max_rt = (operating_node->pred[i]->rt + s[operating_node->pred[i]->id].inst->time);
			}
			operating_node->rt = max_rt;
		}
		if (operating_node->type == Synchronization_task) {
			if (operating_node->id == start->id || operating_node->id == end->id) {
				s[operating_node->id].inst = null_inst;
			}
			else {
				s[operating_node->id].inst = &(operating_node->instances[synchronization_task_arrange(operating_node)]);
			}
			dirty[operating_node->id] = 1;

			for (i = 0; i < operating_node->num_succ; i++) {
				if (topo[operating_node->succ[i]->id] && !push_on[operating_node->succ[i]->id]) { // have not been pushed in queue 
					q.push(operating_node->succ[i]);
					push_on[operating_node->succ[i]->id] = 1;
				}
			}
		}
		else {

			// find corresponding branch
			for (i = 0; i < (int)branchs->size(); i++) {
				if ((*branchs)[i]->start->id == operating_node->id) {
					br = (*branchs)[i];
					break;	
				}
			}

			int *best_indexs = NULL, *indexs = NULL;
			double min_gap = DBL_MAX;
			best_indexs = (int *)malloc(sizeof(int)* br->size);
			indexs = (int *)malloc(sizeof(int)* br->size);
			get_min_index(indexs, 0, br->start, br->size, &min_gap, best_indexs, br);
			//printf("start:%d %lf %lf %lf %lf %lf", br->start->id, br->head->rt, br->head->eet, br->head->dl, s[br->head->id].inst->time, br->dl);
			//for (i = 0; i < br->size; i++) {
			//	printf("best_index:%d", best_indexs[i]);
			//}
			//printf("\n");
			struct node *cur = br->start;
			for (j = 0; j < br->size; j++, cur = cur->succ[0]){
				dirty[cur->id] = 1;
				s[cur->id].inst = &(cur->instances[best_indexs[j]]);
				//printf("curtime: %lf", s[cur->id].inst->time);
				//printf("%lf\n", cur->pred[0]->rt);
				//printf("cur:%d pred:%d\n", cur->id, cur->pred[0]->id);
				//printf("%lf\n", s[cur->pred[0]->id].inst->time);
				cur->rt = cur->pred[0]->rt + s[cur->pred[0]->id].inst->time;
			}
			cur = NULL;
			free(best_indexs);
			free(indexs);
			if (topo[br->tail->id] && !push_on[br->tail->id]) {
				q.push(br->tail);
				push_on[br->tail->id] = 1;
			}
		}
	}
}

void get_min_index(int *indexs, int k, struct node *head, int size, double *min_gap, int *best_indexs, struct branch *br) 
{
	if (k >= size) {
		double gap = gap_evaluation(indexs, size, head, br);
		if (gap < *min_gap) {
			memcpy(best_indexs, indexs, sizeof(int)* size);
			*min_gap = gap;
		}
		return;
	}
	struct node *tmp = head;
	if (k > 0) {
		int k_bak = k;
		while (k_bak > 0) {
			tmp = tmp->succ[0];
			k_bak--;
		}
	}
	for (int i = 0; i < tmp->num_instances; i++){
		indexs[k] = i;
		get_min_index(indexs, k + 1, head, size, min_gap, best_indexs, br);
	}
}


double gap_evaluation(int *indexs, int size, struct node *head, struct branch *br)
{
	struct node *cur = head;
	double sum_cost = 0, sum_time = 0;
	for (int i = 0; i < size; i++) {
		sum_time += cur->instances[indexs[i]].time;
		sum_cost += cur->instances[indexs[i]].cost;
		cur = cur->succ[0];
	}

	if (sum_time + br->rt <= br->dl) {
		return sum_cost;
	}
	else
		return DBL_MAX;
}


int synchronization_task_arrange(struct node *n)
{
	int i, num_inst = n->num_instances, min_index;
	double min_cost = DBL_MAX;
	for (i = 0; i < num_inst; i++) {
		if (n->instances[i].time + n->rt <= n->dl && n->instances[i].cost < min_cost) {
			min_cost = n->instances[i].cost;
			min_index = i;
		}
			
	}
	return min_index;
}


void deadline_assignment(struct node *nodes, int nodes_length, int *topo, struct node *start, struct node *end, 
	std::vector<struct branch *> *branchs, double time_constrain)
{
	int i, j, min_index;
	std::unordered_map<std::pair<int, int>, double, hashfunc> func_map;
	struct solution *s1 = (struct solution *)malloc(sizeof(struct solution) * (nodes_length + 2)),
		*s2 = (struct solution *)malloc(sizeof(struct solution) * (nodes_length + 2));
	struct instance *null_inst = (struct instance *)malloc(sizeof(struct instance));
	null_inst->time = 0;
	null_inst->cost = 0;
	for (i = 0; i < nodes_length; i++) {
		min_index = 0;
		for (j = 1; j < nodes[i].num_instances; j++) {
			if (nodes[i].instances[j].time < nodes[i].instances[min_index].time)
				min_index = j;
		}
		s1[i].inst = &nodes[i].instances[min_index];
		s2[i].inst = &nodes[i].instances[min_index];
	}
	// 为start 和end分配空的inst
	for (i = nodes_length; i < nodes_length + 2; i++) {
		s1[i].inst = null_inst;
		s2[i].inst = null_inst;
	}

	double min_makespan = calculate_est_and_lst(nodes, nodes_length, topo, &func_map, start, end, s1, s2), critical_time;
	printf("min makespan %lf\n", min_makespan);
	// 无解
	if (min_makespan > time_constrain) {
		fprintf(stderr, "\nmin_makespan > time_constrain! No solution!\n");
		exit(-1);
	}
	//min_makespan = calculate_est_and_lst(nodes, nodes_length, topo, &func_map, nodes + 5, nodes + 12), critical_time;

	//printf("min makespan%lf\n", min_makespan);
	// caculate Synchronization tasks' sub-deadline
	for (i = 0; i < nodes_length; i++) {
		if (topo[nodes[i].id] && nodes[i].type == Synchronization_task) {
			nodes[i].eet = nodes[i].min_time * time_constrain / min_makespan;
			//printf("Synchronization_task:%d %lf %lf %lf %f\n", i + 1, nodes[i].eet, nodes[i].min_time, time_constrain, min_makespan);
		}
	}
	// calculate branchs' sub-deadline
	for (i = 0; i < (int)branchs->size(); i++) {
		if (topo[(*branchs)[i]->head->id] && topo[(*branchs)[i]->tail->id]) {
			if (func_map.count(std::make_pair((*branchs)[i]->head->id, (*branchs)[i]->tail->id)) > 0) {
				critical_time = func_map[std::make_pair((*branchs)[i]->head->id, (*branchs)[i]->tail->id)] -
					(*branchs)[i]->head->min_time - (*branchs)[i]->tail->min_time;          // get critical path time
				(*branchs)[i]->eet = critical_time * time_constrain / min_makespan;
				printf("critical:%lf eet:%lf\n", critical_time, (*branchs)[i]->eet);

			}
			else {
				critical_time = calculate_est_and_lst(nodes, nodes_length, topo, &func_map, (*branchs)[i]->head, (*branchs)[i]->tail, s1, s2) -
					(*branchs)[i]->head->min_time - (*branchs)[i]->tail->min_time;
				(*branchs)[i]->eet = critical_time * time_constrain / min_makespan;
			}
			struct node *cur = (*branchs)[i]->start;
			for (int j = 0; j < (*branchs)[i]->size; j++, cur = cur->succ[0]) {
				cur->eet = cur->min_time * ((*branchs)[i]->eet) / critical_time;
			}
		}
		//printf("branch:head:%d tail:%d eet:%lf \n", (*branchs)[i]->head->id, (*branchs)[i]->tail->id, (*branchs)[i]->eet);
	}
	// 接下来分配rt dl = rt + eet
	rt_dl_arrange(start, end, branchs, nodes_length, nodes, topo);

	free(s1);
	free(s2);
	free(null_inst);
}


void rt_dl_arrange(struct node *start, struct node *end, std::vector<struct branch *> *branchs, int nodes_length, struct node *nodes, int *topo)
{
	std::queue<struct node *> ready_queue;
	struct node *operating_node = NULL;
	int *push_on = NULL, *dirty = NULL;    //记录队列状态，节点是否已经进队列
	dirty = (int *)calloc(nodes_length + 2, sizeof(int));
	push_on = (int *)malloc(sizeof(int)* (nodes_length + 2));
	memset(push_on, 0, sizeof(int)* (nodes_length + 2));

	ready_queue.push(start);

	while (!ready_queue.empty()) {
		operating_node = ready_queue.front();
		ready_queue.pop();
		if (start->id == operating_node->id) {
			operating_node->rt = 0;
		}
		else {
			int state = 0;
			// 检查是否所有前驱都已经完成
			for (int i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && !dirty[operating_node->pred[i]->id]) {
					ready_queue.push(operating_node);
					state = 1;
					break;
				}
			}
			// 如果重新入队则进入下一次循环
			if (state) {
				//std::cout << "hello " << operating_node->id + 1 << std::endl;
				continue;
			}
			double max_pred_dl = -1;
			for (int i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && operating_node->pred[i]->dl > max_pred_dl) {
					max_pred_dl = operating_node->pred[i]->dl;
				}
			}
			operating_node->rt = max_pred_dl;

		}
		operating_node->dl = operating_node->rt + operating_node->eet;
		dirty[operating_node->id] = 1;
		for (int i = 0; i < operating_node->num_succ; i++) {
			if (topo[operating_node->succ[i]->id] && !push_on[operating_node->succ[i]->id]) {
				ready_queue.push(operating_node->succ[i]);
				push_on[operating_node->succ[i]->id] = 1;
			}
		}
	}

	// 给branch赋rt dl
	uint32_t branchs_size = branchs->size();
	for (uint32_t i = 0; i < branchs_size; i++) {
		if (topo[(*branchs)[i]->start->id]) {
			(*branchs)[i]->rt = (*branchs)[i]->start->rt;
			(*branchs)[i]->dl = (*branchs)[i]->tail->rt;
			//printf("dl:%lf\n", (*branchs)[i]->tail->rt);
		}
	}

}


void partition_nodes(struct node *start, struct node *end, std::vector<struct branch *> *branchs, int nodes_length, int* topo)
{
	int i, size;
	struct node *operating_node = NULL, *tmp = NULL;
	struct branch *bran = NULL;
	std::queue<struct node *> q;
	q.push(start);
	int *push_on = (int *)calloc(nodes_length + 2, sizeof(int));
	push_on[start->id] = 1;
	while (!q.empty()) {
		operating_node = q.front();
		q.pop();
		for (i = 0; i < operating_node->num_succ; i++) {
			if (operating_node->succ[i]->type == Simple_task) {
				if (!topo[operating_node->succ[i]->id])
					continue;
				size = 1;
				tmp = operating_node->succ[i];
				bran = (struct branch *)malloc(sizeof(struct branch));
				bran->time = tmp->min_time;
				bran->head = operating_node;
				bran->start = tmp;
				int sig = 0; // 记录是否在路径上
				while (tmp->succ[0]->type == Simple_task) {
					if (!topo[tmp->succ[0]->id]) {
						free(bran);
						sig = 1;
						break;
					}
					size++;
					tmp = tmp->succ[0];
					bran->time += tmp->min_time;
				}
				if (sig)
					continue;
				bran->tail = tmp->succ[0];
				bran->size = size;
				if (bran->tail != end && !push_on[bran->tail->id]) {
					q.push(bran->tail);
					push_on[bran->tail->id] = 1;
				}
				branchs->push_back(bran);
				bran = NULL;
			}
			else {
				if (operating_node->succ[i] != end && !push_on[operating_node->succ[i]->id]) {
					q.push(operating_node->succ[i]);
					push_on[operating_node->succ[i]->id] = 1;
				}
			}
		}
	}
	free(push_on);
}


void differentiate_nodes(struct node *nodes, int nodes_length)
{
	int i;

	for (i = 0; i < nodes_length; i++) {
		int count = 0;
		if (nodes[i].num_pred > 1 || nodes[i].num_succ > 1)
				nodes[i].type = Synchronization_task;
		else
			nodes[i].type = Simple_task;
		//printf("%d %d \n", i, nodes[i].type);
	}
}


void read_data(char *file_name, struct node **nodes_output, int *num_nodes, double *time_constraint, int ***topos, int *num_topos,
	struct node *start, struct node *end, double **prob) 
{
	FILE *fp;
	int num_node, num_edges, i, j, temp, current, next;
	int max_cost_index, min_cost_index, max_time_index, min_time_index;
	double temp1;
	void *new_ptr = NULL;
	struct node *nodes;

	fp = fopen(file_name, "r");
	if (fp == NULL) {
		fprintf(stderr, "make sure the file exist");
		exit(1);
	}
	fscanf(fp, "%d", &num_node);
	fscanf(fp, "%d", &num_edges);
	*num_nodes = num_node;
	nodes = (struct node *)malloc(sizeof(struct node) * num_node);
	memset(nodes, 0, sizeof(struct node) * num_node);
	for (i = 0; i < num_node; i++) {
		nodes[i].id = i;
		nodes[i].succ = (struct node**)malloc(1 * sizeof(struct node*));
		nodes[i].pred = (struct node**)malloc(1 * sizeof(struct node*));
		nodes[i].num_pred = 0;
		nodes[i].num_succ = 0;
		nodes[i].pred_len = 1;
		nodes[i].succ_len = 1;
	}
	for (i = 0; i < num_edges; i++) {
		fscanf(fp, "%d %d %d", &temp, &current, &next);
		// 检查后继是否越界，如果是则realloc内存,为了方便每次扩大两倍 应该能够满足需求
		while (nodes[current - 1].succ_len <= nodes[current - 1].num_succ) {
			nodes[current - 1].succ_len *= 2;
			new_ptr = realloc(nodes[current - 1].succ, sizeof(struct node *) * (nodes[current - 1].succ_len));
			if (!new_ptr) {
				fprintf(stderr, "succ realloc error!");
				exit(-1);
			}
			nodes[current - 1].succ = (struct node **)new_ptr;
		}
		nodes[current - 1].succ[nodes[current - 1].num_succ] = &nodes[next - 1];
		nodes[current - 1].num_succ++;

		// 检查前驱是否越界，如果是则realloc内存,为了方便每次扩大两倍 应该能够满足需求
		while (nodes[next - 1].pred_len <= nodes[next - 1].num_pred) {
			nodes[next - 1].pred_len *= 2;
			new_ptr = realloc(nodes[next - 1].pred, sizeof(struct node *) * nodes[next - 1].pred_len);
			if (!new_ptr) {
				fprintf(stderr, "pred realloc error!");
				exit(-1);
			}
			nodes[next - 1].pred = (struct node **)new_ptr;
		}
		nodes[next - 1].pred[nodes[next - 1].num_pred] = &nodes[current - 1];
		nodes[next - 1].num_pred++;
	}

	for (i = 0; i < num_node; i++) {
		fscanf(fp, "%d %d", &temp, &nodes[i].num_instances);
		nodes[i].instances = (struct instance *) malloc(sizeof(struct instance) * nodes[i].num_instances);
		max_cost_index = 0; min_cost_index = 0; max_time_index = 0; min_time_index = 0;
		
		// 实例instance 读入
		for (j = 0; j < nodes[i].num_instances; j++) {
			fscanf(fp, "%d %lf %lf %lf", &nodes[i].instances[j].id,
				&nodes[i].instances[j].reliability, &nodes[i].instances[j].time, &nodes[i].instances[j].cost);
			if (0 == j)
				continue;     //因为下面四个index默认值为0 所以没必要再去判断
			if (nodes[i].instances[max_cost_index].cost < nodes[i].instances[j].cost)
				max_cost_index = j;
			if (nodes[i].instances[j].cost < nodes[i].instances[min_cost_index].cost)
				min_cost_index = j;
			if (nodes[i].instances[max_time_index].time < nodes[i].instances[j].time)
				max_time_index = j;
			if (nodes[i].instances[j].time < nodes[i].instances[min_time_index].time)
				min_time_index = j;
		}
		// 赋值
		nodes[i].max_time = nodes[i].instances[max_time_index].time;
		nodes[i].min_time = nodes[i].instances[min_time_index].time;
		nodes[i].max_cost = nodes[i].instances[max_cost_index].cost;
		nodes[i].min_cost = nodes[i].instances[min_cost_index].cost;
		printf("%lf %lf %lf %lf \n", nodes[i].max_cost, nodes[i].min_cost, nodes[i].max_time, nodes[i].min_time);
	}
	fscanf(fp, "%lf %lf %lf", &temp1, time_constraint, &temp1);
	fscanf(fp, "%d", num_topos);
	*topos = (int **)malloc(sizeof(int *)* (*num_topos));
	for (i = 0; i < (*num_topos); i++) {
		(*topos)[i] = (int *)calloc(((*num_nodes) + 2), sizeof(int));
		(*topos)[i][(*num_nodes)] = 1;
		(*topos)[i][(*num_nodes) + 1] = 1;
	}
	*prob = (double *)calloc(*num_topos, sizeof(double));
	for (i = 0; i < (*num_topos); i++) {
		for (j = 0; j < (*num_nodes); j++) {
			fscanf(fp, "%d", &(*topos)[i][j]);
		}
		fscanf(fp, "%lf", (*prob) + i);
	}
	*nodes_output = nodes;
	start->type = Synchronization_task;
	end->type = Synchronization_task;
	start->eet = 0;
	end->eet = 0;
	start->id = *num_nodes;
	end->id = *num_nodes + 1;
	start->succ_len = 1;
	start->succ = (struct node**)malloc(sizeof(struct node*) * start->succ_len);
	end->pred_len = 1;
	end->pred = (struct node**)malloc(sizeof(struct node*) * end->pred_len);
	void *tmp = NULL;
	for (int i = 0; i < *num_nodes; i++) {
		if (!nodes[i].num_pred) {
			nodes[i].num_pred = 1;
			nodes[i].pred_len = 1;
			nodes[i].pred = (struct node**)malloc(sizeof(struct node*));
			nodes[i].pred[0] = start;
			start->succ[(start->num_succ)++] = nodes + i;
			while (start->num_succ >= start->succ_len) {
				(start->succ_len) *= 2;
				tmp = realloc(start->succ, sizeof(struct node *) * start->succ_len);
				if (!tmp) {
					fprintf(stderr, "succ realloc error!");
					exit(-1);
				}
				start->succ = (struct node **)tmp;
				tmp = NULL;
			}
		}
		if (!nodes[i].num_succ) {
			nodes[i].num_succ = 1;
			nodes[i].succ_len = 1;
			nodes[i].succ = (struct node**)malloc(sizeof(struct node*));
			nodes[i].succ[0] = end;
			end->pred[(end->num_pred)++] = nodes + i;
			while (end->num_pred >= end->pred_len) {
				(end->pred_len) *= 2;
				tmp = realloc(end->pred, sizeof(struct node *) * end->pred_len);
				if (!tmp) {
					fprintf(stderr, "succ realloc error!");
					exit(-1);
				}
				end->pred = (struct node **)tmp;
				tmp = NULL;
			}
		}
	}
	fclose(fp);
}


void calculate_est(struct node *start, struct node *end, int nodes_length, int *topo, struct solution *s, int *contained_nodes)
{
	std::queue<struct node *> ready_queue;
	int i;
	struct node *operating_node;
	double max_ct = -1, max_finish_time = -1;
	int *push_on = NULL, *dirty = NULL;    //记录队列状态，节点是否已经进队列
	//struct solution *s = (struct solution *)malloc(sizeof(struct solution) * nodes_length);
	push_on = (int *)malloc(sizeof(int)* (nodes_length + 2));
	memset(push_on, 0, sizeof(int)* (nodes_length + 2));
	dirty = (int *)calloc(nodes_length + 2, sizeof(int));
	//for (i = 0; i < nodes_length; i++) {
	//	if (0 == nodes[i].num_pred && topo[i] == 1)
	//		ready_queue.push(nodes[i]);
	//}
	ready_queue.push(start);

	while (!ready_queue.empty()) {
		operating_node = ready_queue.front();
		//printf("%d %d %d\n", start->id, end->id, operating_node->id);

		ready_queue.pop();

		if (start == operating_node) {
			s[operating_node->id].st = 0;
			operating_node->est = 0;
		}
		else {
			int state = 0;
			// 检查是否所有前驱都已经完成
			for (i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && contained_nodes[operating_node->pred[i]->id] && !dirty[operating_node->pred[i]->id]) {
					ready_queue.push(operating_node);
					state = 1;
					break;
				}
			}
			// 如果重新入队则进入下一次循环
			if (state) {
				//std::cout << "hello " << operating_node->id + 1 << std::endl;
				continue;
			}
			max_ct = -1;
			for (i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && contained_nodes[operating_node->pred[i]->id] && max_ct < s[operating_node->pred[i]->id].ct) {
					max_ct = s[operating_node->pred[i]->id].ct;
				}
			}
			s[operating_node->id].st = max_ct;
			operating_node->est = max_ct;
		}

		s[operating_node->id].ct = s[operating_node->id].st + s[operating_node->id].inst->time;
		dirty[operating_node->id] = 1;
		for (i = 0; i < operating_node->num_succ; i++) {
			if (topo[operating_node->succ[i]->id] && contained_nodes[operating_node->succ[i]->id] && !push_on[operating_node->succ[i]->id]) {
				ready_queue.push(operating_node->succ[i]);
				push_on[operating_node->succ[i]->id] = 1;
			}
		}
	}
}


void calculate_lst(struct node *start, struct node *end, int nodes_length, int *topo, struct solution *s, int *contained_nodes)
{
	std::queue<struct node *> ready_queue;
	int i;
	double min_st = DBL_MAX;
	struct node *operating_node;
	int *push_on = NULL, *dirty = NULL;    //记录队列状态，节点是否已经进队列
	//struct solution *s = (struct solution *)malloc(sizeof(struct solution) * nodes_length);
	dirty = (int *)calloc(nodes_length + 2, sizeof(int));
	push_on = (int *)malloc(sizeof(int)* (nodes_length + 2));
	memset(push_on, 0, sizeof(int)* (nodes_length + 2));

	ready_queue.push(end);

	while (!ready_queue.empty()) {
		operating_node = ready_queue.front();
		//printf("%d %d %d\n", start->id, end->id, operating_node->id);

		ready_queue.pop();
		//printf("id:%d\n", operating_node->id);
		if (end == operating_node) {
			s[operating_node->id].ct = operating_node->est + s[operating_node->id].inst->time;
		}
		else {
			//min_index = 0;
			int state = 0;
			// 检查是否所有后继都已经完成
			for (i = 0; i < operating_node->num_succ; i++) {
				if (topo[operating_node->succ[i]->id] && contained_nodes[operating_node->succ[i]->id] && !dirty[operating_node->succ[i]->id]) {
					//printf("succ node done: %d", operating_node->succ[i]->id);
					ready_queue.push(operating_node);
					state = 1;
					break;
				}
			}
			// 如果重新入队则进入下一次循环
			if (state) {
				//std::cout << "hello " << operating_node->id + 1 << std::endl;
				continue;
			}
			min_st = DBL_MAX;
			for (i = 0; i < operating_node->num_succ; i++) {
				if (topo[operating_node->succ[i]->id] && contained_nodes[operating_node->succ[i]->id] && min_st > s[operating_node->succ[i]->id].st) {
					min_st = s[operating_node->succ[i]->id].st;
				}
			}
			s[operating_node->id].ct = min_st;

		}

		s[operating_node->id].st = s[operating_node->id].ct - s[operating_node->id].inst->time;
		operating_node->lst = s[operating_node->id].st;
		dirty[operating_node->id] = 1;
		for (i = 0; i < operating_node->num_pred; i++) {
			if (topo[operating_node->pred[i]->id] && contained_nodes[operating_node->pred[i]->id] && !push_on[operating_node->pred[i]->id]) {
				ready_queue.push(operating_node->pred[i]);
				push_on[operating_node->pred[i]->id] = 1;
			}
		}
	}
}

int find_contained_nodes(struct node *start, struct node *end, int *contained_nodes, int *topo) 
{
	if (start->id == end->id) {
		return 1;
	}
	int res = 0;
	for (int i = 0; i < start->num_succ; i++) {
		if (topo[start->succ[i]->id])
			res += find_contained_nodes(start->succ[i], end, contained_nodes, topo);
	}
	if (res)
		contained_nodes[start->id] = 1;
	return res;
}


double calculate_est_and_lst(struct node *nodes, int nodes_length, int *topo, std::unordered_map<std::pair<int, int>, double, hashfunc> *func_map,
struct node *start, struct node *end, struct solution *s1, struct solution *s2)
{
	int *contained_nodes = (int *)calloc(nodes_length + 2, sizeof(int));
	contained_nodes[start->id] = 1;
	contained_nodes[end->id] = 1;
	find_contained_nodes(start, end, contained_nodes, topo);
	//for (i = 0; i < nodes_length + 2; i++)
	//	printf("%d ", contained_nodes[i]);
	//printf("\n");
	calculate_est(start, end, nodes_length, topo, s1, contained_nodes);
	calculate_lst(start, end, nodes_length, topo, s2, contained_nodes);
	//for (i = 0; i < nodes_length; i++) {
	//	//if (1 == topo[i])
	//		printf("%d est:%lf lst:%lf\n", i + 1, nodes[i].est, nodes[i].lst);
	//}
	//printf("%d est:%lf lst:%lf\n", start->id, start->est, start->lst);
	//printf("%d est:%lf lst:%lf\n", end->id, end->est, end->lst);

	(*func_map)[std::make_pair(start->id, end->id)] = critical_path_makespan(start, end, nodes_length, topo, s1, contained_nodes);
	//printf("the critical time:%lf\n", (*func_map)[std::make_pair(start->id, end->id)]);
	free(contained_nodes);
	//free(s1);
	//free(s2);
	//free(null_inst);
	return (*func_map)[std::make_pair(start->id, end->id)];
}


double critical_path_makespan(struct node *start, struct node *end, int nodes_length, int *topo, struct solution *s, int *contained_nodes)
{
	int i;
	double critical_time = 0;
	struct node * operating_node = NULL;
	std::queue<struct node *> q;
	q.push(start);
	int *push_on = (int *)calloc((nodes_length + 2), sizeof(int)), *dirty = (int *)calloc((nodes_length + 2), sizeof(int));
	push_on[start->id] = 1;
	while (1) {
		operating_node = q.front();
		//printf("start:%d end:%d opera:%d\n", start->id, end->id, operating_node->id);
		q.pop();
		if (operating_node != start) {
			int state = 0;
			// 检查是否所有前驱都已经完成
			for (i = 0; i < operating_node->num_pred; i++) {
				if (topo[operating_node->pred[i]->id] && contained_nodes[operating_node->pred[i]->id] && !dirty[operating_node->pred[i]->id]) {
					q.push(operating_node);
					state = 1;
					break;
				}
			}
			// 如果重新入队则进入下一次循环
			if (state) {
				continue;
			}
		}
		dirty[operating_node->id] = 1;
		if (operating_node->est == operating_node->lst)
			critical_time += s[operating_node->id].inst->time;

		for (i = 0; i < operating_node->num_succ; i++) {	
			if (topo[operating_node->succ[i]->id] && contained_nodes[operating_node->succ[i]->id] && !push_on[operating_node->succ[i]->id]) {
				q.push(operating_node->succ[i]);
				push_on[operating_node->succ[i]->id] = 1;
			}
		}
		if (end == operating_node)
			break;
	}
	free(push_on);
	free(dirty);
	return critical_time;
}