#include <opencv2/opencv.hpp>
#include "dbscan_cuda.cuh"
#include <iostream>
#include <cuda_runtime.h>
#define WHITE_VAL 255
#define SIZE 64

__device__ __managed__ int true_count = 1;

__global__ void num_neighbors(int *count_list, Point *points, int no_of_nodes, float eps){
    int index = blockIdx.x*SIZE + threadIdx.x;
    if(index < no_of_nodes){
        int temp=0;
        for(int i=0;i < no_of_nodes; ++i){
            if(i == index)
                continue;
            if(points[index].euclidean_distance(points[i]) <= eps){
                temp++;
            }
        }
        count_list[index]=temp;
    }
}

__global__ void make_graph(int *adj_list, int *offset, Point *points, int no_of_nodes, int eps){
    int index = blockIdx.x*SIZE + threadIdx.x;

    // //debug
    // if(index == 0){
    //     printf("testing : %d\n", offset[1]);//points[0].euclidean_distance(points[1]));
    // }

    if(index < no_of_nodes){
        int curr_ind = 0;
        for(int i=0;i < no_of_nodes; ++i){
            if(i == index)
                continue;
            if(points[index].euclidean_distance(points[i]) <= eps){
                adj_list[offset[index] + curr_ind] = i;
                curr_ind++;
            }
        }
    }
}

Graph::Graph(cv::Mat &binary_image, float eps) : eps(eps){
//filter black points
    find_nodes(binary_image);
    std::vector<int> neighbor_list = std::vector<int>(nodes.size(), 0);
    int *dev_neighbor_list;

    //allocate nodes on device
    Point *dev_nodes;
    cudaMalloc(&dev_nodes, sizeof(Point)*nodes.size());
    cudaMemcpy(dev_nodes, nodes.data(), sizeof(Point)*nodes.size(), cudaMemcpyHostToDevice);
    // std::cout << (int)nodes[0].x << '\n';
    // std::cout <<"size : " <<  nodes.size() << '\n';
    cudaMalloc(&dev_neighbor_list, sizeof(int)*neighbor_list.size());

    //find neighbors
    dim3 dim_block(SIZE, 1);
    dim3 dim_grid((nodes.size() + SIZE-1)/SIZE, 1);
    num_neighbors<<<dim_grid, dim_block>>>(dev_neighbor_list, dev_nodes, nodes.size(), eps);

    //back to host
    cudaMemcpy(neighbor_list.data(), dev_neighbor_list, sizeof(int)*neighbor_list.size(), cudaMemcpyDeviceToHost);

    // //debug
    // for(int i=0;i < nodes.size(); ++i){
    //     std::cout << neighbor_list[i] << ' ';
    // }
    // std::cout << '\n';

    //allocating memory to adjacency list
    prefix_sum = new int[nodes.size()+1];
    prefix_sum[0] = 0;
    for(int i=1;i < nodes.size()+1; ++i){
        prefix_sum[i] = prefix_sum[i-1] + neighbor_list[i-1];
    }
    // std::cout << "prefix : " << prefix_sum[nodes.size()] << '\n';
    cudaMalloc(&adj_list, sizeof(int)*(prefix_sum[nodes.size()]));
    cudaMalloc(&dev_prefix, sizeof(int)*(nodes.size()+1)); 
    cudaMemcpy(dev_prefix, prefix_sum, sizeof(int)*(nodes.size()+1), cudaMemcpyHostToDevice);
    make_graph<<<dim_grid, dim_block>>>(adj_list, dev_prefix, dev_nodes, nodes.size(), eps);

    // //debug
    // std::cout << "adj list :\n";
    // int adj[prefix_sum[nodes.size()]];
    // cudaMemcpy(adj, adj_list, sizeof(int)*prefix_sum[nodes.size()], cudaMemcpyDeviceToHost);
    // for(int i=0;i < nodes.size(); ++i){
    //     for(int j=prefix_sum[i]; j < prefix_sum[i+1]; ++j){
    //         std::cout << adj[j] << ' ';
    //     }
    //     std::cout << '\n';
    // }

    cudaFree(dev_nodes);
    cudaFree(dev_neighbor_list);

}

void Graph::find_nodes(cv::Mat &img){
    uchar *row;
    cv::MatIterator_<uchar> itr, end;
        for(int i=0; i < img.rows; ++i){
            row = img.ptr<uchar>(i);
            for(int j=0; j < img.cols; ++j){
                if(row[j] == WHITE_VAL){
                    nodes.push_back(Point(i,j));
                }
            }

        }
}

__global__ void search(int *adj_list, int *offset, uchar *frontier, uchar *v, float eps, int min_pts, int no_of_nodes){
    int index = blockIdx.x*SIZE + threadIdx.x;
    if(index < no_of_nodes){
        if(frontier[index]){  //if node is a frontier
            frontier[index] = 0; 
            // v[index]=1; 
            for(int neighbor=offset[index];neighbor < offset[index+1]; ++neighbor){ //set all its neighbors as frontiers
                if(!v[adj_list[neighbor]]){
                    //if border point
                    if(offset[adj_list[neighbor]+1]-offset[adj_list[neighbor]] >= min_pts){
                        frontier[adj_list[neighbor]] = 1;
                    }
                    v[adj_list[neighbor]] = 1;
                }
            }
        }

        //the first thread sums the frontier array
        int sum=0;
        if(index == 0){
            for(int i=0;i < no_of_nodes; ++i){
                sum+=frontier[i];
            }
            true_count = sum;
        }
    }
}


__global__ void reset(int node, uchar *frontier, uchar *v){
    /*
    * kernel code to do bfs
    */
        int index = blockIdx.x*SIZE + threadIdx.x;

        int val=0;
        if(index == node)
            val=1;
        frontier[index] = val;
        v[index] = val;

        true_count = 1;
}

DBSCAN::DBSCAN(Graph *g, float eps, int min_pts) : graph(g), no_nodes(g->nodes.size()), eps(eps), min_pts(min_pts){
    //unified memory
    // cudaMallocManaged(&visited, sizeof(int)*no_nodes);
    // cudaMallocManaged(&labels, sizeof(int)*no_nodes);

    visited = new uchar[no_nodes];
    labels = new uchar[no_nodes];
    // cudaMallocManaged(&true_count, sizeof(int));
}

void DBSCAN::identify_cluster(){
    int cluster_id = 1;
    for(int i=0;i < no_nodes; ++i){
        visited[i] = 0;
        labels[i]=0;
    }        

    // allocating memory
    uchar *frontier; 
    uchar *v; 
    cudaMalloc(&frontier, sizeof(uchar)*no_nodes);
    cudaMalloc(&v, sizeof(uchar)*no_nodes);
    int neighbors;
    for(int node=0;node < no_nodes; ++node){
        neighbors = graph->prefix_sum[node+1] - graph->prefix_sum[node];
        if(!visited[node] && neighbors >= min_pts){
            // std::cout << "n : " << neighbors << '\n';
            // std::cout << "hi";
            visited[node] = 1;
            labels[node] = cluster_id; 
            bfs(frontier, v, node, eps, min_pts, cluster_id++);
        }
    }
    cudaFree(frontier);
    cudaFree(v);

}

void DBSCAN::bfs(uchar *frontier, uchar *v, int node, float eps, int min_pts, int cluster_id){
    /*
    * start from a node and do bfs
    */

    // //debug
    // int adj[graph->prefix_sum[graph->nodes.size()]];
    // cudaMemcpy(adj, graph->adj_list, sizeof(int)*graph->prefix_sum[graph->nodes.size()], cudaMemcpyDeviceToHost);
    // for(int i=0;i < graph->nodes.size(); ++i){
    //     for(int j=graph->prefix_sum[i]; j < graph->prefix_sum[i+1]; ++j){
    //         std::cout << adj[j] << ' ';
    //     }
    //     std::cout << '\n';
    // }


    dim3 dim_block(SIZE, 1);
    dim3 dim_grid((no_nodes + SIZE-1)/SIZE);
    reset<<<dim_grid,dim_block>>>(node, frontier, v);
    cudaDeviceSynchronize();
    // *true_count = 1;

    //debug
    // int counter=1;

    while(true_count){
        search<<<dim_grid, dim_block>>>(graph->adj_list, graph->dev_prefix, frontier, v, eps, min_pts, no_nodes);
        cudaDeviceSynchronize();
        // std::cout << *true_count << '\n';

        //debug
        // if(counter == 10){
        //     break;
        // }
        // counter++;
        
    }
    //back to host
    uchar V[no_nodes];
    cudaMemcpy(V, v, sizeof(uchar)*no_nodes, cudaMemcpyDeviceToHost);        

    for(int node=0;node < no_nodes; ++node){
        if(V[node]){
            labels[node] = cluster_id;
            visited[node] = 1;
        }
    }
}
