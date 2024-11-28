#include <opencv2/opencv.hpp>
#include <iostream>
#include <cuda_runtime.h>
#define WHITE_VAL 255
#define SIZE 64


struct Point{
    Point() = default;
    Point(int i, int j): x(i), y(j){}
    int x, y;

    __device__ __host__ float euclidean_distance(Point &other){
            return sqrt((float)((this->x - other.x)*(this->x - other.x) + (this->y - other.y)*(this->y - other.y)));
    }
    __device__ __host__ int manhattan_distance(Point &other){
            return abs(this->x - other.x) + abs(this->y - other.y);
    }


};

__global__ void num_neighbors(int *count_list, Point *points, int no_of_nodes, float eps);

__global__ void make_graph(int *adj_list, int *offset, Point *points, int no_of_nodes, int eps);

class Graph{
    public:
        Graph() = default;
        ~Graph(){
            cudaFree(adj_list);
            cudaFree(dev_prefix);
            delete []prefix_sum;
        }
        Graph(cv::Mat &binary_image, float eps);
        size_t size(){
            return nodes.size();
        }

        Point node(int index){
            return nodes[index];
        }
    private:
        std::vector<Point> nodes;
        int *adj_list; 
        int *dev_prefix;
        int *prefix_sum;
        float eps;

        void find_nodes(cv::Mat &img);
        friend class DBSCAN;
};

__global__ void search(int *adj_list, int *offset, uchar *frontier, uchar *v, float eps, int min_pts, int no_of_nodes, int *true_count);

__global__ void reset(int node, uchar *frontier, uchar *v);

class DBSCAN{
    public:
        DBSCAN(Graph *g, float eps, int min_pts);
        ~DBSCAN(){
            delete []visited;
            delete []labels;
            // cudaFree/(true_count);
        }

        void identify_cluster();
        
        void show_labels(){
            std::cout << "labels :\n";
            for(int i=0;i < no_nodes; ++i){
                std::cout << (int)labels[i] << ' ';
            }
            std::cout << '\n';
        }

        uchar label(int index){
            return labels[index];
        }
    private:
        Graph *graph;
        int no_nodes;
        uchar *visited;
        uchar *labels;
        float eps;
        int min_pts;
        // int *true_count;

        void bfs(uchar *frontier, uchar *v, int node, float eps, int min_pts, int cluster_id);

};

void gpu_dbscan(cv::Mat img);