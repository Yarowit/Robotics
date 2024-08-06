#define mapSizeInMeters_X 22
#define mapSizeInMeters_Y 10
#define pixelsPerMeter 10
// #define pixelsPerMeter 10

#define maxQueueSize 100

#include <queue> // for priority queue
#include <set>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

bool feasible_position(int x, int y);

const int M_X = mapSizeInMeters_X * pixelsPerMeter;
const int M_Y = mapSizeInMeters_Y * pixelsPerMeter;
bool Map[M_X][M_Y];

struct {
    unsigned int timer;
    int visits;
} VisitedMap[M_X][M_Y];
unsigned int VisitedTimerCooldown = 100;
bool leftTurningMode = false;

int Dij_Map[M_X][M_Y];

const int y_off =  mapSizeInMeters_Y / 2 * pixelsPerMeter;
const int x_off =  1*pixelsPerMeter;

// robot size
const float rsizeInMeters = 0.2;
const float hsize = rsizeInMeters * pixelsPerMeter / 2 ;


std::ofstream myfile;

//debug

// **************************************************************
// **************************************************************
// **************************************************************
// **************************************************************
/*
   Kod do mapowania w webotsach

*/
// **************************************************************
// **************************************************************
// **************************************************************
// **************************************************************
// **************************************************************
// **************************************************************

float h(int i, int j, int x, int y, int X, int Y){
    // x, y - current
    // X, Y - goal
    // add preference to use straight lines
    float mod=0;
    const int dirs_x[] = {-1,1,0,0};
    const int dirs_y[] = {0,0,-1,1};
        // for each direction
    for(int d = 0; d < 4; d++){
        int nx = i + dirs_x[d];
        int ny = j + dirs_y[d];
        if(!feasible_position(nx,ny))
            mod += 3.0;
    }
// std::cout<<"Can go to "<<nx<<" "<<ny<<" \n";


    return (sqrt((X-i)*(X-i) + (Y-j)*(Y-j)) - 1*(sqrt((x-i)*(x-i) + (y-j)*(y-j)))) + mod;
    // return std::abs(X-x) + std::abs(Y-y);
}

struct D_Map {
    short g; // distance to the beggining
    float h; // heuristic function value
    bool legal;
    D_Map(int currentDistance, float heuristic){
        legal = true;
        g = currentDistance;
        h = heuristic;
    }
    D_Map(){
        g=0;h=0;legal=true;
    }
} map[M_X][M_Y];
// struct Coords;

struct Coords {
    int x;
    int y;
    int predacessor_x;
    int predacessor_y;
    float shortestPathLength;
    Coords(int x, int y, int predx, int predy, int shp) : x(x), y(y), predacessor_x(predx),predacessor_y(predy),shortestPathLength(shp) {}
};

bool pqueue_compare(Coords const& c1, Coords const& c2){
    // return "true" if "p1" is ordered
    // before "p2", for example:

    // map needs to be updated when inserting to queue
    struct D_Map one = map[c1.x][c1.y];
    struct D_Map two = map[c2.x][c2.y];
    return 0.2*one.g + one.h > 0.2*two.g + two.h;
}

bool operator<(Coords const& c1, Coords const& c2)
{
    // could be boost::tie
    if(c1.x != c2.x)
        return c1.x < c2.x;
    else
        return c1.y < c2.y;
    // return lhs.y < rhs.y;
}

bool feasible_position(int x, int y){
    // return (Map[x+ x_off][y+y_off] == true);
    if(x < M_X && x >= 0 && y < M_Y && y >= 0 ){
        return !Map[x][y];
    }else{
        return false;
    }
    // if(x+hsize < M_X && x-hsize >= 0 && y+hsize < M_Y && y-hsize >= 0 ){
    //     for(int i = x-hsize; i< x+hsize; i++)
    //         for(int j = y-hsize; j< y+hsize; j++)
    //             if(Map[i+ x_off][j+y_off] == true)
    //                 return false;
    //     return true;
    // }else{
    //     return false;
    // }
}
void preprocessMap(){
    // int border_size = hsize *3.5/2;
    // int border_size = hsize *3/2;
    // int border_size = hsize *3/2;
    int border_size = hsize *4/2;
    bool newMap[M_X][M_Y];
    for(int x=0;x<mapSizeInMeters_X * pixelsPerMeter;x++)
        for(int y=0;y<mapSizeInMeters_Y * pixelsPerMeter;y++){
            bool not_feasible = false;
            if(!(x+border_size < M_X && x-border_size >= 0 && y+border_size < M_Y && y-border_size >= 0 ))
                not_feasible = true;
            else
                for(int i = x-border_size; i< x+border_size; i++){
                    for(int j = y-border_size; j< y+border_size; j++)
                        if(Map[i][j]){
                            not_feasible = true;
                            // break;
                        } 
                }
            // if(not_feasible)
            newMap[x][y] = not_feasible;
        }
    for(int x=0;x<mapSizeInMeters_X * pixelsPerMeter;x++)
        for(int y=0;y<mapSizeInMeters_Y * pixelsPerMeter;y++)
            Map[x][y] = newMap[x][y];
    // printMap();
}
vector<vector<int>> postprocess(vector<vector<int>> v);
vector<vector<int>> dijkstra(int x, int y, int X, int Y){
    int mmn=0;
    myfile<<"Launching Dijkstra"<<std::endl;
    std::cout<<"Launching Dijkstra"<<std::endl;
    // dijstra with a heuristic
    unsigned int pcount = 1;

    // int x,y; // initial position
    // int X,Y; // final position
    // int i,j; // current exploring position

    std::priority_queue<Coords, std::vector<Coords>, decltype(&pqueue_compare)> queue(pqueue_compare);
    // std::priority_queue<Node, std::vector<Node>, decltype(&Compare)> openSet(Compare);
    std::set<Coords> visited;
    // std::set<Coords, decltype(&set_operator)> visited(set_operator);

    queue.push(Coords(x,y,-1,-1,0));
    std::cout<<queue.empty()<<std::endl;
    map[x][y] = D_Map(0,h(x,y,x,y,X,Y));

    while(!queue.empty()){
        // myfile<<queue.size()<<std::endl;

        //delete 200 worst
        // if(queue.size() > 400){
        //     std::queue<Coords> temp;
        //     // std::priority_queue<Coords, std::vector<Coords>, decltype(&pqueue_compare)> newqueue(pqueue_compare);
        //     while(temp.size() < 200){
        //         temp.push(queue.top());
        //         queue.pop();
        //     }
        //     while(queue.size()>0) queue.pop();
        //     while(temp.size() > 0){
        //         queue.push(temp.front());
        //         temp.pop();
        //     }

        // }
        Coords c = queue.top();
        // std::cout<<"checking "<<c.x<<" "<<c.y<<std::endl;
        queue.pop();
        visited.insert(c);

        // std::cout<<(visited.count(c))<<std::endl;
        // std::cout<<(*visited.find(c)).x<<std::endl;
        // std::cout<<(*visited.find(c)).y<<std::endl;
        // found the goal
        // myfile<<c.x<<" "<<c.y<<"- > "<<X<<" "<<Y<<std::endl;
        if(c.x == X && c.y == Y){
            break;
        }
        
        // for each neighbour
        // north
        const int dirs_x[] = {-1,1,0,0};
        const int dirs_y[] = {0,0,-1,1};
        // for each direction
        for(int d = 0; d < 4; d++){
            int nx = c.x + dirs_x[d];
            int ny = c.y + dirs_y[d];
            if(feasible_position(nx,ny)){
// std::cout<<"Can go to "<<nx<<" "<<ny<<" \n";
                float mod = 0;
                for(int dd = 0; dd < 4; dd++){
                    if(!feasible_position(nx + dirs_x[dd],ny+dirs_y[dd]))
                        mod += 0.1;
                }
                Coords n(nx, ny,c.x,c.y, floor(c.shortestPathLength) + 1 + mod);
                // Coords n(nx, ny,c.x,c.y, c.shortestPathLength + 1);
    
                auto found = visited.find(n);

                if(found == visited.end()){
                    // update map
                    map[nx][ny] = D_Map(n.shortestPathLength,h(n.x,n.y,x,y,X,Y));
                    queue.push(n);
// std::cout<<"A new one, Push\n";
                }else{
                    // there was a better solution already
                    if(false)
                    if((*found).shortestPathLength > n.shortestPathLength){
                        // update map
                        map[nx][ny] = D_Map(n.shortestPathLength,h(n.x,n.y,x,y,X,Y));

                        Coords newFound = *found;
                        newFound.predacessor_x = n.predacessor_x;
                        newFound.predacessor_y = n.predacessor_y;
                        visited.erase(found);
                        visited.insert(newFound);
                    }
                }
            }
        }

        //################# DEBUG ######################
        const int ff = 100;
        if((visited.size()%ff == 0 && visited.size()/ff == pcount) || mmn==1131){
            pcount++;
            for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
                for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
                    if(Map[i][j])
                        Dij_Map[i][j] = 1;
                    else
                        Dij_Map[i][j] = 0;
                }
            //mark visited
            auto up = [](Coords element) { Dij_Map[element.x][element.y] = 3; }; 
            // auto up = [&Dij_Map](Coords element) { Dij_Map[element.x][element.y] = 3; }; 
        
            // Use std::for_each to iterate over the set and apply 
            // the lambda function to each element 
            for_each(visited.begin(), visited.end(), up); 
            // myfile<<"\n\nVisited:"<<visited.size()<<"\n\n";
            myfile<<"\n\n***************************************************\n\n";
            for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
                for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
                    if(i == x && j == y) myfile<<"SS";
                    else if(i == X && j == Y) myfile<<"EE";
                    else switch(Dij_Map[i][j]){
                    // else switch(Dij_Map[i][mapSizeInMeters_Y * pixelsPerMeter-1-j]){
                        case 0:
                            myfile<<"  ";
                            break;
                        case 1:
                            myfile<<"##";
                            break;
                        case 2:
                            myfile<<"++";
                            break;
                        case 3:
                            myfile<<"--";
                            break;
                        default:
                            break;
                    }
                myfile<<std::endl;
            }
            myfile<<std::endl;
            myfile<<"\n\n***************************************************\n\n";
        }
        // myfile<<mmn++<<std::endl;
        // std::cout<<"Q: "<<queue.size()<<std::endl;
    }

    // now backtrack
    // std::cout<<"Backtrack:\n";
    /*
    int pred_x = X, pred_y = Y;
    while(pred_x != -1){
        std::cout<<pred_x<<" "<<pred_y<<std::endl;
        Coords cmp(pred_x,pred_y,0,0,0);
        cmp = *visited.find(cmp);
        pred_x = cmp.predacessor_x;
        pred_y = cmp.predacessor_y;
    }
    */
    myfile<<"DIj done"<<std::endl;
    // printing for debug
    // int Dij_Map[M_X][M_Y];
    for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
        for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
            if(Map[i][j])
                Dij_Map[i][j] = 1;
            else
                Dij_Map[i][j] = 0;
        }
    //mark visited
    auto up = [](Coords element) { Dij_Map[element.x][element.y] = 3; }; 
    // auto up = [&Dij_Map](Coords element) { Dij_Map[element.x][element.y] = 3; }; 
  
    // Use std::for_each to iterate over the set and apply 
    // the lambda function to each element 
    for_each(visited.begin(), visited.end(), up); 
    myfile<<"lambdafi"<<std::endl;
    
    int pred_x = X, pred_y = Y;
    vector<vector<int>> result;
    
    myfile<<"up to -1"<<std::endl;
    
    while(pred_x != -1){
        // Dij_Map[pred_y-y_off+x_off][pred_x-x_off+y_off] = 2;
        Dij_Map[pred_x][pred_y] = 2;
        
        Coords cmp(pred_x,pred_y,0,0,0);
        cmp = *visited.find(cmp);
        // myfile<<pred_x<<" "<<pred_y<<" "<<cmp.shortestPathLength<<std::endl;
        pred_x = cmp.predacessor_x;
        pred_y = cmp.predacessor_y;
        result.push_back({pred_x,pred_y});
    }
    myfile<<"reverse"<<std::endl;
    result.pop_back();
    reverse(result.begin(),result.end());
    myfile<<"post"<<std::endl;
    result = postprocess(result);

    myfile<<"\n\n************RESULT********************\n\n";
    for(auto el : result){
        // myfile<<el[0]<<" "<<el[1]<<" ";
        // myfile<<(float)(el[0]-x_off)/pixelsPerMeter<<" "<<(float)(el[1]-y_off)/pixelsPerMeter<<endl;
        Dij_Map[el[0]][el[1]] = 4;
    }
    myfile<<"\n\n***************************************************\n\n";
    for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
        for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
            if(i == x && j == y) myfile<<"SS";
            else if(i == X && j == Y) myfile<<"EE";
            else switch(Dij_Map[i][j]){
            // else switch(Dij_Map[i][mapSizeInMeters_Y * pixelsPerMeter-1-j]){
                case 0:
                    myfile<<"  ";
                    break;
                case 1:
                    myfile<<"##";
                    break;
                case 2:
                    myfile<<"++";
                    break;
                case 3:
                    myfile<<"--";
                    break;
                case 4:
                    myfile<<"><";
                    break;
                default:
                    break;
            }
        myfile<<std::endl;
    }
    myfile<<std::endl;
    myfile<<"\n\n***************************************************\n\n";

    return result;
}


vector<vector<int>> postprocess(vector<vector<int>> v){
    vector<vector<int>> res;
    
    unsigned int checkLength = 100;

    unsigned int i=1;
    while(i < v.size()-1){
        unsigned int j;
        

        for(j = min(i + checkLength, (unsigned int)v.size()-1); j > i; j--){
            bool notDoable = false;
            // float length = sqrt((v[i][0]-v[j][0])*(v[i][0]-v[j][0]) + (v[i][1]-v[j][1])*(v[i][1]-v[j][1]));
            for(double k = 0.001; k < 1; k+= 0.001){
            // for(double k = 0.001; k < 1; k+= 0.001){
                // int a;
                // if((v[j][0]-v[i][0]) * k - floor((v[j][0]-v[i][0]) * k) > 0.5)
                //     a = v[i][0] + (v[j][0]-v[i][0]) * k;
                // else
                //     a = v[i][0] + (v[j][0]-v[i][0]) * k - 1;

                // int b;
                // if((v[j][1]-v[i][1]) * k - floor((v[j][1]-v[i][1]) * k) > 0.5)
                //     b = v[i][1] + (v[j][1]-v[i][1]) * k;
                // else
                //     b = v[i][1] + (v[j][1]-v[i][1]) * k - 1;
                int a = v[i][0] + round((v[j][0]-v[i][0]) * k);
                int b = v[i][1] + round((v[j][1]-v[i][1]) * k);
                // int 
                
                // myfile<<a<<" "<<b<<" "<<Map[a][b]<<endl;
                
                if(Map[a][b]){
                    notDoable = true;
                    break;
                }
            }
            if(!notDoable) break;
        }
        res.push_back({v[j][0], v[j][1]});
        // myfile<<v[j][0]<<" ******** "<<v[j][1]<<endl;

        i = j;
    }

    return res;
}
