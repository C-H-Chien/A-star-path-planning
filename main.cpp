#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cv.h>
#include <time.h>
#include <highgui.h>
#include <math.h> 
#include <string> 
#include <sstream>
using namespace std; 
using namespace cv;


#define PEN 2

#define img_M 960
#define img_N 480

#define M  16  
#define N  8 
//map marks  
#define PI 3.14156
#define  AVAIL     255  
#define  UNAVAIL   0  
#define  START   100  
#define  END     111  
#define  ROAD     10  
#define GET_F(X)  (X->G + X->H ) 
//#define GET_F(X)  (X->G + X->H + X->W) 


typedef struct Node  
{  
    //for node itself  
    int type; //node type  
    int i; //i index  
    int j; //j index  
    //for A star algorithm  
    double G; //past road cost  
    double H; //heuristic, F = G + H  
	double W;
    struct Node* parent; //parent node, used for trace road  
    struct Node* next; //only used for open and close list  
}Node; 

void draw_circle(cv::Mat img1,int x,int y,int pen,int R,int G,int B);
void draw_line(cv::Mat img1,int sx,int sy,int ex,int ey,int pen,int R,int G,int B);
void init_openlist(); 
void init_closelist();  
void destroy_openlist();
void insert_into_openlist(Node* new_node);
void insert_into_closelist(Node* new_node);
Node* find_node_in_list_by_ij(Node* node_list, int di, int dj);
Node* pop_firstnode_from_openlist();
void remove_node_from_openlist(Node* nd);
void remove_node_from_closelist(Node* nd);
double calc_W(int cur_i, int cur_j);
double calc_H(int cur_i, int cur_j, int end_i, int end_j);
double calc_G(Node* cur_node);
void init_start_node(Node* st, int si, int sj, int ei, int ej);
void init_end_node(Node* ed, int ei, int ej);
void init_pass_node(Node* pd, int pi, int pj);

int check_neighbor(int map[][N], int width, int height,   
    int di, int dj, Node* parent_node, Node* end_node);

void extend_node(Node* cd, int map[][N], int width, int height, Node* end_node);

Node* a_star_search(int map[M][N], int width, int height,   
                    int start_i, int start_j, int end_i, int end_j);




int mappp[M][N];
cv::Mat img  =cv::imread("1.png",1);

int one=0;
char imgname[200];
  
//==========================open close list operation================  

Node* open_list;  
Node* close_list;  
void init_openlist()  
{  
    open_list = NULL;  
}  
void init_closelist()  
{  
    close_list = NULL;  
}  
void destroy_openlist()  
{  
    Node* q;  
    Node* p = open_list;  
    while (p != NULL)  
    {  
        q = p->next;  
        free(p);  
        p = q;  
    }  
}  
void destroy_closelist()  
{  
    Node* q;  
    Node* p = close_list;  
    while (p != NULL)  
    {  
        q = p->next;  
        free(p);  
        p = q;  
    }  
}  
void insert_into_openlist(Node* new_node) //insert and sort by F  
{  
    Node* p;  
    Node* q;  
    if (open_list == NULL)  
    {  
        open_list = new_node; //insert as the first  
        return;  
    }  
    p = open_list; 
	if (GET_F(new_node) < GET_F(p))  
    {  
		open_list = new_node;
		open_list->next = p; 

        return;  
    } 
    while (p != NULL)  
    {  
        q = p;  
        p = p->next;  
        if (p == NULL)  
        {  
            q->next = new_node; //insert as the last  
            return;  
        }  
        else if (GET_F(new_node) < GET_F(p))  
        {  
            q->next = new_node; //insert before p, sorted  
            new_node->next = p;  
            return;  
        }  
    }  
      
}  
void insert_into_closelist(Node* new_node) //just insert before head  
{  
    if (close_list == NULL)  
    {  
        close_list = new_node; //insert as the first  
        return;  
    }  
    else  
    {  
        new_node->next = close_list; //insert before head  
        close_list = new_node;  
        return;  
    }  
}  
Node* find_node_in_list_by_ij(Node* node_list, int di, int dj)  
{  
    Node* p = node_list;  
    while (p)  
    {  
        if (p->i == di && p->j == dj)  
            return p;  
        p = p->next;  
    }  
    return NULL;  
}  
Node* pop_firstnode_from_openlist() //get the minimum node sorted by F  
{  
    Node* p = open_list;  
    if (p == NULL)  
    {  
        return NULL;  
    }  
    else  
    {  
        open_list = p->next;  
        p->next = NULL;  
        return p;  
    }  
}  
void remove_node_from_openlist(Node* nd) //just remove it, do not destroy it  
{  
    Node* q;  
    Node* p = open_list;  
    if (open_list == nd)  
    {  
        open_list = open_list->next;  
        return;  
    }  
    while (p)  
    {  
        q = p;  
        p = p->next;  
        if (p == nd) //found  
        {  
            q->next = p->next;  
            p->next = NULL;  
            return;  
        }  
    }  
}  
void remove_node_from_closelist(Node* nd) //just remove it, do not destroy it  
{  
    Node* q;  
    Node* p = close_list;  
    if (close_list == nd)  
    {  
        close_list = close_list->next;  
        return;  
    }  
    while (p)  
    {  
        q = p;  
        p = p->next;  
        if (p == nd) //found  
        {  
            q->next = p->next;  
            p->next = NULL;  
            return;  
        }  
    }  
}  
//======================= H ¡B G start=============================  
double calc_W(int cur_i, int cur_j)  
{  
	int i,j;
	int TH=50;
	int w=0;
	int reg_i,reg_j;
	int len;

	/*reg_i = cur_i - TH/2;
	reg_j = cur_j - TH/2;
	for(j=0;j < TH;j++)
	{
		for(i=0;i < TH;i++)
		{
			if(mappp[reg_i+i][reg_j+j] == 0 )
			{
				w=1;
				break;
			}
		}
		if(w==1) break;
	}*/

	int k;
	int temp=0;
	for(k=1;k < (TH/2);k++)
	{
		temp=2*k+1;
		reg_i = cur_i - k;
		reg_j = cur_j - k;
		for(j=0;j < temp;j++)
		{
			for(i=0;i < temp;i++)
			{
				if(mappp[reg_i+i][reg_j+j] == 0 )
				{
					w=1;
					break;
				}
			}
			if(w==1) break;
		}
		if(w==1) break;
	}

	
	len =TH - (abs(reg_j+j - cur_j) + abs(reg_i+i - cur_i));
	if(w==0) len=0;


    return len*8;  
	//return 0;
} 

double calc_H(int cur_i, int cur_j, int end_i, int end_j)  
{  
    return (abs(end_j - cur_j) + abs(end_i - cur_i)) * 10.0; //the heuristic  
}  
double calc_G(Node* cur_node)  
{  
    Node* p = cur_node->parent;  
    if (abs(p->i - cur_node->i) + abs(p->j - cur_node->j) > 1)  
        return 14.0 + p->G; //the diagonal cost is 14  
    else  
        return 10.0 + p->G; //the adjacent cost is 10  
} 
//======================= H ¡B G end===============================
void init_start_node(Node* st, int si, int sj, int ei, int ej)  
{  
    memset(st, 0, sizeof(Node));  
    st->type = START;  
    st->i = si;  
    st->j = sj;  
    st->H = calc_H(si, sj, ei, ej);  
	st->W = calc_W(si,sj);
    st->G = 0;  
}  
void init_end_node(Node* ed, int ei, int ej)  
{  
    memset(ed, 0, sizeof(Node));  
    ed->type = END;  
    ed->i = ei;  
    ed->j = ej;  
    ed->H = 0;
	ed->W = 0;
    ed->G = 9999; //temp value  
}  
void init_pass_node(Node* pd, int pi, int pj)  
{  
    memset(pd, 0, sizeof(Node));  
    pd->type = AVAIL;  
    pd->i = pi;  
    pd->j = pj;  
}  
//check the candidate node (i,j) when extending parent_node  
int check_neighbor(int map[][N], int width, int height,   
    int di, int dj, Node* parent_node, Node* end_node)  
{  
    Node* p;  
    Node* temp;  
    double new_G;  
    if (di < 0 || dj < 0 || di > height-1 || dj > width-1)  
        return UNAVAIL;  
    //1. check available  
    if (map[di][dj] == UNAVAIL)  
        return UNAVAIL;  
    //2. check if existed in close list  
    p = find_node_in_list_by_ij(close_list, di, dj);   
    if (p != NULL)  
    {  
        //found in the closed list, check if the new G is better, added 2012-05-09  
        temp = p->parent;  
        p->parent = parent_node;  
        new_G = calc_G(p);  
        if (new_G >= p->G)  
        {  
            p->parent = temp; //if new_G is worse, recover the parent  
        }  
        else  
        {  
            //the new_G is better, remove it from close list, insert it into open list  
            p->G = new_G;  
            remove_node_from_closelist(p); //remove it  
            insert_into_openlist(p); //insert it, sorted  
        }  
        return AVAIL;  
    } 
    //3. check if existed in open list  
    p = find_node_in_list_by_ij(open_list, di, dj); //in open list  
    if (p != NULL)  
    {  
        //found in the open list, check if the new G is better  
        temp = p->parent;  
        p->parent = parent_node;  
        new_G = calc_G(p);  
        if (new_G >= p->G)  
        {  
            p->parent = temp; //if new_G is worse, recover the parent  
        }  
        else  
        {  
            //the new_G is better, resort the list  
            p->G = new_G;  
            remove_node_from_openlist(p); //remove it  
            insert_into_openlist(p); //insert it, sorted  
        }  
        return AVAIL;  
    }  
      
    //4. none of above, insert a new node into open list  
    if (map[di][dj] == END)  
    {  
        //4~. check if it is end node  
        end_node->parent = parent_node;  
        end_node->G = calc_G(end_node);  
        insert_into_openlist(end_node); //insert into openlist  
        return AVAIL;  
    }  
    else  
    {  
        //4~~. create a new node  
        p = (Node*)malloc(sizeof(Node));  
        init_pass_node(p, di, dj);  
        p->parent = parent_node;  
        p->H = calc_H(di, dj, end_node->i, end_node->j);
		//p->W = calc_W(di,dj);
        p->G = calc_G(p);  
        insert_into_openlist(p); //insert into openlist  
        return AVAIL;  
    }  
}  
//extend the current node on the map  (reimplemented when porting a star to another application) 
void extend_node(Node* cd, int map[][N], int width, int height, Node* end_node)  
{  
    int up_status, down_status, left_status, right_status;  
    int ci, cj; //cur node i, j  
    int ti, tj; //temp i, j  
    ci = cd->i;  
    cj = cd->j;  
    //1. up  
    ti = ci - 1;  
    tj = cj;  
    up_status = check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //2. down  
    ti = ci + 1;  
    tj = cj;  
    down_status = check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //3. left  
    ti = ci;  
    tj = cj - 1;  
    left_status = check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //4. right  
    ti = ci;  
    tj = cj + 1;  
    right_status = check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //5. leftup  
    ti = ci - 1;  
    tj = cj - 1;  
    if (up_status == AVAIL && left_status == AVAIL)  
        check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //6. rightup  
    ti = ci - 1;  
    tj = cj + 1;  
    if (up_status == AVAIL && right_status == AVAIL)  
        check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //7. leftdown  
    ti = ci + 1;  
    tj = cj - 1;  
    if (down_status == AVAIL && left_status == AVAIL)  
        check_neighbor(map, width, height, ti, tj, cd, end_node);  
    //8. rightdown  
    ti = ci + 1;  
    tj = cj + 1;  
    if (down_status == AVAIL && right_status == AVAIL)  
        check_neighbor(map, width, height, ti, tj, cd, end_node);  
      
}    
//search a road on a map, return node_list  
Node* a_star_search(int map[M][N], int width, int height,   
                    int start_i, int start_j, int end_i, int end_j)  
{  
	for(int i=0;i<N;i++)
		for(int j=0;j<M;j++)
		{
			mappp[j][i]=map[j][i];
		}
    Node* cur_node;  
    Node* start_node;  
    Node* end_node;  
    //create start and end node  
    start_node = (Node*)malloc(sizeof(Node));  
    init_start_node(start_node, start_i, start_j, end_i, end_j);  
    end_node = (Node*)malloc(sizeof(Node));  
    init_end_node(end_node, end_i, end_j);  
      
    //init open and close list  
    init_openlist();  
    init_closelist();  
    //put start_node into open list  
    insert_into_openlist(start_node);  
      
    //start searching  
    while (1)  
    {  
        cur_node = pop_firstnode_from_openlist(); //it has the minimum F value  
        if (cur_node == NULL || cur_node->type == END)  
        {  
            break; //found the road or no road found  
        }  
          
        extend_node(cur_node, map, width, height, end_node); //the key step!!  
        insert_into_closelist(cur_node);  
    }  
    //you can track the road by the node->parent  
    return cur_node;  
}  



int main()
{
	int start_i = 3;
	int start_j = 4;
	int end_i   = 13;
	int end_j   = 4;
	cv::Point PointArray[4];
	cv::namedWindow("a",1);

	draw_line(img,  0,  0,959,  0,PEN,0,0,0);
	draw_line(img,  0,  0,  0,479,PEN,0,0,0);
	draw_line(img,  0,479,959,479,PEN,0,0,0);
	draw_line(img,959,  0,959,479,PEN,0,0,0);
	for(int i=0;i < 8;i++)
	{
		draw_line(img,   0,  i*60,960,i*60,PEN/2,0,0,0);
	}
	for(int i=0;i < 8;i++)
	{
		draw_line(img,i*60,  0,i*60,480,PEN/2,0,0,0);
	}
	for(int i=0;i < 8;i++)
	{
		draw_line(img,960-i*60,  0,960-i*60,500,PEN/2,0,0,0);
	}
	draw_line(img,8*60,  0,8*60,500,PEN/2,0,0,0);

	PointArray[0]=Point(start_i*60+1,start_j*60+1);
	PointArray[1]=Point(start_i*60+59,start_j*60+1);
	PointArray[2]=Point(start_i*60+59,start_j*60+59);
	PointArray[3]=Point(start_i*60+1,start_j*60+59);
	cv::fillConvexPoly(img,PointArray,4,CV_RGB(0,255,0),CV_AA,0);

	PointArray[0]=Point(end_i*60+1,end_j*60+1);
	PointArray[1]=Point(end_i*60+59,end_j*60+1);
	PointArray[2]=Point(end_i*60+59,end_j*60+59);
	PointArray[3]=Point(end_i*60+1,end_j*60+59);
	cv::fillConvexPoly(img,PointArray,4,CV_RGB(255,0,0),CV_AA,0);

	PointArray[0]=Point(7*60+1,3*60+1);
	PointArray[1]=Point(7*60+119,3*60+1);
	PointArray[2]=Point(7*60+119,3*60+119);
	PointArray[3]=Point(7*60+1,3*60+119);
	cv::fillConvexPoly(img,PointArray,4,CV_RGB(0,0,0),CV_AA,0);

	putText(img,"S",cv::Point(    start_i*60+20,start_j*60+40),FONT_HERSHEY_TRIPLEX  ,1,cv::Scalar(0,0,0));

	putText(img,"E",cv::Point(end_i*60+20,end_j*60+40),FONT_HERSHEY_TRIPLEX  ,1,cv::Scalar(0,0,0));

	strcpy(imgname, "");
	sprintf(imgname,"image/%d.png",one);

	imwrite (imgname, img);
	one=one+1;
	cv::imshow("a",img);


	cv::waitKey(0);




	for(int i=0;i<N;i++)
	{
		for(int j=0;j<M;j++)
		{
			if( j>=7 && j<=8 && i>=3 && i<=4)
			{
				mappp[j][i]=0;
			}else
			{
				mappp[j][i]=255;
			}
			//printf("%d",mappp[j][i]);
		}
		//printf("\n");
	}
	mappp[start_i][start_j]=START;
	mappp[end_i][end_j]=END;


    Node* cur_node;  
    Node* start_node;  
    Node* end_node;  
    //create start and end node  
    start_node = (Node*)malloc(sizeof(Node));  
    init_start_node(start_node, start_i, start_j, end_i, end_j);  
    end_node = (Node*)malloc(sizeof(Node));  
    init_end_node(end_node, end_i, end_j);  
      
    //init open and close list  
    init_openlist();  
    init_closelist();  
    //put start_node into open list  
    insert_into_openlist(start_node);  
      
    //start searching 
	Node* p;
	Node* q;
	std::ostringstream strs;
	std::string str;

	double angle_rad;
	double angle;
	double x;
	double y;


    while (1)  
    {  
		img  =cv::imread("1.png",1);
		draw_line(img,  0,  0,959,  0,PEN,0,0,0);
		draw_line(img,  0,  0,  0,479,PEN,0,0,0);
		draw_line(img,  0,479,959,479,PEN,0,0,0);
		draw_line(img,959,  0,959,479,PEN,0,0,0);
		for(int i=0;i < 8;i++)
		{
			draw_line(img,   0,  i*60,960,i*60,PEN/2,0,0,0);
		}
		for(int i=0;i < 8;i++)
		{
			draw_line(img,i*60,  0,i*60,480,PEN/2,0,0,0);
		}
		for(int i=0;i < 8;i++)
		{
			draw_line(img,960-i*60,  0,960-i*60,500,PEN/2,0,0,0);
		}
		draw_line(img,8*60,  0,8*60,500,PEN/2,0,0,0);

		PointArray[0]=Point(start_i*60+1,start_j*60+1);
		PointArray[1]=Point(start_i*60+59,start_j*60+1);
		PointArray[2]=Point(start_i*60+59,start_j*60+59);
		PointArray[3]=Point(start_i*60+1,start_j*60+59);
		cv::fillConvexPoly(img,PointArray,4,CV_RGB(0,255,0),CV_AA,0);

		PointArray[0]=Point(end_i*60+1,end_j*60+1);
		PointArray[1]=Point(end_i*60+59,end_j*60+1);
		PointArray[2]=Point(end_i*60+59,end_j*60+59);
		PointArray[3]=Point(end_i*60+1,end_j*60+59);
		cv::fillConvexPoly(img,PointArray,4,CV_RGB(255,0,0),CV_AA,0);

		PointArray[0]=Point(7*60+1,3*60+1);
		PointArray[1]=Point(7*60+119,3*60+1);
		PointArray[2]=Point(7*60+119,3*60+119);
		PointArray[3]=Point(7*60+1,3*60+119);
		cv::fillConvexPoly(img,PointArray,4,CV_RGB(0,0,0),CV_AA,0);

		putText(img,"S",cv::Point(    start_i*60+20,start_j*60+40),FONT_HERSHEY_TRIPLEX  ,1,cv::Scalar(0,0,0));

		putText(img,"E",cv::Point(end_i*60+20,end_j*60+40),FONT_HERSHEY_TRIPLEX  ,1,cv::Scalar(0,0,0));

		cv::imshow("a",img);
		cv::waitKey(100);


        cur_node = pop_firstnode_from_openlist(); //it has the minimum F value  
        if (cur_node == NULL || cur_node->type == END)  
        {  
            break; //found the road or no road found  
        }  
          
        extend_node(cur_node, mappp, N, M, end_node); //the key step!!  
        insert_into_closelist(cur_node); 


		p = open_list;
		while(p)
		{
			strs.str("");
			strs.clear();
			strs << p->G;
			str = strs.str();
			putText(img,"G:"+str,cv::Point(p->i*60+10,p->j*60+30),FONT_HERSHEY_TRIPLEX  ,0.5,cv::Scalar(0,0,0));

			strs.str("");
			strs.clear();
			strs << p->H;
			str = strs.str();
			putText(img,"H:"+str,cv::Point(p->i*60+10,p->j*60+50),FONT_HERSHEY_TRIPLEX  ,0.5,cv::Scalar(0,0,0));

			strs.str("");
			strs.clear();
			strs << (p->G + p->H);
			str = strs.str();
			putText(img,"F:"+str,cv::Point(p->i*60+10,p->j*60+10),FONT_HERSHEY_TRIPLEX  ,0.5,cv::Scalar(0,0,0));

			circle(img,cv::Point(p->i*60+30,p->j*60+30),PEN/2,cv::Scalar(255,0,0),5,CV_AA,0);
			q=p->parent;

			x=(q->i - p->i);
			y=(q->j - p->j);
			angle_rad =-atan2(y,x);
			angle     =angle_rad*180/PI;
			
			draw_line(img, p->i*60+30, p->j*60+30, p->i*60+30+30*cos(angle_rad), p->j*60+30-30*sin(angle_rad), PEN/2,0,0,255);


			p=p->next;
		}

		p = close_list;
		while(p)
		{
			if(p->type != START )
			{
				
				PointArray[0]=Point( p->i*60+1,   p->j*60+1);
				PointArray[1]=Point(p->i*60+59,   p->j*60+1);
				PointArray[2]=Point(p->i*60+59,p->j*60+59);
				PointArray[3]=Point( p->i*60+1,p->j*60+59);


				cv::fillConvexPoly(img,PointArray,4,CV_RGB(238,234,47),CV_AA,0);


				strs.str("");
				strs.clear();
				strs << p->G;
				str = strs.str();
				putText(img,"G:"+str,cv::Point(p->i*60+10,p->j*60+30),FONT_HERSHEY_TRIPLEX  ,0.5,cv::Scalar(0,0,0));

				strs.str("");
				strs.clear();
				strs << p->H;
				str = strs.str();
				putText(img,"H:"+str,cv::Point(p->i*60+10,p->j*60+50),FONT_HERSHEY_TRIPLEX  ,0.5,cv::Scalar(0,0,0));

				strs.str("");
				strs.clear();
				strs << (p->G + p->H);
				str = strs.str();
				putText(img,"F:"+str,cv::Point(p->i*60+10,p->j*60+10),FONT_HERSHEY_TRIPLEX  ,0.5,cv::Scalar(0,0,0));

				circle(img,cv::Point(p->i*60+30,p->j*60+30),PEN/2,cv::Scalar(0,255,0),5,CV_AA,0);
				q=p->parent;
				x=(q->i - p->i);
				y=(q->j - p->j);
				angle_rad =-atan2(y,x);
				//angle     =angle_rad*180/PI;
			
				draw_line(img, p->i*60+30, p->j*60+30, p->i*60+30+30*cos(angle_rad), p->j*60+30-30*sin(angle_rad), PEN/2,0,255,0);

			}
			p=p->next;
		}
		
		strcpy(imgname, "");
		sprintf(imgname,"image/%d.png",one);

		imwrite (imgname, img);

		cv::imshow("a",img);
		cv::waitKey(50);
		one=one+1;
		system("pause");
    }  


	//imwrite ("output1.png ", img);
	
	return 0;
}



void draw_line(cv::Mat img1,int sx,int sy,int ex,int ey,int pen,int R,int G,int B)
{
	line(img1,cv::Point(sx,sy),cv::Point(ex,ey),cv::Scalar(B,G,R),pen,CV_AA,0);
}

void draw_circle(cv::Mat img1,int x,int y,int pen,int R,int G,int B)
{
	circle(img1,cv::Point(x,y),pen,cv::Scalar(B,G,R),1,CV_AA,0);
}