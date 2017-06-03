/*
Team Id:           eYRC-BV#721
Author List:       Ankit Gupta, Karamveer, Prashant Singh
Fiename:           shortest_path.c
Theme:             Bothoven
Functions:         dijkstra()
Global Variables   bool,visited,n
*/
#include<stdlib.h>
#include<stdio.h>
#include "matrix.c" //file having the graph of arena
#define MAX 50
#define infinite 9999

typedef enum boolean{false,true} bool;
bool visited[MAX];
int n=48;    /* Denotes number of nodes in the graph */
int path[30];
/*
Function Name:dijkstra()
Input:starting node and lastnode for which shortest path is to be found
Output:an address to an array having the nodes of shortest path linked to each other
Logic:this function uses the dijkstra shortest path algorithm to find the shortest path
Example call:dijkstra(3,17);
*/

void dijkstra(int startnode,int lastnode)
{

 //static  int path[30];

    int distance[MAX],pred[MAX];
    int visited[MAX],count,mindistance,nextnode=0,i,j,k=0;
    startnode=startnode-1;
    lastnode=lastnode-1;

    //pred[] stores the predecessor of each node
    //count gives the number of nodes seen so far
    //create the cost matrix
   

    //initialize pred[],distance[] and visited[]
    for(i=0;i<n;i++)
    {
	distance[i]=cost[startnode][i];
	pred[i]=startnode;
	visited[i]=0;
    }

    distance[startnode]=0;
    visited[startnode]=1;
    count=1;

    while(count<n-1)
    {
	mindistance=infinite;

	//nextnode gives the node at minimum distance
	for(i=0;i<n;i++)
	    if(distance[i]<mindistance&&!visited[i])
	    {
		mindistance=distance[i];
		nextnode=i;
	    }

	    //check if a better path exists through nextnode
	    visited[nextnode]=1;
	    for(i=0;i<n;i++)
		if(!visited[i])
		    if(mindistance+cost[nextnode][i]<distance[i])
		    {
			distance[i]=mindistance+cost[nextnode][i];
			pred[i]=nextnode;
		    }
	count++;
    }

	     for(i=0;i < n;i++)
		if(i==lastnode)
		{
		//	printf("\nDistance of %d = %d", i, distance[i]);
		
		//	printf("\nPath = %d", i+1);
			k=0;
			path[k]=i+1;
			j=i;
			do
			{
				j=pred[j];
		//		printf(" <-%d", j+1);
				k=k+1;
				path[k]=j+1;
			}
			while(j!=startnode);
		}
		
		
		
    
   
    
 /*  j = steps - 1;   // j will Point to last Element
   i = 0;       // i will be pointing to first element
 
   while (i < j) {
      temp = path[i];
      path[i] = path[j];
      path[j] = temp;
      i++;             // increment i
      j--;          // decrement j
   }
   */
 /* lcd_init();
	lcd_print(1,1,path[0],2);
	lcd_print(1,4,path[1],2);
	lcd_print(1,7,path[2],2);
	lcd_print(1,10,path[3],2);
	lcd_print(1,12,path[4],2);
	lcd_print(2,1,path[5],2);
	lcd_print(2,4,path[6],2);
	lcd_print(2,7,path[7],2);
	lcd_print(2,10,path[8],2);
	lcd_print(2,12,path[9],2);*/

	
	
	   	
}/*
int main ()
{
    int start=1, last=13,i;
     
    dijkstra(start,last);
     
     for(i=0;i<30;i++)
     {     printf(" ");

	 printf("%d",path[i]);
	 }

     return 0;
}*/

