//把场地外涂成黑色
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <findball/field.h>
#include <findball/obstacle.h>

typedef enum {Green=0,Black,Else,Edge,Checked,Denied,Unknown} PicColor;
typedef enum {Line=0,Point,Divided,Unfound} Dotfeature;
template<class T>
class Node{
private:
	Node<T>*next;
public:
	T data;
	T address;
	Node(const T &data,const T &address,Node<T>*next=0);
	void insertAfter(Node<T>*p);
	Node<T>*deleteAfter();
	Node<T>*nextNode();
	const Node<T>*nextNode() const;
};
template<class T>
Node<T>::Node(const T&data,const T &address,Node<T>*next):data(data),address(address),next(next){}
template<class T>
Node<T>*Node<T>::nextNode(){
	return next;
}
template<class T>
const Node<T>*Node<T>::nextNode() const{
	return next;
}
template<class T>
void Node<T>::insertAfter(Node<T>*p){
	p->next=next;
	next=p;
}
template<class T>
Node<T>*Node<T>::deleteAfter(){
	Node<T>*tempPtr=next;
	if(next==0)
		return 0;
	next=tempPtr->next;
	return tempPtr;
}
template<class T>
class Queue{
private:
	Node<T>*front,*rear;
	Node<T>*prevPtr,*currPtr;
	int  size;
	Node<T>*newNode(const T &item,const T &address,Node<T>*ptrNext);
	void freeNode(Node<T>*p);
	void copy(const Queue<T>&L);
public:	
	Queue();
	Queue(const Queue<T>&L);
	~Queue(){};
	Queue<T>&operator=(const Queue<T>&L);
	bool isEmpty() const;
	void clear();
	void insertRear(const T&item,const T &address);
	T deleteFront();
	T backAddress();
	T maxHigh();
	T minHigh();
	int backSize();
};
template<class T>
Node<T>* Queue<T>::newNode(const T &item,const T &address,Node<T>*ptrNext){
	Node<T>* newnode;
	newnode=new Node<T>(item,address,ptrNext);
	return newnode;
}
template<class T>
void Queue<T>::freeNode(Node<T>*p){
	delete p;
}
template<class T>
void Queue<T>::copy(const Queue<T>&L){
	Node<T>*p;
	p=L.front;
	while(p!=NULL){
		insertRear(p->data);
		p=p->nextNode();
	}
}
template<class T>
Queue<T>::Queue():front(NULL),rear(NULL),prevPtr(NULL),currPtr(NULL),size(0){}
template<class T>
Queue<T>::Queue(const Queue<T>&L){
	copy(L);
}
template<class T>
Queue<T>& Queue<T>::operator=(const Queue<T>&L){
	if(this!=L){
		clear();
		copy(L);
	}
	return *this;
}
template<class T>
bool Queue<T>::isEmpty() const{
	if(size==0)
		return true;
	else return false;
}
template<class T>
void Queue<T>::clear(){
	Node<T>*p,*q;
	p=front;
	while(p!=NULL){
		q=p->nextNode();
		freeNode(p);
		p=q;
	}
	front=NULL;
	rear=NULL;
	prevPtr=NULL;
	currPtr=NULL;
	size=0;
}
template<class T>
void Queue<T>::insertRear(const T&item,const T &address){
	Node<T> *p,*nextf=NULL;
	prevPtr=rear;
	if(rear==NULL){
		p=newNode(item,address,front);
		front=p;
		rear=front;
	}
	else{
		p=newNode(item,address,nextf);
		rear->insertAfter(p);
		rear=p;
	}
	currPtr=rear;
	size++;
}
template<class T>
T Queue<T>::deleteFront(){
	Node<T>*p;
	T item;
	item=front->data;
	p=front;
	if(front==rear)rear=NULL;
	front=front->nextNode();
	freeNode(p);
	size--;
	return item;
}
template<class T>
T Queue<T>::backAddress(){
	return front->address;
}
template<class T>
T Queue<T>::maxHigh(){
	int max=0;	
	Node<T> *p=front;
	while(p!=NULL){
		if(p->address>max)max=p->address;
		p=p->nextNode();
	}
	return max;
}
template<class T>
T Queue<T>::minHigh(){
	int min=front->address;	
	Node<T> *p=front;
	while(p!=NULL){
		if(p->address<min)min=p->address;
		p=p->nextNode();
	}
	return min;
}
template<class T>
int Queue<T>::backSize(){
	return size;
}

class ImageChanger{
private:
ros::NodeHandle nh;
image_transport::Publisher pub;
image_transport::Publisher rawpub;
image_transport::Publisher fieldpub;
image_transport::Publisher bvpub;
image_transport::Subscriber sub;
ros::Publisher line_pub;
ros::Publisher obstacle_pub;
image_transport::ImageTransport it;
public:
ImageChanger():it(nh){
	sub=it.subscribe("image_color_classified",1,&ImageChanger::imageCb,this,image_transport::TransportHints());	
	line_pub = nh.advertise<findball::field>("vision_field",10);
	obstacle_pub = nh.advertise<findball::obstacle>("vision_obsitacle",10);	
	pub=it.advertise("field_img", 1);
	rawpub=it.advertise("raw_field_img", 1);
	fieldpub=it.advertise("line_img",1);
	bvpub=it.advertise("bv_img",1);
}
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{//ROS_INFO("sub");
	int c,i,j,k,count,bcount,m,n;
	/*for(i=0;i<msg->width;i++){
	for(j=msg->height-1;j>=0;j--){	
	ROS_INFO("%d",msg->data[j*msg->width+i]);}}*/
	findball::field boundary;
	findball::obstacle area;
	boundary.width=msg->width;
//分片，每10＊10个像素分为一块，在该块的100个像素中有超过20的像素点为绿色，即把该块标记为绿色，否则标记为白色。
	int pix=10;
	int* edge=new int[msg->width/pix];
	int* slope=new int[msg->width];
	sensor_msgs::Image msg3;
	msg3.width=msg->width/pix;
  	msg3.height=msg->height/pix;
  	msg3.encoding="mono8";
  	msg3.step=msg->step/pix;//ROS_INFO("%d,%d,%d",msg3.width,msg3.height,msg3.step);
	msg3.data.resize(msg3.width*msg3.height);
	for(i=0;i<msg->width;i=i+pix){
	  for(j=0;j<msg->height;j=j+pix){
		  for(count=0,bcount=0,m=i;m<i+pix;m++){
			for(n=j;n<j+pix;n++){
			   if(msg->data[n*msg->width+m]==4*32)count++;
			   else if(msg->data[n*msg->width+m]==0)bcount++;			
			}
		  }
		  if(count>20){
			   msg3.data[(j/pix)*msg3.width+(i/pix)]=4*32;
		  }
		  else if(bcount>20){
			   msg3.data[(j/pix)*msg3.width+(i/pix)]=0;
		  }
		  else { 
			   msg3.data[(j/pix)*msg3.width+(i/pix)]=255;
		  }
	  }	
	}
//分片图像简单去噪
	msg3.data.resize(msg3.width*msg3.height);
	for(i=1;i<(msg3.width-1);i++){
	   for(count=0,j=1;j<(msg3.height-1);j++){
	      if(msg3.data[j*msg3.width+i]==4*32){
		if(msg3.data[(j+1)*msg3.width+i]==4*32)count++;
		if(msg3.data[j*msg3.width+(i+1)]==4*32)count++;
		if(msg3.data[(j-1)*msg3.width+i]==4*32)count++;
		if(msg3.data[j*msg3.width+(i-1)]==4*32)count++;
	      	if(count<2)msg3.data[j*msg3.width+i]=255;
	      }
	      count=0;
	   }
	}
//生成分片后的图像bv_msg
	sensor_msgs::Image msg0;
	msg0.width=msg3.width;
  	msg0.height=msg3.height;
  	msg0.encoding="mono8";
  	msg0.step=msg3.step;
	msg0.data.resize(msg3.width*msg3.height);
	for(i=0;i<msg3.width;i++){	
	   for(j=0;j<msg3.height;j++){	
		msg0.data[j*msg3.width+i]=msg3.data[j*msg3.width+i];
		}	
	}
//分片图像去噪，关键在于找到对应每个横坐标的最高绿色像素的纵坐标,采用宽搜
	//ROS_INFO("@******");
	Queue<int> queue;
	int **pic;
	pic=new int *[msg3.height];
	for(i=0;i<msg3.height;i++)pic[i]=new int [msg3.width];
	for(i=0;i<msg3.height;i++){
		for(j=0;j<msg3.width;j++){
			pic[i][j]=Unknown;
		}
	} 	
	for(i=0;i<msg3.width;i++){
		edge[i]=msg3.height;
	}
	for(i=0;i<msg3.width;){
		if(msg3.data[(msg3.height-1)*msg3.width+i]!=4*32){
			i++;
			pic[msg3.height-1][i]=Else;
		}
		else{
			if(pic[msg3.height-1][i]==Unknown){	
				queue.insertRear(i,msg3.height-1);
				pic[msg3.height-1][i]=Green;
				edge[i]=msg3.height-1;
				do{
					m=queue.backAddress();
					n=queue.deleteFront();
					if(n<msg3.width-1&&n>0){
						if(m>0){
							if(pic[m-1][n]==Unknown){
								if(msg3.data[(m-1)*msg3.width+n]==4*32){
									queue.insertRear(n,m-1);
									pic[m-1][n]=Green;
								}
								else {pic[m-1][n]=Else;}
							}
							if(pic[m][n+1]==Unknown){
								if(msg3.data[m*msg3.width+(n+1)]==4*32){
									queue.insertRear(n+1,m);
									pic[m][n+1]=Green;
								}
								else {pic[m][n+1]=Else;}
							}
							if(pic[m][n-1]==Unknown){
								if(msg3.data[(m)*msg3.width+(n-1)]==4*32){
									queue.insertRear(n-1,m);
									pic[m][n-1]=Green;
								}
								else {pic[m][n-1]=Else;}
							}
						}
						else {
							if(pic[m][n+1]==Unknown){
								if(msg3.data[m*msg3.width+(n+1)]==4*32){
									queue.insertRear(n+1,m);
									pic[m][n+1]=Green;
								}
								else {pic[m][n+1]=Else;}
							}
							if(pic[m][n-1]==Unknown){
								if(msg3.data[(m)*msg3.width+(n-1)]==4*32){
									queue.insertRear(n-1,m);
									pic[m][n-1]=Green;
								}
								else {pic[m][n-1]=Else;}
							}
						}
					}
					else if(n==msg3.width){
						if(m>0){
							if(pic[m-1][n]==Unknown){
								if(msg3.data[(m-1)*msg3.width+n]==4*32){
									queue.insertRear(n,m-1);
									pic[m-1][n]=Green;
								}
								else {pic[m-1][n]=Else;}
							}
							if(pic[m][n-1]==Unknown){
								if(msg3.data[(m)*msg3.width+(n-1)]==4*32){
									queue.insertRear(n-1,m);
									pic[m][n-1]=Green;
								}
								else {pic[m][n-1]=Else;}
							}
						}
						else{
							if(pic[m][n-1]==Unknown){
								if(msg3.data[(m)*msg3.width+(n-1)]==4*32){
									queue.insertRear(n-1,m);
									pic[m][n-1]=Green;
								}
								else {pic[m][n-1]=Else;}
							}
						}
					}
					else if(n==0){
						if(m>0){
							if(pic[m-1][n]==Unknown){
								if(msg3.data[(m-1)*msg3.width+n]==4*32){
									queue.insertRear(n,m-1);
									pic[m-1][n]=Green;
								}
								else {pic[m-1][n]=Else;}
							}
							if(pic[m][n+1]==Unknown){
								if(msg3.data[m*msg3.width+(n+1)]==4*32){
									queue.insertRear(n+1,m);
									pic[m][n+1]=Green;
								}
								else {pic[m][n+1]=Else;}
							}
						}
						else{
							if(pic[m][n+1]==Unknown){
								if(msg3.data[m*msg3.width+(n+1)]==4*32){
									queue.insertRear(n+1,m);
									pic[m][n+1]=Green;
								}
								else {pic[m][n+1]=Else;}
							}
						}
					}
					else {}
				}while(!queue.isEmpty());	
			}
			else {i++;}
		}
	}
	queue.clear();
	for(i=0;i<msg3.height;i++){
		for(j=0;j<msg3.width;j++){
			if(pic[i][j]==Green&&edge[j]>i)edge[j]=i;
		}
	} 
	for(i=0;i<msg3.width;i++){
		if(edge[i]==msg3.height)edge[i]=msg3.height-1;
	}	
	//ROS_INFO("#******");
	//ROS_INFO("%d//",msg3.height);
	//for(i=0;i<msg3.width;i++)ROS_INFO("%d",edge[i]);

//因为障碍物的存在，为防止线性拟合时出现越界现象，增加的算法，但有时会导致边界线识别出现错误
	int* bvline=new int[msg3.width];
	int lebo,ribo,yob;
	for(i=0;i<msg3.width;i++){
		bvline[i]=Unfound;
	}
	for(i=1;i<msg3.width;i++){
		if((edge[i]-edge[i-1])>2||(edge[i]-edge[i-1])<-2){
			bvline[i]=Divided;//ROS_INFO("bvi=%d",i);
		}
	}
	for(i=1;i<msg3.width;i++){
		if(bvline[i]==Divided){
		   for(j=i+1;j<msg3.width;j++){
			if(j-i>6){//j=i+1;
	//for(m=i;m<j;m++)edge[m]=edge[i-1]+(edge[j]-edge[i-1])*(m-i+1)/(j-i+1);j=i+6;
			i=j;
			j=msg3.width;
			}
			else if(bvline[j]==Divided){
			//lebo=i;ribo=j;yob=edge[i];
	for(m=i;m<j;m++)edge[m]=edge[i-1]+(edge[j]-edge[i-1])*(m-i+1)/(j-i+1);
			i=j;
			j=msg3.width;//ROS_INFO("bvi");
			}
			/*else if(j==msg3.width-1){
	for(m=i;m<j;m++)edge[m]=edge[i-1]+(edge[j]-edge[i-1])*(m-i+1)/(j-i+1);
			i=msg3.width;
			j=msg3.width;
			}*/
		   }
		}
	}
//对找到的宽边界进行细化，找到原图像的绿色边界，精确到像素
	boundary.data.resize(msg->width);	
	for(i=0;i<msg->width;i=i+pix){
	    	  for(m=i;m<i+pix;m++){
			for(n=pix*edge[i/pix];n<pix*edge[i/pix]+pix;n++){
			   if(edge[i/pix]==msg3.height-1){
				boundary.data[m]=msg->height-1;
				break;
			   }
			   else if(msg->data[n*msg->width+m]==4*32){
				boundary.data[m]=n;
				break;
			   }
			   else boundary.data[m]=pix*edge[i/pix]+pix;
			}
		  }
	}
//在颜色分割后的图中用白色把场地边界标记出
	sensor_msgs::Image msg2;
	msg2.width=msg->width;
  	msg2.height=msg->height;
  	msg2.encoding="mono8";
  	msg2.step=msg->step;
	msg2.data.resize(msg->width*msg->height);
	for(i=0;i<msg->width;i++){	
	   for(j=0;j<msg->height;j++){	
		if(j==boundary.data[i])c=255;
		else c=msg->data[j*msg->width+i];
		msg2.data[j*msg->width+i]=c;
		}	
	}
//用最小二乘法拟合边界线，拟合为数条直线，采用机器学习的方法确定折线关节点
//要处理的图像有两类，一类是左下角顶点和右下角顶点均在场地内，还有一种是存在不属于绿色场地内的左下角顶点或右下角顶点，对第二种情况进行标记后处理
	int Lbottom,Rbottom;
	for(i=0,m=0;i<msg->width;i++){
		if(boundary.data[i]==msg->height-1){boundary.data[i]=msg->height-1;m++;}
		else break;
	}
	for(i=msg->width-1,n=0;i>0;i--){
		if(boundary.data[i]==msg->height-1){boundary.data[i]=msg->height-1;n++;}
		else break;
	}
	if(m+n>msg->width){
		m=0;
		n=0;
	}
	Lbottom=m;
	Rbottom=msg->width-1-n;
	ROS_INFO("LeftBottom=%d,RightBottom=%d",Lbottom,Rbottom);
//采用先划分后合并的拟合方法
	double Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0,stage=msg->width/10,X,Y,r,a,b;
	double* x=new double[msg->width];
	double* y=new double[msg->width];
	int* dotline=new int[msg->width];
	for(i=0;i<msg->width;i++){
	    	x[i]=double(i);
		y[i]=double(boundary.data[i]);
		dotline[i]=Unfound;
	}	
	for(j=0;j<msg->width;j=j+stage){	
	  	for(i=j,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+stage;i++){
	    		Sx=Sx+x[i];
	    		Sx2=Sx2+x[i]*x[i];
	    		Sy=Sy+y[i];
	    		Sy2=Sy2+y[i]*y[i];
	    		Sxy=Sxy+x[i]*y[i];
	 	 }
	  	X=Sx/stage;Y=Sy/stage;
	  	r=(Sxy-stage*X*Y)/sqrt((Sx2-stage*X*X)*(Sy2-stage*Y*Y));
	  	if(r>0.8||r<-0.8){
	    		b=(Sxy-stage*X*Y)/(Sx2-stage*X*X);
	    		a=Y-b*X;
	    		for(i=j;i<j+stage;i++){
	      			boundary.data[int(x[i])]=int(b*x[i]+a);
				if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
				else if(boundary.data[i]<0)boundary.data[i]=0;
				dotline[i]=Line;
	    		}
	  	}
	  	else {
			for(i=j,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+stage/2;i++){
	     			Sx=Sx+x[i];
	     			Sx2=Sx2+x[i]*x[i];
	     			Sy=Sy+y[i];
	     			Sy2=Sy2+y[i]*y[i];
	     			Sxy=Sxy+x[i]*y[i];
	  		}
	   		X=2*Sx/stage;Y=2*Sy/stage;
	   		r=(Sxy-(stage/2)*X*Y)/sqrt((Sx2-(stage/2)*X*X)*(Sy2-(stage/2)*Y*Y));
	   		if(r>0.8||r<-0.8){
	    			b=(Sxy-(stage/2)*X*Y)/(Sx2-(stage/2)*X*X);
	     			a=Y-b*X;
	     			for(i=j;i<j+stage/2;i++){
	       				boundary.data[int(x[i])]=int(b*x[i]+a);
					if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
					else if(boundary.data[i]<0)boundary.data[i]=0;
					dotline[i]=Line;
	     			}
	   		}else{
				for(i=j,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+stage/4;i++){
	     				Sx=Sx+x[i];
	     				Sx2=Sx2+x[i]*x[i];
	     				Sy=Sy+y[i];
	     				Sy2=Sy2+y[i]*y[i];
	     				Sxy=Sxy+x[i]*y[i];
	  			}
	   			X=4*Sx/stage;Y=4*Sy/stage;
	    			b=(Sxy-(stage/4)*X*Y)/(Sx2-(stage/4)*X*X);
	     			a=Y-b*X;
				r=(Sxy-(stage/4)*X*Y)/sqrt((Sx2-(stage/4)*X*X)*(Sy2-(stage/4)*Y*Y));
	     			for(i=j;i<j+stage/4;i++){
	       				boundary.data[int(x[i])]=int(b*x[i]+a);
					if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
					else if(boundary.data[i]<0)boundary.data[i]=0;
					if(r>0.8||r<-0.8)dotline[i]=Line;
					else dotline[i]=Point;
	     			}
				for(i=j+stage/4,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+stage/2;i++){
	     				Sx=Sx+x[i];
	     				Sx2=Sx2+x[i]*x[i];
	     				Sy=Sy+y[i];
	     				Sy2=Sy2+y[i]*y[i];
	     				Sxy=Sxy+x[i]*y[i];
	  			}
	   			X=4*Sx/stage;Y=4*Sy/stage;
	    			b=(Sxy-(stage/4)*X*Y)/(Sx2-(stage/4)*X*X);
	     			a=Y-b*X;
				r=(Sxy-(stage/4)*X*Y)/sqrt((Sx2-(stage/4)*X*X)*(Sy2-(stage/4)*Y*Y));
	     			for(i=j+stage/4;i<j+stage/2;i++){
	       				boundary.data[int(x[i])]=int(b*x[i]+a);
					if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
					else if(boundary.data[i]<0)boundary.data[i]=0;
					if(r>0.8||r<-0.8)dotline[i]=Line;
					else dotline[i]=Point;
	     			}
			}
	   		for(i=j+stage/2,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+stage;i++){
	     			Sx=Sx+x[i];
	     			Sx2=Sx2+x[i]*x[i];
	     			Sy=Sy+y[i];
	     			Sy2=Sy2+y[i]*y[i];
	     			Sxy=Sxy+x[i]*y[i];
	   		}
	   		X=2*Sx/stage;Y=2*Sy/stage;
	   		r=(Sxy-(stage/2)*X*Y)/sqrt((Sx2-(stage/2)*X*X)*(Sy2-(stage/2)*Y*Y));
	   		if(r>0.8||r<-0.8){
	     			b=(Sxy-(stage/2)*X*Y)/(Sx2-(stage/2)*X*X);
	     			a=Y-b*X;
	     			for(i=j+stage/2;i<j+stage;i++){
	       				boundary.data[int(x[i])]=int(b*x[i]+a);
					if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
					else if(boundary.data[i]<0)boundary.data[i]=0;
					dotline[i]=Line;
	     			}
	   		}else{
				for(i=j+stage/2,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+3*stage/4;i++){
	     				Sx=Sx+x[i];
	     				Sx2=Sx2+x[i]*x[i];
	     				Sy=Sy+y[i];
	     				Sy2=Sy2+y[i]*y[i];
	     				Sxy=Sxy+x[i]*y[i];
	  			}
	   			X=4*Sx/stage;Y=4*Sy/stage;
	    			b=(Sxy-(stage/4)*X*Y)/(Sx2-(stage/4)*X*X);
	     			a=Y-b*X;
				r=(Sxy-(stage/4)*X*Y)/sqrt((Sx2-(stage/4)*X*X)*(Sy2-(stage/4)*Y*Y));
	     			for(i=j+stage/2;i<j+3*stage/4;i++){
	       				boundary.data[int(x[i])]=int(b*x[i]+a);
					if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
					else if(boundary.data[i]<0)boundary.data[i]=0;
					if(r>0.8||r<-0.8)dotline[i]=Line;
					else dotline[i]=Point;
	     			}
				for(i=j+3*stage/4,Sx=0.0,Sy=0.0,Sx2=0.0,Sy2=0.0,Sxy=0.0;i<j+stage;i++){
	     				Sx=Sx+x[i];
	     				Sx2=Sx2+x[i]*x[i];
	     				Sy=Sy+y[i];
	     				Sy2=Sy2+y[i]*y[i];
	     				Sxy=Sxy+x[i]*y[i];
	  			}
	   			X=4*Sx/stage;Y=4*Sy/stage;
	    			b=(Sxy-(stage/4)*X*Y)/(Sx2-(stage/4)*X*X);
	     			a=Y-b*X;
				r=(Sxy-(stage/4)*X*Y)/sqrt((Sx2-(stage/4)*X*X)*(Sy2-(stage/4)*Y*Y));
	     			for(i=j+3*stage/4;i<j+stage;i++){
	       				boundary.data[int(x[i])]=int(b*x[i]+a);
					if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
					else if(boundary.data[i]<0)boundary.data[i]=0;
					if(r>0.8||r<-0.8)dotline[i]=Line;
					else dotline[i]=Point;
	     			}
			}
	  	}
	}
	for(i=0;i<msg->width;i++){
	   if(i==0){
		if(dotline[i]==Point||dotline[i]==Unfound){
		  for(j=0;j<msg->width;j++){
			if(dotline[j]==Line){
				for(k=0;k<j;k++){
				boundary.data[k]=boundary.data[0]+(boundary.data[j]-boundary.data[0])*k/j;
				if(boundary.data[k]>=msg->height)boundary.data[k]=msg->height-1;
				else if(boundary.data[k]<0)boundary.data[k]=0;
					dotline[k]=Line;
				}
				j=msg->width;
			}
			else {
			    if(j==msg->width){
				for(k=0;k<msg->width;k++){
				boundary.data[k]=boundary.data[0]+(boundary.data[msg->width-1]-boundary.data[0])*k/(msg->width-1);
				if(boundary.data[k]>=msg->height)boundary.data[k]=msg->height-1;
				else if(boundary.data[k]<0)boundary.data[k]=0;
					dotline[k]=Line;
				}
			    }
			}
		  }
	       }
	  }//for(j=0;j<msg->width;j++)ROS_INFO("%d",boundary.data[j]);
	  else{
		if(dotline[i]==Point||dotline[i]==Unfound){
			for(j=i;j<msg->width;j++){
				if(dotline[j]==Line){
					for(k=i-1;k<j;k++){
				boundary.data[k]=boundary.data[i-1]+(boundary.data[j]-boundary.data[i-1])*(k-i+1)/(j-i+1);
				if(boundary.data[k]>=msg->height)boundary.data[k]=msg->height-1;
				else if(boundary.data[k]<0)boundary.data[k]=0;
					dotline[k]=Line;
					}
				j=msg->width;
				}
				/*else {
				    if(j==msg->width-1){
					for(k=i-1;k<j;k++){
				boundary.data[k]=boundary.data[i-1]+(boundary.data[j]-boundary.data[i-1])*(k-i+1)/(j-i+1);
			if(boundary.data[k]>=msg->height)boundary.data[k]=msg->height-1;
			else if(boundary.data[k]<0)boundary.data[k]=0;
					dotline[k]=Line;
					}
				    }
				}*/
			}
		}
	  }
	}
	/*for(i=0;i<msg->width;i++){
		if(boundary.data[i]>=msg->height)boundary.data[i]=msg->height-1;
		else if(boundary.data[i]<0)boundary.data[i]=0;
	}*/
//边界线凸包处理
	int* Ylocate=new int[msg->width];	
	int* Xlocate=new int[msg->width];
	int len=0;
	slope[0]=0;
	for(i=0,j=1;j<msg->width;i++,j++){
		slope[j]=boundary.data[j]-boundary.data[i];
		if(slope[j]!=slope[i])dotline[i]=Divided;
	}
	//for(i=0;i<msg->width;i++)ROS_INFO("%d",slope[i]);
	dotline[0]=Divided;
	dotline[msg->width-1]=Divided;	
	/*for(i=0;i<msg->width;i++){
		if(i%8==0)dotline[i]=Divided;
	}*/
	for(i=0;i<msg->width;i++){
		if(dotline[i]==Divided){
			Xlocate[len]=i;//ROS_INFO("i=%d",i);
			Ylocate[len]=boundary.data[i];
			len++;
		}
	}
	ROS_INFO("len1=%d",len);
	if(Lbottom>0||Rbottom<(msg->width-1)){
		if(Lbottom>0){
			for(i=0;i<len;){
				if(Xlocate[i]<=Lbottom){
					dotline[Xlocate[i]]=Unfound;//ROS_INFO("%d",Xlocate[i]);
					i++;
				}
				else i=len;
			}
		}
		if(Rbottom<(msg->width-1)){
			for(j=len-1;j>=0;){
				if(Xlocate[j]>=Rbottom){
					dotline[Xlocate[j]]=Unfound;
					j--;
				}
				else j=-1;
			}	
		}
	}
	dotline[Lbottom]=Divided;
	dotline[Rbottom]=Divided;//ROS_INFO("LeftBottom=%d,RightBottom=%d",Lbottom,Rbottom);
	for(i=0;i<msg->width;i++){
		Xlocate[i]=0;
		Ylocate[i]=0;
	}
	len=0;
	for(i=0;i<msg->width;i++){
		if(dotline[i]==Divided){
			Xlocate[len]=i;//ROS_INFO("i=%d",i);
			Ylocate[len]=boundary.data[i];
			len++;
		}
	}
	ROS_INFO("len2=%d",len);
	for(i=0;i<msg->width;i++){
		dotline[i]=Unfound;
	}
	if(len>3){
		for(i=0;i<len-3;){
		 if(Ylocate[i+1]>(Ylocate[i]*(Xlocate[i+2]-Xlocate[i+1])+Ylocate[i+2]*(Xlocate[i+1]-Xlocate[i]))/(Xlocate[i+2]-Xlocate[i])||Xlocate[i]%8==0){dotline[Xlocate[i]]=Divided;
		  for(j=i+2;j<len;j++){
		   if(Ylocate[j]<(Ylocate[i]*(Xlocate[j+1]-Xlocate[j])+Ylocate[j+1]*(Xlocate[j]-Xlocate[i]))/(Xlocate[j+1]-Xlocate[i])||Xlocate[j]%8==0){
			dotline[Xlocate[j]]=Divided;
			for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
			}
			i=j;
		        break;
		   }
			if(j==len-2){
				dotline[Xlocate[j]]=Divided;
				j=len-1;
				for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}
				i=j;
			break;
			}
			if(j==len-1){
				dotline[Xlocate[j]]=Divided;
				j=len-1;
				for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}
				i=j;
			break;
			}
		  }
		 }else i++;
		}
	}
	if(Lbottom>0||Rbottom<(msg->width-1)){
		if(Lbottom>0){
			for(i=0;i<len;){
				if(Xlocate[i]<Lbottom){
					dotline[Xlocate[i]]=Unfound;//ROS_INFO("%d",Xlocate[i]);
					i++;
				}
				else i=len;
			}
		}
		if(Rbottom<(msg->width-1)){
			for(j=len-1;j>=0;){
				if(Xlocate[j]>Rbottom){
					dotline[Xlocate[j]]=Unfound;
					j--;
				}
				else j=-1;
			}	
		}
	}
	//dotline[Lbottom]=Divided;
	//dotline[Rbottom]=Divided;	
	for(i=0;i<msg->width;i++){
		Xlocate[i]=0;
		Ylocate[i]=0;
	}
	len=0;
	for(i=0;i<msg->width;i++){
		if(dotline[i]==Divided){
			Xlocate[len]=i;//ROS_INFO("i=%d",i);
			Ylocate[len]=boundary.data[i];
			len++;
		}
	}
	for(i=0;i<msg->width;i++){
		dotline[i]=Unfound;
	}
	ROS_INFO("len3=%d",len);
	if(len>3){
		for(i=0;i<len-2;){
		 if(Ylocate[i+1]>(Ylocate[i]*(Xlocate[i+2]-Xlocate[i+1])+Ylocate[i+2]*(Xlocate[i+1]-Xlocate[i]))/(Xlocate[i+2]-Xlocate[i])){
		  dotline[Xlocate[i]]=Divided;
		  for(j=i+2;j<len;j++){
		   if(Ylocate[j]<(Ylocate[i]*(Xlocate[j+1]-Xlocate[j])+Ylocate[j+1]*(Xlocate[j]-Xlocate[i]))/(Xlocate[j+1]-Xlocate[i])){
			dotline[Xlocate[j]]=Divided;
			for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
			}i=j;
		        break;
		   }
			if(j==len-2){
				j=len-1;
				dotline[Xlocate[j]]=Divided;
				for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}i=j;
			break;
			}
			if(j==len-1){
				j=len-1;
				dotline[Xlocate[j]]=Divided;
				for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}
				i=j;
			break;
			}
		  }
		 }else i++;
		}
	}
	for(i=0;i<msg->width;i++){
		Xlocate[i]=0;
		Ylocate[i]=0;
	}
	len=0;
	for(i=0;i<msg->width;i++){
		if(dotline[i]==Divided){
			Xlocate[len]=i;//ROS_INFO("i=%d",i);
			Ylocate[len]=boundary.data[i];
			len++;
		}
	}
	for(i=0;i<msg->width;i++){
		dotline[i]=Unfound;
	}
	ROS_INFO("len4=%d",len);
	if(len>3){
		for(i=0;i<len-2;){
		 if(Ylocate[i+1]>(Ylocate[i]*(Xlocate[i+2]-Xlocate[i+1])+Ylocate[i+2]*(Xlocate[i+1]-Xlocate[i]))/(Xlocate[i+2]-Xlocate[i])){
		 for(j=i+2;j<len;j++){
		   if(Ylocate[j]<(Ylocate[i]*(Xlocate[j+1]-Xlocate[j])+Ylocate[j+1]*(Xlocate[j]-Xlocate[i]))/(Xlocate[j+1]-Xlocate[i])){
			for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
			}i=j;
		        break;
		   }
			if(j==len-2){
				j=len-1;
				for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}i=j;
			break;
			}
			if(j==len-1){
				dotline[Xlocate[i]]=Divided;
				j=len-1;
				for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}
				i=j;
			break;
			}
		  }
		 }else i++;
		}
	}
	else if(len==3){
		i=0;
		if(Ylocate[i+1]>(Ylocate[i]*(Xlocate[i+2]-Xlocate[i+1])+Ylocate[i+2]*(Xlocate[i+1]-Xlocate[i]))/(Xlocate[i+2]-Xlocate[i])){
			j=i+2;
			for(k=Xlocate[i]+1;k<Xlocate[j];k++){
				boundary.data[k]=boundary.data[Xlocate[i]]+(boundary.data[Xlocate[j]]-boundary.data[Xlocate[i]])*(k-Xlocate[i])/(Xlocate[j]-Xlocate[i]);
				}
		}
	}
	delete [] edge;
	//for(i=0;i<msg->width;i++)ROS_INFO("%d",boundary.data[i]);*/
//寻找障碍物，首先确定边界，然后在分片图像中找寻，宽搜
	int* fieldline=new int[msg3.width];
	int boX=0,boY=0,bolX=0;
	for(i=0;i<msg->width;i=i+pix){
		for(j=0;j<msg->height;j=j+pix){
	    	  	for(n=j;n<j+pix;n++){
			   	if(n==boundary.data[i]){
					pic[j/pix][i/pix]=Edge;
					fieldline[i/pix]=j/pix;//ROS_INFO("i=%d,f=%d",i/pix,fieldline[i/pix]);
					n=j+pix;
					j=msg->height;
				}
			}
		}
	}//ROS_INFO("fieldline[lebo](%d) ",fieldline[lebo]);
	Queue<int> queue2;
	for(i=0;i<msg3.width;i++){
		for(j=fieldline[i];j<msg3.height;){//ROS_INFO("%d1",j);
		    if(msg3.data[j*msg3.width+i]!=0){//ROS_INFO("%d2",j);
			j++;
		    }else{
			if(pic[j][i]==Else||pic[j][i]==Unknown){//ROS_INFO("%d3",j);
				queue2.insertRear(i,j);
				do{
					m=queue2.backAddress();
					n=queue2.deleteFront();
					queue.insertRear(n,m);
					if(n<msg3.width-1&&n>0){
						if(m<msg3.height-1){
							if(pic[m+1][n]==Unknown||pic[m+1][n]==Else){
								if(msg3.data[(m+1)*msg3.width+n]==0){
									queue2.insertRear(n,m+1);
									pic[m+1][n]=Checked;
								}
								else {pic[m+1][n]=Denied;}
							}
							if(pic[m][n+1]==Unknown||pic[m][n+1]==Else){
								if(msg3.data[m*msg3.width+(n+1)]==0){
									queue2.insertRear(n+1,m);
									pic[m][n+1]=Checked;
								}
								else {pic[m][n+1]=Denied;}
							}
							if(pic[m][n-1]==Unknown||pic[m][n-1]==Else){
								if(msg3.data[(m)*msg3.width+(n-1)]==0){
									queue2.insertRear(n-1,m);
									pic[m][n-1]=Checked;
								}
								else {pic[m][n-1]=Denied;}
							}
						}
						else {
							if(pic[m][n+1]==Unknown||pic[m][n+1]==Else){//ROS_INFO("1#########");
								if(msg3.data[m*msg3.width+(n+1)]==0){
									queue2.insertRear(n+1,m);
									pic[m][n+1]=Checked;
								}
								else {pic[m][n+1]=Denied;}
							}
							if(pic[m][n-1]==Unknown||pic[m][n-1]==Else){//ROS_INFO("2#############");
								if(msg3.data[m*msg3.width+(n-1)]==0){
									queue2.insertRear(n-1,m);
									pic[m][n-1]=Checked;
								}
								else {pic[m][n-1]=Denied;}
							}
						}
					}
					else if(n==msg3.width){
						if(m<msg3.height-1){
							if(pic[m+1][n]==Unknown||pic[m+1][n]==Else){
								if(msg3.data[(m+1)*msg3.width+n]==0){
									queue2.insertRear(n,m+1);
									pic[m+1][n]=Checked;
								}
								else {pic[m+1][n]=Denied;}
							}
							if(pic[m][n-1]==Unknown||pic[m][n-1]==Else){
								if(msg3.data[(m)*msg3.width+(n-1)]==0){
									queue2.insertRear(n-1,m);
									pic[m][n-1]=Checked;
								}
								else {pic[m][n-1]=Denied;}
							}
						}
						else{
							if(pic[m][n-1]==Unknown||pic[m][n-1]==Else){
								if(msg3.data[(m)*msg3.width+(n-1)]==0){
									queue2.insertRear(n-1,m);
									pic[m][n-1]=Checked;
								}
								else {pic[m][n-1]=Denied;}
							}
						}
					}
					else if(n==0){
						if(m<msg3.height-1){
							if(pic[m+1][n]==Unknown||pic[m+1][n]==Else){
								if(msg3.data[(m+1)*msg3.width+n]==0){
									queue2.insertRear(n,m+1);
									pic[m+1][n]=Checked;
								}
								else {pic[m+1][n]=Denied;}
							}
							if(pic[m][n+1]==Unknown||pic[m][n+1]==Else){
								if(msg3.data[m*msg3.width+(n+1)]==0){
									queue2.insertRear(n+1,m);
									pic[m][n+1]=Checked;
								}
								else {pic[m][n+1]=Denied;}
							}
						}
						else{
							if(pic[m][n+1]==Unknown||pic[m][n+1]==Else){
								if(msg3.data[m*msg3.width+(n+1)]==0){
									queue2.insertRear(n+1,m);
									pic[m][n+1]=Checked;
								}
								else {pic[m][n+1]=Denied;}
							}
						}
					}
					else {}
				}while(!queue2.isEmpty());//ROS_INFO("size=%d",queue.backSize());
				//ROS_INFO("max=%d,min=%d",queue.maxHigh(),queue.minHigh());
				if(queue.backSize()>8&&(queue.maxHigh()-queue.minHigh())>2){
					do{
						m=queue.backAddress();
						n=queue.deleteFront();
						if(m>=fieldline[n])pic[m][n]=Black;
						else pic[m][n]=Green;
					}while(!queue.isEmpty());
				}else{
					do{
						m=queue.backAddress();
						n=queue.deleteFront();
						pic[m][n]=Green;
					}while(!queue.isEmpty());
				}
			}
			else {j++;}
		   }
		}
	}
	queue2.clear();
	queue.clear();//ROS_INFO("2*****************************");
	for(j=msg3.height-1;j>=0;j--){
		for(i=0;i<msg3.width;i++){
			if(pic[j][i]==Black){//ROS_INFO("boX=%d,boY=%d",i,j);
				pic[j][i]=Checked;
				queue2.insertRear(i,j);
				boX=i;
				boY=j;
				bolX=i;
				do{
					m=queue2.backAddress();
					n=queue2.deleteFront();
					queue.insertRear(n,m);
					if(n<msg3.width-1&&n>0){
						if(m>0){
							if(pic[m-1][n]==Black){
								queue2.insertRear(n,m-1);
								pic[m-1][n]=Checked;
							}
							if(pic[m][n+1]==Black){
								queue2.insertRear(n+1,m);
								pic[m][n+1]=Checked;
							}
							if(pic[m][n-1]==Black){
								queue2.insertRear(n-1,m);
								pic[m][n-1]=Checked;
							}
						}
						else {
							if(pic[m][n+1]==Black){
								queue2.insertRear(n+1,m);
								pic[m][n+1]=Checked;
							}
							if(pic[m][n-1]==Black){
								queue2.insertRear(n-1,m);
								pic[m][n-1]=Checked;
							}
						}
					}
					else if(n==msg3.width){
						if(m>0){
							if(pic[m-1][n]==Black){
								queue2.insertRear(n,m-1);
								pic[m-1][n]=Checked;
							}
							if(pic[m][n-1]==Black){
								queue2.insertRear(n-1,m);
								pic[m][n-1]=Checked;
							}
						}
						else{
							if(pic[m][n-1]==Black){
								queue2.insertRear(n-1,m);
								pic[m][n-1]=Checked;
							}
						}
					}
					else if(n==0){
						if(m>0){
							if(pic[m-1][n]==Black){
								queue2.insertRear(n,m-1);
								pic[m-1][n]=Checked;
							}
							if(pic[m][n+1]==Black){
								queue2.insertRear(n+1,m);
								pic[m][n+1]=Checked;
							}
						}
						else{
							if(pic[m][n+1]==Black){
								queue2.insertRear(n+1,m);
								pic[m][n+1]=Checked;
							}
						}
					}
					else {}
				}while(!queue2.isEmpty());
				//ROS_INFO("boX1=%d,boY1=%d,bolX1=%d",boX,boY,bolX);						
				for(j=boY;j>=boY-2;j--){
					for(i=0;i<msg3.width;i++){
						if(pic[j][i]==Checked){
							if(i<boX)boX=i;
							i=msg3.width;
						}
					}
				}
				for(j=boY;j>=boY-2;j--){
					for(i=msg3.width-1;i>=0;i--){
						if(pic[j][i]==Checked){
							if(i>bolX)bolX=i;
							i=0;
						}
					}
				}
				//ROS_INFO("boX=%d,boY=%d,bolX=%d",boX,boY,bolX);
				if(bolX-boX==0){}
				else{
				area.iLeftEdgeInImageX=boX*pix;
				area.iLeftEdgeInImageY=boY*pix+pix-1;
				area.iRightEdgeInImageX=(bolX+1)*pix;
				area.iRightEdgeInImageY=boY*pix+pix-1;
				area.iHeghtInImage=(queue.maxHigh()-queue.minHigh()+1)*pix;//ROS_INFO("4*****************************");
				i=msg->width;
				j=-1;
				}
			}
		}
	}//ROS_INFO("3*****************************");
	/*if(area.iHeghtInImage==0&&area.iLeftEdgeInImageX==0&&ribo!=0){
		area.iLeftEdgeInImageX=lebo*pix;
		area.iLeftEdgeInImageY=yob*pix;
		area.iRightEdgeInImageX=ribo*pix;
		area.iRightEdgeInImageY=yob*pix;
		area.iHeghtInImage=(yob-fieldline[lebo])*pix;//ROS_INFO("(yob-fieldline[lebo])(%d) ",(yob-fieldline[lebo]));	
	}*/
	ROS_INFO("LeftEdge(%d,%d) ",area.iLeftEdgeInImageX,area.iLeftEdgeInImageY);
	ROS_INFO("RightEdge(%d,%d) ",area.iRightEdgeInImageX,area.iRightEdgeInImageY);
	ROS_INFO("High(%d) ",area.iHeghtInImage);
//输出找到的分片图像的绿色边界图raw_field_img
	msg3.data.resize(msg3.width*msg3.height);
	for(i=0;i<msg3.width;i++){	
	   for(j=0;j<msg3.height;j++){	
		if(pic[j][i]==Green)c=255;
		else if(pic[j][i]==Edge)c=32*7;
		else if(pic[j][i]==Checked)c=32*5;
		else if(pic[j][i]==Denied)c=32*3;
		else if(pic[j][i]==Unknown)c=0;
		msg3.data[j*msg3.width+i]=c;
		}	
	}
//作出最终效果图，作出细化后的绿色边界图，并把障碍物标出
	sensor_msgs::Image msg1;
	msg1.width=msg->width;
  	msg1.height=msg->height;
  	msg1.encoding="mono8";
  	msg1.step=msg->step;
	msg1.data.resize(msg->width*msg->height);
	for(i=0;i<msg->width;i++){	
	   for(j=0;j<msg->height;j++){	
		//if(j==boundary.data[i])c=255;
		if(j<boundary.data[i])c=0;
		else c=msg->data[j*msg->width+i];
		msg1.data[j*msg->width+i]=c;
		}	
	}
	/*for(i=area.iLeftEdgeInImageX;i<area.iRightEdgeInImageX;i++){
	  for(j=area.iLeftEdgeInImageY;j>area.iLeftEdgeInImageY-area.iHeghtInImage;j--){	
		msg1.data[j*msg->width+i]=255;
	  }
	}*/

    pub.publish(msg1);
    rawpub.publish(msg3);
    bvpub.publish(msg0);
    fieldpub.publish(msg2);
    line_pub.publish(boundary);
    obstacle_pub.publish(area);//ROS_INFO("pub");
}
};
int main(int argc, char **argv){
  ros::init(argc, argv, "find_field");
  ImageChanger ic;
  ros::spin();
}
/*场地识别的难点：
1、场地中有非绿色物体，比如球，机器人，障碍物，场线
2、对于视野中上端超出场地的物体，如障碍物，球门，要能把场地边界线连上
3、尽量使场地边界线成直线*/
