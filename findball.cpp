///修改了权值w和min2，用于寻找椭圆
//修改了命名
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
/// opencv and cv_bridge includes
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//using namespace cv;
//using namespace std;

#include <vector>
#include <math.h>
#include <findball/ball.h>

#define COLOR_Orange  1*32
#define	COLOR_Yellow  2*32
#define	COLOR_Blue  3*32
#define	COLOR_Green 4*32
#define	COLOR_White  5*32
#define	COLOR_Black  0

#define WIDTH 480
#define HEIGHT 360

#define MINSIZE  10
#define MIN_BALLRATIO 0.3       // 球最小比率

struct coor{int x,y;};//坐标
struct circle{double x,y,r;};//圆
struct line{double a,b,c;};//直线


class ball{
private:
    static int s[8];
    static coor Orange_point[100000];   //all the points whose color is orange
    static int Posibility[400][400];  //the possibility whether the point is in the ball, maximum is 10, minimum is 1
    static int Color_of_pixel[400][400];  //color of every pixel in the picture
    static int r_num[1000];
    static coor Direction[5];//逆时针方向
    static int Height,Width;
    static double xx,yy,min,rr[1000],old_w;  //xx,yy为拟合时圆心的坐标，rr为拟合时凸包上的点与（xx，yy）的距离，min为类似方差的指标
    circle old_b,follow;
    int old_flag;

    static int bfs(coor a);
    static int cross(coor &a,coor &b,coor &c);//叉积
    static int dot(coor &a,coor &b,coor &c);//点积
    static double dis(coor &a,coor &b,coor &c);
    static double cir_dis(circle &a,circle &b);
    static void sort(int i,int j);
    static int convex(int l);//凸包
    static line midperpendicular(coor &a,coor &b);//中垂线
    static void crossover_point(line i,line j);//交点
    static void circumcenter(coor &a,coor &b,coor &c);//外心
    static double count(circle &,int &);
    static double count2(int x,int y,double r);//圆心(x,y)半径r凸包测试min2
    void noise(int& l);
    void reset(int GoalColor,const sensor_msgs::Image& image);


public:
    circle b;
    int flag;
    double w;
    void find(const sensor_msgs::Image& input,double &dx,double &dy,double &dRadius,int GoalColor=4*32);
    sensor_msgs::Image finder(const sensor_msgs::Image image);
    // 统计在以center为圆心，radius为半径的圆的求颜色的占有率
    double ball_ratio(const sensor_msgs::Image image, cv::Point2f center, float radius);
    ball();
};

coor ball::Direction[5]={{1,0},{0,1},{-1,0},{0,-1},{0,0}};//逆时针方向

coor ball::Orange_point[100000];
int ball::Posibility[400][400];
int ball::Color_of_pixel[400][400];
int ball::r_num[1000];
int ball::Height,ball::Width;
double ball::xx,ball::yy,ball::min,ball::rr[1000],ball::old_w;
int ball::s[8]={10,5,30,10,1,1,10,10};
ball::ball():old_flag(0){}


int ball::dot(coor &a,coor &b,coor &c)//ba*ca点积
{
    return ((b.x-a.x)*(c.x-a.x)+(b.y-a.y)*(c.y-a.y));
}

int ball::cross(coor &a,coor &b,coor &c)//ba*ca叉积 >0逆时针 <0顺时针
{
    return ((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
}

double ball::dis(coor &a,coor &b,coor &c)//c到ab的距离
{
    double x=dot(a,b,c);
    return sqrt(dot(a,c,c)-x*x/dot(a,b,b));
}

double ball::cir_dis(circle &a,circle &b)//计算两个圆之间的差距
{
    return 1+(abs(a.r*a.r-b.r*b.r)+(a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))/(b.r*b.r+a.r*a.r);
}


line ball::midperpendicular(coor &a,coor &b)//求ab的中垂线
{
    line k;
    k.a=a.x-b.x;
    k.b=a.y-b.y;
    k.c=-k.a*((a.x+b.x)/2.0)-k.b*((a.y+b.y)/2.0);
    return k;
}

void ball::crossover_point(line i,line j)//求两线交点
{
    if (j.b*i.a==i.b*j.a) {
        xx=0;
        yy=0;
        return;
    }
    xx=(j.c*i.b-i.c*j.b)/(j.b*i.a-i.b*j.a);
    yy=(j.c*i.a-i.c*j.a)/(i.b*j.a-j.b*i.a);
}

void ball::circumcenter(coor &a,coor &b,coor &c)//求三角形外心
{
    crossover_point(midperpendicular(a,b),midperpendicular(a,c));
}


int ball::bfs(coor a){//以a为起点宽搜色块
    int l=0;
    int r=1;
    Orange_point[l]=a;
    Posibility[a.x][a.y]=10;
    while ( l<r ) {
        a=Orange_point[l++];
        for (int k=0;k<4;k++)
                {
                  if ( Posibility[a.x+Direction[k].x][a.y+Direction[k].y]==1 ){
                     coor b={a.x+Direction[k].x,a.y+Direction[k].y};
                 if ( ( Color_of_pixel[b.x][b.y]==COLOR_Black || Color_of_pixel[b.x][b.y]==COLOR_White || Color_of_pixel[b.x][b.y]==COLOR_Yellow) && Posibility[a.x][a.y]>2 ) {
            Orange_point[r++]=b;
            Posibility[b.x][b.y]=Posibility[a.x][a.y]-1;
              }
             if ( (Color_of_pixel[b.x][b.y]==COLOR_Green ) && Posibility[a.x][a.y]>6 ) {
            Orange_point[r++]=b;
            Posibility[b.x][b.y]=Posibility[a.x][a.y]-5;
              }
             if (Color_of_pixel[b.x][b.y]==COLOR_Orange) {
            Orange_point[r++]=b;
            Posibility[b.x][b.y]=10;
              }
                   }
                }
    }
    l=0;
        for (int i=0;i<r;i++) if(Color_of_pixel[Orange_point[i].x][Orange_point[i].y]==COLOR_Orange || Color_of_pixel[Orange_point[i].x][Orange_point[i].y]==COLOR_Yellow || Color_of_pixel[Orange_point[i].x][Orange_point[i].y]==COLOR_Black)
        {
        Orange_point[l++]=Orange_point[i];
        Color_of_pixel[Orange_point[i].x][Orange_point[i].y]==COLOR_Orange;
        }
        return l;  //橙色的点的数量
}

void ball::sort(int i,int j)//顺时针极角排序
{
    int l=i;
    int r=j;
    coor a=Orange_point[(l+r)>>1];
    coor b;
    while (l<=r) {
        while ( cross(Orange_point[0],a,Orange_point[l])>0 ) l++; //以a为基点
        while ( cross(Orange_point[0],a,Orange_point[r])<0 ) r--;
        if (l<=r) {
            b=Orange_point[l];
            Orange_point[l++]=Orange_point[r];
            Orange_point[r--]=b;
        }
    }
    if ( i<r ) sort(i,r);
    if ( l<j ) sort(l,j);
}



int ball::convex(int l)//凸包
{
    Orange_point[l]=Orange_point[0];
    int tot=1;
    for (int i=2;i<=l;i++) {
        while (cross(Orange_point[tot-1],Orange_point[tot],Orange_point[i])>=0 && tot>0){
            if (cross(Orange_point[tot-1],Orange_point[tot],Orange_point[i])==0 ){
                if (abs(Orange_point[tot-1].x-Orange_point[tot].x)+abs(Orange_point[tot-1].y-Orange_point[tot].y)>abs(Orange_point[tot-1].x-Orange_point[i].x)+abs(Orange_point[tot-1].y-Orange_point[i].y))
                    Orange_point[i]=Orange_point[tot];
            }
            tot--;
        }
        Orange_point[++tot]=Orange_point[i];
    }
    l=0;
    for (int i=0;i<tot;i++) if (!(Orange_point[i].x<3&&(Orange_point[i].y<3||Orange_point[i].y>Width-2))&&!(Orange_point[i].x>Height-2&&(Orange_point[i].y<3||Orange_point[i].y>Width-2)))
        Orange_point[l++]=Orange_point[i];
    tot=l;
    Orange_point[tot]=Orange_point[0];
    return tot;   //凸包上的点的数量
}


double ball::count(circle &b,int &tot)//以(x,y)作圆心的拟合测试
{
    b.r=0;
    double min=0.01;
    for (int i=0;i<tot;i++){
        rr[i]=sqrt((Orange_point[i].x-b.x)*(Orange_point[i].x-b.x)+(Orange_point[i].y-b.y)*(Orange_point[i].y-b.y)+0.0);
        b.r=b.r+rr[i];
    }
    b.r=b.r/tot;
    if (b.r>100) return 100;
    for (int i=0;i<tot;i++){
        min=min+abs(1-rr[i]/b.r);
    }
    return min/(tot-3);
}


void ball::reset(int GoalColor,const sensor_msgs::Image& image)//初始化
{
    Height=image.height;
    Width=image.width;
    b.x=0;
    b.y=0;
    b.r=0;
    flag=0;
    min=10;
    memset(Posibility,0,sizeof(Posibility));
    for (int i=1;i<=Height;i++)//颜色分割，bfs数组初始化
        for (int j=1;j<=Width;j++) {
            Posibility[i][j]=1;
            Color_of_pixel[i][j] = image.data[i*Width+j];
            //change GoalColor to orange
            if (GoalColor==COLOR_Orange) {
                // do nothing
            }
            else {
                if (Color_of_pixel[i][j]==GoalColor)
                    Color_of_pixel[i][j]=COLOR_Orange;//交换识别色为橙色
                else if (Color_of_pixel[i][j]==COLOR_Orange)
                    Color_of_pixel[i][j]=GoalColor;
            }
        }
}

void ball::noise(int& l)//去噪点处理
{
    int n=l,max=0,k;
    circle b={0,0,0};
    for (int i=0;i<n;i++) {
        b.x=b.x+Orange_point[i].x;
        b.y=b.y+Orange_point[i].y;
    }
    b.x=b.x/n;
    b.y=b.y/n;
    for (int i=0;i<n;i++) {
        double d=(Orange_point[i].x-b.x)*(Orange_point[i].x-b.x)+(Orange_point[i].y-b.y)*(Orange_point[i].y-b.y);
        b.r=b.r+d;
        d=sqrt(d);
        r_num[int(d)]++;
        if (d>max) max=int(d);
    }
    b.r=sqrt(b.r/n);
    //for (int i=max+1;i>b.r;i--) if (r_num[i]<4) k=i;
        for (int i=b.r;i<max+1;i++) if (r_num[i]>3) k=i; //找有可能的最大球
    for (int i=0;i<=max;i++) {
        r_num[i]=0;
    }
    l=1;
    for (int i=1;i<n;i++) if (sqrt(double(Orange_point[i].x-b.x)*(Orange_point[i].x-b.x)+(Orange_point[i].y-b.y)*(Orange_point[i].y-b.y))>=k) {
        Color_of_pixel[Orange_point[i].x][Orange_point[i].y]=5*32;   //将有可能的最大球之外的点置为绿色
    }
    for (int i=1;i<n;i++) {
        int d=sqrt(double(Orange_point[i].x-b.x)*(Orange_point[i].x-b.x)+(Orange_point[i].y-b.y)*(Orange_point[i].y-b.y));
        if (d+d>=b.r&&d<k) {
            int num=0;
            //for (int f=0;f<4;f++) if (Color_of_pixel[Orange_point[i].x+Direction[f].x][Orange_point[i].y+Direction[f].y]==COLOR_Orange) num++;
            //if ( num<=2 ) Orange_point[l++]=Orange_point[i];
                        Orange_point[l++]=Orange_point[i];
        }
    }
    b.r=b.r*sqrt(2.0);
    if ( old_flag ) {
        if (b.r<old_b.r/2 || b.r>old_b.r*2 || (b.x-old_b.x)*(b.x-old_b.x)+(b.y-old_b.y)*(b.y-old_b.y)>=b.r*b.r+old_b.r*old_b.r) return;
        double min1=(abs(b.r-old_b.r)/old_b.r+1)*old_w;
        min1=min1*count2(b.x,b.y,b.r);
        if (min1>min) return;
        follow=b;
        min=min1;
    }
}

double ball::count2(int x,int y,double r)//以(x,y)作圆心的颜色测试
{
    double min=1;
    double d=1;
    int num=1,sum=1;
    for (int i=x-r;i<=x+r;i++) if ( i<=Height && i>=1 ) {
        for (int j=y-r;j<=y+r;j++) if ( j<=Width && j>=1 ) {
            if ( (i-x)*(i-x)+(j-y)*(j-y)<=r*r ) {   //in the circle
                d=d+sqrt(fabs(r*r-((i-x)*(i-x)+(j-y)*(j-y))));
                min=min+s[Color_of_pixel[i][j]]*sqrt(fabs(r*r-((i-x)*(i-x)+(j-y)*(j-y))));
                sum++;
                if (Color_of_pixel[i][j]==COLOR_Orange) num++;
            }
        }
    }
    min=min/d;
    min=min*(sum/num);
    if (sum>num*4) return 1000000;
    if (r<10) r=10;
    sum=1;
    num=1;
    for (int i=x-2*r;i<=x+2*r;i++) if ( i<=Height && i>=1 ) {
        for (int j=y-2*r;j<=y+2*r;j++) if ( j<=Width && j>=1 ) {
            if ( Color_of_pixel[i][j]!=COLOR_White && Color_of_pixel[i][j]!=COLOR_Orange ) sum++;
            if ( Color_of_pixel[i][j]==COLOR_Green ) num++;
        }
    }
    min=min*num/sum;
    if (num*4<=sum) min=1000000;
    return min;
}

void ball::find(const sensor_msgs::Image& image,double &dx,double &dy,double &dRadius,int GoalColor)
{
    srand(time(0));
    this->reset(GoalColor,image);//初始化
    w=1000;//*****************************************
        //ROS_INFO("Height= %d, Width= %d",Height,Width); //******************************************
    for (int i=1;i<=Height;i++) {
        for (int j=1;j<=Width;j++) if ( Posibility[i][j]==1 && Color_of_pixel[i][j]==COLOR_Orange ){//Posibility[i][j]==1保证每个色块只搜一次
            circle best={0,0,0};
            coor a={i,j};
            int t=bfs(a);
                        //ROS_INFO("t= %d",t);  //***************************************
            int l=t;
            this->noise(l);
                        sort(1,l-1);
            int tot=convex(l);
                        //ROS_INFO("tot= %d",tot); //***************************************

                        //将较小的色块剔除
            int sum=0;
            for (int k=2;k<tot;k++) {//求面积sum
                sum+=cross(Orange_point[0],Orange_point[k],Orange_point[k-1]);
            }
            if ( t*20/(sum+1)<=5 || t<10 || tot<4) {//ROS_INFO("continue!");
                        continue;}//大小和色块判断

            double min2=50;  //最好球的权值*********************************
            for (int k=0;k<=20;k++) {//随机取3个点求圆心，作拟合测试*********************************
                int a=rand()%tot;
                int b=rand()%tot;
                int c=rand()%tot;
                while (b==a) b=rand()%tot;
                while (c==a||c==b) c=rand()%tot;
                circumcenter(Orange_point[a],Orange_point[b],Orange_point[c]);
                circle now={xx,yy,0};
                double min1=count(now,tot); //当前球的权值
                //if (old_flag) min1=min1*cir_dis(now,old_b);
                if (min1<min2 && ((now.r>=old_b.r/2 && now.r<=old_b.r*2)||(old_flag==0))) {
                    best=now;  //半径匹配之后的最好球
                    min2=min1;
                }
            }
            if (min2>=50) continue;//*********************************
            min2=min2*count2(best.x,best.y,best.r);
            if (min2<w) {
                b=best;
                w=min2;
                flag=1;
            }
        }
    }
    /*if (min<w&&cir_dis(follow,old_b)<cir_dis(b,old_b)) {//****************************
        b=follow;
        w=min;
        flag=2;
    }
    if (b.r!=0){
        old_b=b;
        old_w=w;
        old_flag=flag;
    } else old_flag=0;*/

    dx=b.y;
    dy=b.x;
    dRadius=b.r;

        //ROS_INFO("%f %f %f",dx,dy,dRadius);

}

double ball::ball_ratio(const sensor_msgs::Image image, cv::Point2f center, float radius)
{
    if(radius < 0.1)
        return -1;
    Height = image.height;
    Width = image.width;
    double ratio;
    ratio = 0;
    for(int h = int(center.y - radius); h <= int(center.y + radius); h++)
    {
        for(int w = int(center.x - radius); w <= int(center.x + radius); w++)
        {
            if(w < 0 || w >= Width || h < 0 || h >= Height)     // 超出图像边界
                continue;
            if((h-center.y)*(h-center.y)+(w-center.x)*(w-center.x) > radius*radius) // 在圆外
                continue;
            if(image.data[(h+1)*Width + (w+1)] == COLOR_Orange)
            {
                ratio += 1;
            }
        }
    }
    return (ratio*2.0/(CV_PI*radius*radius));
}

sensor_msgs::Image ball::finder(const sensor_msgs::Image image)
{
    /// opencv Mat format image imgMat
    ROS_INFO("ball finder");
    cv::Mat imgMat;
    Height = image.height;
    Width = image.width;
//    cv_bridge::toCvCopy()
    imgMat = cv::Mat(Height, Width, CV_8UC3, cv::Scalar(3));
    for(int j = 1; j <= Height; j++)
    {
        for(int i = 1; i <= Width; i++)
        {
            if(image.data[j * Width + i] == COLOR_Orange)       // 橙黄色 - 球
            {
                imgMat.at<cv::Vec3b>(j-1, i-1)[0] = 255;
                imgMat.at<cv::Vec3b>(j-1, i-1)[1] = 255;
                imgMat.at<cv::Vec3b>(j-1, i-1)[2] = 255;
            }

//            else if(image.data[j * Width + i] == COLOR_Black)
//            {
//                imgMat.at<cv::Vec3b>(j-1, i-1)[0] = 127;
//                imgMat.at<cv::Vec3b>(j-1, i-1)[1] = 127;
//                imgMat.at<cv::Vec3b>(j-1, i-1)[2] = 127;
//            }
        }
    }
    // Hough 圆
//    std::vector<cv::Vec3f> circles;
    //
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imggray;
    // Convert to gray image - one channel only
//    imggray = imgMat.clone();
    cv::cvtColor(imgMat, imggray, CV_BGR2GRAY);
    // 模糊处理 -> 降噪
//    cv::blur(imggray, imggray, cv::Size(5, 5));
    // 高斯模糊去噪
//    cv::GaussianBlur(imggray, imggray, cv::Size(5, 5), 2, 2);
//    cv::GaussianBlur(imggray, imggray, cv::Size(6, 6), 0, 0);
    // 寻找边缘
    cv::findContours(imggray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    int num_hull;   // 找到的边缘的数量
    num_hull = (int)contours.size();
    // 对每一个轮廓计算其凸包
    std::vector<std::vector<cv::Point> > hull(contours.size());
    // 遍历所有边缘，去除点数小于阈值的边缘
    std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();        // 指向第一条边缘
    while(itc != contours.end())
    {
        if(itc->size() < MINSIZE)       // 边缘所占据的节点数小于阈值
        {
            itc = contours.erase(itc);
        }
        else
        {
            ++itc;
        }
    }
    for(int i = 0; i < contours.size(); i++)
    {
        cv::convexHull(cv::Mat(contours[i]), hull[i], false, true);
    }
    // 遍历所有凸包，去除点数小于阈值的凸包
//    std::vector<std::vector<cv::Point> >::iterator ith = hull.begin();
//    while(ith != hull.end())
//    {
//        if(ith->size() < MINSIZE)
//        {
//            ith = hull.erase(ith);
//            itc = contours.erase(itc);
//        }
//        else
//        {
//            ++ith;
//            ++itc;
//        }
//    }
//    cv::Mat drawing = cv::Mat::zeros(imggray.size(), CV_8UC3);  // 构造全零三通道矩阵
    ROS_INFO("Number of contours: %d", (int)contours.size());
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        imgMat =  cv_ptr->image;
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return image;
    }
    if(contours.size() == 0)
    {
        ROS_INFO("NO BALL FOUND");
    }
    else
    {
        std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();        // 指向第一条边缘
        std::vector<std::vector<cv::Point> >::iterator ith = hull.begin(); ith = hull.begin();
        while(itc != contours.end()/* i < contours.size()*/)
        {
            float radius;
            cv::Point2f center;
            cv::minEnclosingCircle(cv::Mat(*itc/*contours[i]*/), center, radius);
            cv::Point center_point;
            center_point = center;
            // 统计圆中黄色点（求颜色）的个数
            double ratio = ball_ratio(image, center, radius);
            if(ratio < 0)
                break;
            if(ratio < MIN_BALLRATIO)   // 球颜色的比率小于阈值，去除
            {
                itc = contours.erase(itc);
                ith = hull.erase(ith);
            }
            else{
                cv::circle(imgMat, center_point, static_cast<int>(radius+4), cv::Scalar(255, 255, 0), 2);
                ROS_INFO("[finder]x = %f, y = %f, r = %f", center.x, center.y, radius+4);
                ++itc;
                ++ith;
            }
        }
    }
    ROS_INFO("%d BALL WAS FOUND", (int)contours.size());

//    for(int i = 0; i < num_hull; i++)
//    {
//        // hull[i] 第 i 个凸包
//    }
    // 绘出轮廓及其凸包
    for(int i = 0; i < contours.size(); i++)
    {
        cv::Scalar color_contours = cv::Scalar(255, 0, 0);  // Blue
//        cv::Scalar color_hull =  cv::Scalar(0, 0, 255);     // Red
        // 轮廓
        cv::drawContours(imgMat, contours, i, color_contours, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
        // 凸包
//        cv::drawContours(drawing, hull, i, color_hull, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
    }

    // Canny parameter sensitive **************** can damage the original result ****************
//    cv::Canny(imggray, imggray, 25, 75);
    // Reduce the noise so we avoid false circle detection
//    cv::GaussianBlur(imggray, imggray, cv::Size(9, 9), 2, 2 );
    // 凸包 - applying opencv cv::convexHull()
//    cv::convexHull(inPoints, outHull, clock, returnpoints);
////    // Hough 圆变换
//    cv::HoughCircles(imggray, circles, CV_HOUGH_GRADIENT, 2, imggray.rows/16, 100, 100, 0, 0 );
//    for(size_t i = 0; i < circles.size(); i++)
//    {
//        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        int radius = cvRound(circles[i][2]);
//        // circle center
//        cv::circle(imgMat, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
//        // circle outline
//        cv::circle(imgMat, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

//    }
//    // publish temp image
//////    imshow("grayed image", imgMat);

    cv_bridge::CvImage out_msg;
    out_msg.image = imgMat;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // sensor_msgs::image_encodings::BGR8 Mat 图片三通道 MONO8单通道灰度图
    sensor_msgs::Image grayimg = *out_msg.toImageMsg();
////    image_tmp.publish(grayimg);
    return grayimg;
}




//*****************************************************************************

class ImageChanger
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  ros::Publisher ball_pub;
  image_transport::Publisher image_pub;  //for test*************
  image_transport::Publisher image_tmp;  // temp image

private:
  double x, y, r;

public:
  ball my_ball;

  ImageChanger()
    : it(nh)
  {
      image_tmp = it.advertise("new_out", 1);       // new out image
    image_pub = it.advertise("out", 1);        //for test*************
    image_sub = it.subscribe("image_color_classified", 1, &ImageChanger::image_change, this);//field_img//
    ROS_INFO("I receive image");
    ball_pub = nh.advertise<findball::ball>("ball", 1000);
  }



  void image_change(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("sub");
    sensor_msgs::Image image;
    image = *msg;

    sensor_msgs::Image grayimg;
    grayimg = my_ball.finder(image);
    image_tmp.publish(grayimg);

    my_ball.find(image, x, y, r, COLOR_Orange);

    ROS_INFO("x=%f y=%f r=%f \n Dis=%f",x, y, r, -0.2858*r+6.0192);
    //ROS_INFO("x=%d y=%d r=%d",int(x+0.5), int(y+0.5), int(r+0.5));

    findball::ball ball_msg;
    ball_msg.iCenterInImageX = int(x+0.5);
    ball_msg.iCenterInImageY = int(y+0.5);
    ball_msg.iRadiusInImage = int(r+0.5);
    float Distance = sqrt(pow((50.4196/r),2) - pow(1.3,2));
    ball_msg.fDistance = Distance;
    if(int(r+0.5) < 2)
      ball_msg.bBallWasSeen = 0;
    else
      ball_msg.bBallWasSeen = 1;
    ROS_INFO("ball_was_seen:%d",ball_msg.bBallWasSeen);
    float Angle;
    Angle = asinf(0.11*(x-240)/(r*Distance))*180/3.14159;
    ROS_INFO("Distance = %f  Angle = %f",Distance,Angle);

    ball_pub.publish(ball_msg);

    for(int i = 0; i < image.width; i++)
    {
      for(int j = 0; j < image.height; j++)
      {
        if(( (i-x)*(i-x) + (j-y)*(j-y) ) <= r*r && ( (i-x)*(i-x) + (j-y)*(j-y) ) >= (r-2)*(r-2))
           image.data[j*image.width+i] = 255;
      }
    }
    image_pub.publish(image);
    ROS_INFO("pub");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_changer2");
  ros::NodeHandle nh2;
  ball my_ball;
  ImageChanger ic;
  ros::spin();
  return 0;
}


////7.3新算法
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
//#include <math.h>
//#include <findball/ball.h>
//#include <cv_bridge/cv_bridge.h>
//#include <cv.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
////#include "gait/head_angle_msg.h"

//#define COLOR_Orange  1*32
//#define	COLOR_Yellow  2*32
//#define	COLOR_Blue  3*32
//#define	COLOR_Green 4*32
//#define	COLOR_White  5*32
//#define	COLOR_Black  0
//#define WIDTH 360
//#define HEIGHT 480

//struct coor{int x,y;};//坐标
//struct circle{double x,y,r;};//圆
//struct line{double a,b,c;};//直线

//static int r_pre = 15;
//static int loop_time = 10;
//bool ball_seen[10];

//class ball{
//private:
//    static int max_int(int a, int b);
//    static int min_int(int a, int b);
//    void adjust_center(const sensor_msgs::Image& image,int &old_x, int &old_y,int &std_R,float &rate);
//public:
//    void find(sensor_msgs::Image& input,double &dx,double &dy,double &dRadius,int GoalColor=4*32);
//    ball();
//};

//ball::ball()
//{

//}

//int ball::max_int(int a, int b)
//{
//    if(a < b)
//        return b;
//    else
//        return a;
//}

//int ball::min_int(int a, int b)
//{
//    if(a > b)
//        return b;
//    else
//        return a;
//}


//void ball::adjust_center(const sensor_msgs::Image& image,int &old_x, int &old_y,int &std_R,float &rate)
//{
//    int new_x,new_y;
//    int num_tot;
//    int num_orange;
//    for(new_x = max_int(0,old_x-10); new_x < min_int(WIDTH,old_x+10); new_x = new_x+2)
//    {
//        for(new_y = max_int(0,old_y-10); new_y < min_int(HEIGHT,old_y+10); new_y = new_y+2)
//        {
//            num_orange = 0;
//            num_tot = 0;
//            for(int k1 = max_int(1,new_x-std_R); k1 <= min_int(WIDTH-1,new_x+std_R); k1++)
//            {
//                for(int k2 = max_int(1,new_y-std_R); k2 <= min_int(HEIGHT-1,new_y+std_R); k2++)
//                {
//                    if(pow(k1-new_x,2)+pow(k2-new_y,2) < pow(std_R,2))
//                    {
//                        num_tot++;
//                        if(image.data[k2*image.width+k1] == COLOR_Orange)
//                            num_orange++;
//                    }
//                }
//            }
//            if(float(num_orange)/num_tot > rate)
//            {
//               rate = float(num_orange)/num_tot;
//               old_x = new_x;
//               old_y = new_y;
//            }
//        }
//    }
//}

//void ball::find(sensor_msgs::Image& image,double &dx,double &dy,double &dRadius,int GoalColor)
//{
//    srand(time(0));
//    int std_R = 15;//标准半径
//    int num;
//    //滤波
//    for(int i = 1; i < image.width-1; i++)
//    {
//        for(int j = 1; j < image.height-1; j++)
//        {
//            if(image.data[j*image.width+i] == COLOR_Orange)
//            {
//                num = 0;
//                for(int k1 = i-1; k1 <= i+1; k1++)
//                {
//                    for(int k2 = j-1; k2 <= j+1; k2++)
//                    {
//                        if(image.data[k2*image.width+k1] == COLOR_Orange)
//                            num++;
//                    }
//                }
//                if(num < 3)
//                    image.data[j*image.width+i] = COLOR_Black;
//            }
//            /*else
//                image.data[j*image.width+i] = COLOR_Black;*/
//        }
//    }
//    //算法开始
//    int num_tot;//std_R半径内的总点数
//    int num_orange;//std_R半径内的橙色点数
//    int num_tot2;//std_R到std_R+4圆环内的点总数
//    int num_green;//std_R到std_R+4圆环内的绿色点数
//    float max_rate = 0;
//    int x = 0,y = 0;
//    //以std_R为半径找到橙色占有率最大的区域，确定圆心
//    for(int i = 1; i < image.width-1; i++)
//    {
//        for(int j = 1; j < image.height-1; j++)
//        {
//            if(image.data[j*image.width+i] == COLOR_Orange)
//            {
//                num_orange = 0;
//                num_tot = 0;
//                num_tot2 = 0;
//                num_green = 0;
//                //统计橙色占有率
//                for(int k1 = max_int(1,i-std_R); k1 <= min_int(image.width-1,i+std_R); k1++)
//                {
//                    for(int k2 = max_int(1,j-std_R); k2 <= min_int(image.height-1,j+std_R); k2++)
//                    {
//                         if(pow(k1-i,2)+pow(k2-j,2) < pow(std_R,2))
//                         {
//                            num_tot++;
//                            if(image.data[k2*image.width+k1] == COLOR_Orange)
//                                num_orange++;
//                         }
//                    }
//                }

//                if(float(num_orange)/num_tot > max_rate)
//                {
//                    //统计圆环的绿色占有率
//                    for(int k1 = max_int(1,i-std_R-4); k1 <= min_int(image.width-1,i+std_R+4); k1++)
//                    {
//                        for(int k2 = max_int(1,j-std_R-4); k2 <= min_int(image.height-1,j+std_R+4); k2++)
//                        {
//                             if(pow(k1-i,2)+pow(k2-j,2) > pow(std_R,2) && pow(k1-i,2)+pow(k2-j,2) < pow(std_R+4,2))
//                             {
//                                num_tot2++;
//                                if(image.data[k2*image.width+k1] == COLOR_Green)
//                                    num_green++;
//                             }
//                        }
//                    }
//                    if(float(num_green)/num_tot2 > 0.3)//圆环的绿色占有率需大于0.1
//                    {
//                        max_rate = float(num_orange)/num_tot;
//                        x = i;
//                        y = j;
//                    }
//                }
//            }
//        }
//    }
//    //确定半径
//    if(max_rate > 0.1)
//    {
//        bool inc = true;//ture表示增加，false表示减小
//        float rate_now = 1;//当前区域的橙色占有率
//        float rate_pre;//上一次尝试的橙色占有率
//        //现扩大半径，直到橙色占有率小于0.2，即认为它是可能的最大半径
//        while(rate_now > 0.3 && std_R < 30)
//        {
//            if(inc)
//                std_R++;
//            num_orange = 0;
//            num_tot=0;
//            for(int k1 = max_int(1,x-std_R); k1 <= min_int(image.width-1,x+std_R); k1++)
//            {
//                for(int k2 = max_int(1,y-std_R); k2 <= min_int(image.height-1,y+std_R); k2++)
//                {
//                    if(pow(k1-x,2)+pow(k2-y,2) < pow(std_R,2))
//                    {
//                        num_tot++;
//                        if(image.data[k2*image.width+k1] == COLOR_Orange)
//                            num_orange++;
//                    }
//                }
//            }
//            rate_now = float(num_orange)/num_tot;
//            adjust_center(image,x,y,std_R,rate_now);
//        }
//        rate_pre = rate_now;
//        while(std_R > 0)
//        {
//            std_R--;
//            num_orange = 0;
//            num_tot=0;
//            for(int k1 = max_int(1,x-std_R); k1 <= min_int(image.width-1,x+std_R); k1++)
//            {
//                for(int k2 = max_int(1,y-std_R); k2 <= min_int(image.height-1,y+std_R); k2++)
//                {
//                    if(pow(k1-x,2)+pow(k2-y,2) < pow(std_R,2))
//                    {
//                        num_tot++;
//                        if(image.data[k2*image.width+k1] == COLOR_Orange)
//                            num_orange++;
//                    }
//                }            }

//            rate_now = float(num_orange)/num_tot;
//            if((rate_now/rate_pre < 1.102) || rate_now > 0.4)
//            {
//                 //ROS_ERROR("AHAHAHAHAHA  %f  %f",rate_now,rate_now/rate_pre);
//                 break;
//            }
//            rate_pre = rate_now;
//            adjust_center(image,x,y,std_R,rate_pre);
//        }
//        dx = x;
//        dy = y;
//        dRadius = std_R*0.1+r_pre*0.9;
//        r_pre = dRadius;
//    }
//    else
//    {
//      dx = 0;
//      dy = 0;
//      dRadius = 0;
//      r_pre = 15;
//    }


//}


////*****************************************************************************

//class ImageChangerBall
//{
//  ros::NodeHandle nh;
//  image_transport::ImageTransport it;
//  image_transport::Subscriber image_sub;
//  ros::Publisher ball_pub;
//  //ros::Subscriber head_angle_sub;
//  //ros::Subscriber field_sub;
//  image_transport::Publisher image_pub;  //for test*************

//private:
//  double x, y, r;
//  double head_angle;

//public:
//  ball my_ball;

//  ImageChangerBall()
//    : it(nh)
//  {

//    image_pub = it.advertise("findball/out", 1);        //输出图片
//    image_sub = it.subscribe("findball/field_img", 1, &ImageChangerBall::image_change, this);//image_color_classified
//    ROS_INFO("I receive image");
//    ball_pub = nh.advertise<findball::ball>("findball/ball", 1000);
//    //head_angle_sub = nh.subscribe("Head_angle_message", 1, &ImageChangerBall::get_head_angle, this);
//    //field_sub = nh.subscribe("findball/field",10);
//    head_angle = 0;
//  }

//  /*void get_head_angle(const gait::head_angle_msg::ConstPtr &headAngle)
//  {
//      head_angle = headAngle->angle_head_up;
//  }*/

//  void image_change(const sensor_msgs::ImageConstPtr& msg)
//  {
//    ROS_INFO("image change");
//    sensor_msgs::Image image;
//    image = *msg;
//    cv::Mat imgMat;
//    imgMat = cv::Mat(image.height, image.width, CV_8UC1, cv::Scalar(0));
//    for(int j = 0; j < image.height; j++)
//    {
//        for(int i = 0; i < image.width; i++)
//        {
//            if(image.data[j * image.width + i] == COLOR_Orange)
//            {
//                imgMat.at<uchar>(j, i) = 255;
//            }
//        }
//    }
//    imshow("grayed image", imgMat);
//    float r_d[3][30];//radius and distance look-up table
//    float y_d[2][30];//Image_Y and distance look-up table
//    //head_angle=0,Radius
//    r_d[0][0] = 6.0; r_d[0][1] = 6.0; r_d[0][2] = 6.0; r_d[0][3] = 5.5; r_d[0][4] = 5.0;
//    r_d[0][5] = 3.2; r_d[0][6] = 3.0; r_d[0][7] = 2.8; r_d[0][8] = 2.3; r_d[0][9] = 2.2;
//    r_d[0][10] = 2.0; r_d[0][11] = 1.9; r_d[0][12] = 1.8; r_d[0][13] = 1.6; r_d[0][14] = 1.5;
//    r_d[0][15] = 1.4; r_d[0][16] = 1.2; r_d[0][17] = 1.2; r_d[0][18] = 1.1; r_d[0][19] = 1.1;
//    r_d[0][20] = 1.0; r_d[0][21] = 0.8; r_d[0][22] = 0.8; r_d[0][23] = 0.8; r_d[0][24] = 0.8;
//    r_d[0][25] = 0.8; r_d[0][26] = 0.8; r_d[0][27] = 0.8; r_d[0][28] = 0.8; r_d[0][29] = 0.8;
//    //head_angle=20,Radius
//    r_d[1][0] = 6.0; r_d[1][1] = 6.0; r_d[1][2] = 6.0; r_d[1][3] = 5.5; r_d[1][4] = 5.0;
//    r_d[1][5] = 3.5; r_d[1][6] = 2.9; r_d[1][7] = 2.4; r_d[1][8] = 2.2; r_d[1][9] = 1.8;
//    r_d[1][10] = 1.6; r_d[1][11] = 1.5; r_d[1][12] = 1.4; r_d[1][13] = 1.2; r_d[1][14] = 1.1;
//    r_d[1][15] = 1.0; r_d[1][16] = 1.0; r_d[1][17] = 0.9; r_d[1][18] = 0.9; r_d[1][19] = 0.8;
//    r_d[1][20] = 0.6; r_d[1][21] = 0.4; r_d[1][22] = 0.4; r_d[1][23] = 0.4; r_d[1][24] = 0.4;
//    r_d[1][25] = 0.4; r_d[1][26] = 0.4; r_d[1][27] = 0.4; r_d[1][28] = 0.4; r_d[1][29] = 0.4;
//    //head_angle=40,Radius
//    r_d[2][0] = 6.0; r_d[2][1] = 6.0; r_d[2][2] = 6.0; r_d[2][3] = 5.5; r_d[2][4] = 5.0;
//    r_d[2][5] = 4.0; r_d[2][6] = 3.5; r_d[2][7] = 2.9; r_d[2][8] = 2.5; r_d[2][9] = 2.1;
//    r_d[2][10] = 1.6; r_d[2][11] = 1.3; r_d[2][12] = 1.2; r_d[2][13] = 1.0; r_d[2][14] = 1.0;
//    r_d[2][15] = 0.9; r_d[2][16] = 0.7; r_d[2][17] = 0.4; r_d[2][18] = 0.2; r_d[2][19] = 0.2;
//    r_d[2][20] = 0.2; r_d[2][21] = 0.2; r_d[2][22] = 0.2; r_d[2][23] = 0.2; r_d[2][24] = 0.2;
//    r_d[2][25] = 0.2; r_d[2][26] = 0.2; r_d[2][27] = 0.2; r_d[2][28] = 0.2; r_d[2][29] = 0.2;
//    //head_angle=40,Image_Y
//    y_d[0][0] = 6.0; y_d[0][1] = 6.0; y_d[0][2] = 6.0; y_d[0][3] = 5.5; y_d[0][4] = 5.0;
//    y_d[0][5] = 4.0; y_d[0][6] = 3.5; y_d[0][7] = 3.0; y_d[0][8] = 2.6; y_d[0][9] = 2.2;
//    y_d[0][10] = 1.9; y_d[0][11] = 1.6; y_d[0][12] = 1.4; y_d[0][13] = 1.3; y_d[0][14] = 1.1;
//    y_d[0][15] = 1.0; y_d[0][16] = 0.8; y_d[0][17] = 0.7; y_d[0][18] = 0.6; y_d[0][19] = 0.5;
//    y_d[0][20] = 0.4; y_d[0][21] = 0.3; y_d[0][22] = 0.2; y_d[0][23] = 0.2; y_d[0][24] = 0.2;
//    y_d[0][25] = 0.2; y_d[0][26] = 0.2; y_d[0][27] = 0.2; y_d[0][28] = 0.2; y_d[0][29] = 0.2;
//    //head_angle=60,Image_Y
//    y_d[1][0] = 2.3; y_d[1][1] = 2.2; y_d[1][2] = 2.1; y_d[1][3] = 1.9; y_d[1][4] = 1.7;
//    y_d[1][5] = 1.6; y_d[1][6] = 1.4; y_d[1][7] = 1.5; y_d[1][8] = 1.2; y_d[1][9] = 1.0;
//    y_d[1][10] = 0.9; y_d[1][11] = 0.8; y_d[1][12] = 0.7; y_d[1][13] = 0.6; y_d[1][14] = 0.5;
//    y_d[1][15] = 0.4; y_d[1][16] = 0.3; y_d[1][17] = 0.2; y_d[1][18] = 0.2; y_d[1][19] = 0.2;
//    y_d[1][20] = 0.2; y_d[1][21] = 0.2; y_d[1][22] = 0.2; y_d[1][23] = 0.2; y_d[1][24] = 0.2;
//    y_d[1][25] = 0.2; y_d[1][26] = 0.2; y_d[1][27] = 0.2; y_d[1][28] = 0.2; y_d[1][29] = 0.2;

//    my_ball.find(image, x, y, r, COLOR_Orange);
//    ROS_INFO("ball: x=%d y=%d r=%d",int(x+0.5), int(y+0.5), int(r+0.5));
//    ROS_ERROR("head_angle:%f",head_angle);
//    findball::ball ball_msg;
//    ball_msg.bBallWasSeen = 1;
//    ball_msg.iCenterInImageX = int(x+0.5);
//    ball_msg.iCenterInImageY = int(y+0.5);
//    ball_msg.iRadiusInImage = int(r+0.5);
//    //seen or not
//    int seen_times = 0;
//    for(int i = 1; i < loop_time; i++ )
//        ball_seen[i] = ball_seen[i-1];
//    if(r < 2)
//        ball_seen[0] = false;
//    else
//        ball_seen[0] = true;
//    for(int i = 0; i < loop_time; i++ )
//    {
//        if(ball_seen[i] == true)
//            seen_times++;
//    }
//    if(seen_times < 4 || ball_seen[0] == false)
//        ball_msg.bBallWasSeen = 0;
//    //distance
//    float Distance;
//    float a,b;
//    if(r > 29)
//        Distance = 0.5;
//    else
//    {
//        if (head_angle < 0)
//        {
//            a = r_d[0][int(r)+1];
//            b = r_d[0][int(r)];
//        }
//        else if (head_angle >=0 && head_angle < 20)
//        {
//            a = r_d[0][int(r)+1]*(head_angle/20) + r_d[1][int(r)+1]*((20 - head_angle)/20);
//            b = r_d[0][int(r)]*(head_angle/20) + r_d[1][int(r)]*((20 - head_angle)/20);
//        }
//        else if (head_angle >=20 && head_angle < 40)
//        {
//            a = r_d[1][int(r)+1]*((head_angle - 20)/20) + r_d[2][int(r)+1]*((40 - head_angle)/20);
//            b = r_d[1][int(r)]*((head_angle - 20)/20) + r_d[2][int(r)]*((40 - head_angle)/20);
//        }
//        else if (head_angle >=40 && head_angle < 60)
//        {
//            a = y_d[0][int(y/12)+1]*((head_angle - 40)/20) + y_d[1][int(y/12)+1]*((60 - head_angle)/20);
//            b = y_d[0][int(y/12)]*((head_angle - 40)/20) + y_d[1][int(y/12)]*((60 - head_angle)/20);
//        }
//        else
//        {
//            a = y_d[0][int(y/12)+1];
//            b = y_d[0][int(y/12)];
//        }
//        Distance = (r - int(r))*(b - a) + a;

//    }
//    ball_msg.fDistance = Distance;
//    //Angle
//    float Angle;
//    if(r == 0)
//        Angle = 0;
//    else if(0.11*(x-240)/(r*Distance) > 1)
//        Angle = 70;
//    else if( 0.11*(x-240)/(r*Distance) < -1)
//        Angle = -70;
//    else
//        Angle = asinf(0.11*(x-240)/(r*Distance))*180/3.14159;
//    ball_msg.fAngle = Angle;
//    ROS_INFO("ball: Distance = %f  Angle = %f",Distance,Angle);

//    ball_pub.publish(ball_msg);

//    for(int i = 0; i < image.width; i++)
//    {
//      for(int j = 0; j < image.height; j++)
//      {
//        if(( (i-x)*(i-x) + (j-y)*(j-y) ) <= r*r && ( (i-x)*(i-x) + (j-y)*(j-y) ) >= (r-2)*(r-2))
//           image.data[j*image.width+i] = 255;
//      }
//    }
//    image_pub.publish(image);

//  }
//};



//int main(int argc, char** argv)
//{
//    for(int i = 0; i < loop_time; i++)
//        ball_seen[i] = false;
//  ros::init(argc, argv, "find_ball");
//  ros::NodeHandle nh2;
//  ball my_ball;
//  ImageChangerBall icb;
//  ros::spin();
//  return 0;
//}


