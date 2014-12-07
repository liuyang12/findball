#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <findball/ball.h>
#include <fstream> 
using namespace std;

int width, height, ball_x, ball_y, ball_r;

void drawball(const findball::ball::ConstPtr& msg)
{
  ball_x = msg->iCenterInImageX;
  ball_y = msg->iCenterInImageY;
  ball_r = msg->iRadiusInImage;
  ROS_INFO("x=%d  y=%d  r=%d",ball_x, ball_y, ball_r);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drawball");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("ball_img", 1);
  ros::Subscriber sub = nh.subscribe<findball::ball>("ball",1,drawball);

  

  nh.setParam("width", 480);
  nh.setParam("height", 360);
  /*nh.setParam("ball_x", 160);
  nh.setParam("ball_y", 100);
  nh.setParam("ball_r", 20);*/

  nh.getParam("width",width);
  nh.getParam("height",height);
  /*nh.getParam("ball_x",ball_x);
  nh.getParam("ball_y",ball_y);
  nh.getParam("ball_r",ball_r);*/

  sensor_msgs::Image ball_image;
  ball_image.height = height;
  ball_image.width = width;
  ball_image.step = width;
  ball_image.encoding = "mono8";
  ball_image.data.resize(height * width);

  int f_x[480][360];//原始LUT（仅有测试点）
  int f_y[480][360];
  int f_LUT_x[480][360];//生成的LUT
  int f_LUT_y[480][360];
  //初始化
  for(int i = 0; i < 480; i++)
  {
    for(int j = 0; j < 360; j++)
    {
       f_x[i][j] = 0;
       f_y[i][j] = 0;
       f_LUT_x[i][j] = 0;
       f_LUT_y[i][j] = 0;
    }
   }
  //读入数据
  double temp_fx,temp_fy;
  int x,y;  
  ifstream fin;//读入数据
  ofstream fout_x;//写数据（180x240）
  ofstream fout_y;//写数据（180x240）
  ofstream fout_dis_x;//写数据（距离x）
  ofstream fout_dis_y;//写数据（距离y）
  fin.open("/home/young/catkin_ws/f_LUT_origin.txt");
  fout_x.open("/home/young/catkin_ws/f_LUT_x.txt");
  fout_y.open("/home/young/catkin_ws/f_LUT_y.txt");
  fout_dis_x.open("/home/young/catkin_ws/LUT_x.txt");
  fout_dis_y.open("/home/young/catkin_ws/LUT_y.txt");
  while(fin >> x)
  {
    fin >> y;
    fin >> temp_fx;
    fin >> temp_fy;
    f_x[x][y] = int(temp_fx+0.5);
    f_y[x][y] = int(temp_fy+0.5);
    f_x[479-x][y] = int(temp_fx+0.5);
    f_y[479-x][y] = int(temp_fy+0.5);
  }
  fin.close();
  //画图
  for(int i = 0; i < ball_image.width; i++)
  {
    for(int j = 0; j < ball_image.height; j++)
    {
        ball_image.data[j*width+i] = 255;
          if(f_y[i][j] != 0)
          {
             for(int k1 = i-3; k1 < i+3; k1++)
             {
                for(int k2 = j-3; k2 < j+3; k2++)
                {
                   ball_image.data[k2*width+k1] = 0;
                }
             }
          }
    }
  }
  //制表
  int r;
  int sum_x;
  int count_x;
  int sum_y;
  int count_y;
  //计算每个点的fx（离他最近的有效点的值）
  for(int i = 0; i < 480; i++)
  {
    for(int j = 179; j < 360; j++)
    {
       sum_x = 0;
       count_x = 0;
       sum_y = 0;
       count_y = 0;
       r = 0;
       while(true)
       {
         for(int k1 = i-r; k1 <= i+r; k1++)
         {
           for(int k2 = j-r; k2 <= j+r; k2++)
           {
             if(k1 > 0 && k2 >0 && k1 < 480 && k2 < 360)
             {
	       sum_x = sum_x + f_x[k1][k2];
	       if(f_x[k1][k2] != 0)
	         {count_x++;}
             }
           }
         } 
         if(sum_x != 0)
         {
           f_LUT_x[i][j] = sum_x/count_x;
           break;
         }
         else
         {
           r++;
         }
       }
       //计算每个点的fy（离他最近的有效点的值）
       r = 0;
       while(true)
       {
         for(int k1 = i-r; k1 <= i+r; k1++)
         {
           for(int k2 = j-r; k2 <= j+r; k2++)
           {
             if(k1 >= 0 && k2 >= 0 && k1 < 480 && k2 < 360)
             { 
	       sum_y = sum_y + f_y[k1][k2];
	       if(f_y[k1][k2] != 0)
	         count_y++;
             }
           }
         }
         if(sum_y != 0)
         {
           f_LUT_y[i][j] = sum_y/count_y;
           break;
         }
         else
         {
           r++;
         }
       }
     }
   }
   for(int i = 0; i < 360; i++)
   {
     f_LUT_x[239][i] = 0;
   } 
   //写完整的LUT
   int dis_x,dis_y;
   for(int i = 180; i < 360; i++)
     {
       for(int j = 0; j < 480; j++)
        {
           dis_y = f_LUT_y[j][i]*130/(i-179);
           if(j == 239)
               dis_x = 0;
           else
               dis_x = (j-239)*sqrt(pow(130,2)+pow(dis_y,2))/sqrt(pow(f_LUT_x[j][i],2)+pow((i-179),2));
           fout_dis_x << dis_x << " ";
           fout_dis_y << dis_y << " ";
           fout_x << f_LUT_x[j][i] << " ";
           fout_y << f_LUT_y[j][i] << " ";
        }
        fout_dis_x << "\n";
        fout_dis_y << "\n";
        fout_x << "\n";
        fout_y << "\n";
      }

  ros::Rate loop_rate(1);
  int pex;
  while (nh.ok()) {
    /*for(int i = 0; i < ball_image.width; i++)
	{
	  for(int j = 0; j < ball_image.height; j++)
	  {
        pex = 255;
        if(f_LUT_y[i][j]-90 >= 0)
        {
          pex = f_LUT_y[i][j]-120;
        }
        else
        {
          pex = 0;
        }
        if(pex*5 > 255)
        {
          pex = 255;
        }
        else
        {
          pex = pex*5;
        }
	    ball_image.data[j*width+i] = pex;
	  }
    }*/

    pub.publish(ball_image);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
