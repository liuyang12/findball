#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include "ColorTable.h"
#include <cstdlib>
#include <stdio.h>

#define COLOR_TABLE_SIZE (64*64*64)
#define COLOR_TABLE_SIZE	(64*64*64)
#define MAX_NUM_CALIBRATED	(0xdfffffff)

using namespace ros;

enum ColorClass{
	Orange = 0,
	Yellow = Orange+1,
	Blue = Yellow+1,
	Green = Blue+1,
	White = Green+1,
	Magenta = White+1,
	Cyan = Magenta+1,
	Black = Cyan+1,
	NoColor = Black+1,
	NumOfCalibrationColors = NoColor,
	YellowOrange = NoColor+1,
	NumOfAllColors = YellowOrange+1
};

enum GT_ColorClass{
	noColor, /*<! all other objects */
	orange, /*<! ball */
	yellow, /*<! yellow goal and flag */
	skyBlue, /*<! skyblue goal and flag */
	pink, /*<! pink flag */
	red, /*<! red player */
	blue, /*<! blue player */
	green, /*<! playground and green flag */
	gray, /*<! player */
	white, /*<! boundaries and lines */
	black, /*<! for ball-challenge */
	yellowOrange, /*<! ball or yellow goal */
	numOfColors /*<! number of colors */
};

class CCombTable{
public:
	CCombTable(){}
	~CCombTable(){}
	bool ReadCombTable(char * strFileName);
	bool SaveCombTable(char * strFileName);
	bool ReadTable64(char * strFileName);
	bool SaveTable64(char * strFileName);
	bool ClassifyImage(sensor_msgs::Image *pOriginal,sensor_msgs::Image *pClassified,bool bUseSoftColorAlgorithm=false);
	unsigned char m_ColorTable[COLOR_TABLE_SIZE];

	void YUV2BGR(CCombTable * pDestTable);
	void YUV2BGR();
	void BGR2YUV(CCombTable * pDestTable);
	void BGR2YUV();
	unsigned char My2GT(unsigned char cls);
	unsigned char GT2My(unsigned char gtcls);
};

class ImageChanger
{
  ros::NodeHandle nh;

//Create image transport handle and connect it to the incoming/outgoing topics 
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  image_transport::Publisher pub; 

  CCombTable m_colorTable;

public:
  ImageChanger()
    : it(nh)
  { 
    char m_strColorTable[64] = "/home/young/文档/test_pics/all.c64";
//    sub = it.subscribe("resized_image_to_color_classification", 1, &ImageChanger::imageCb, this);
      sub = it.subscribe("image", 1, &ImageChanger::imageCb, this);
//    sub = it.subscribe("image", 1, &ImageChanger::imageCb, this, image_transport::TransportHints());
    pub = it.advertise("image_color_classified", 1); 
    
    if (!m_colorTable.ReadTable64(m_strColorTable))
      ROS_INFO("File not exists!");

  }

// Callback function
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    sensor_msgs::Image pub_image, classified_image;
    pub_image = *msg;

    ROS_INFO("passed1");
// Adjusting the data of the image according to the parameters
    classified_image.height = pub_image.height;
    classified_image.width = pub_image.width;
    classified_image.step = pub_image.width;
    size_t st0 = (pub_image.height * pub_image.width);
    classified_image.data.resize(st0);
    classified_image.encoding = "mono8";

    m_colorTable.ClassifyImage(&pub_image, &classified_image, false);
 
    ROS_INFO("passed2");


//*************************************test**********************************
  ROS_INFO("width=%d height=%d", classified_image.width, classified_image.height);

   /* for(int i = 0; i < classified_image.height; i++)
    {
       for(int j = 0; j < classified_image.width; j++)
         printf("%d ",classified_image.data[i*classified_image.width+j]);
       printf("\n");
    }*/
//*************************************test********************************

//Publishing the adjusted image
    pub.publish(classified_image);

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_changer");
  ImageChanger ic;
  ros::spin();
  return 0;
}

bool CCombTable::ReadCombTable(char * strFileName)
{
	FILE *fp=NULL;
	fp = fopen(strFileName,"rb");
	if(!fp)return false;
	size_t iRead=fread(m_ColorTable,sizeof(unsigned char),COLOR_TABLE_SIZE,fp);
	fclose(fp);
	return (iRead==COLOR_TABLE_SIZE);
}

bool CCombTable::SaveCombTable(char * strFileName)
{
	FILE *fp=NULL;
	fp = fopen(strFileName,"wb");
	if(!fp)return false;
	fwrite(m_ColorTable,sizeof(unsigned char),COLOR_TABLE_SIZE,fp);
	fclose(fp);
	return true;
}

bool CCombTable::ReadTable64(char * strFileName)
{
	FILE *fp=NULL;
	fp = fopen(strFileName,"rb");
	if(!fp)return false;
	fseek(fp,0,SEEK_SET);
	size_t iLen=ftell(fp);
	fseek(fp,0,SEEK_END);
	iLen=ftell(fp)-iLen;
	fseek(fp,0,SEEK_SET);
	if(iLen<0)
	{
		fclose(fp);
		return false;
	}

	unsigned char *chCompress=new unsigned char[iLen];
	memset(m_ColorTable,NoColor,sizeof(unsigned char)*COLOR_TABLE_SIZE);
	size_t iRead=fread(chCompress,sizeof(unsigned char),iLen,fp);
	if(iRead!=iLen || iRead%5)
	{
		delete [] chCompress;
		fclose(fp);
		return false;
	}
	else
	{
		size_t currLen=0;
		size_t lSet=0;
		unsigned char currClass=NoColor;
		unsigned char *pt=chCompress;
		for(size_t i=0;i<iRead/5;i++)
		{
			currLen= (pt[0]) | (pt[1]<<8) | (pt[2]<<16) | (pt[3]<<24);
			currClass=GT2My(pt[4]);
			memset(m_ColorTable+lSet,currClass,currLen);
			lSet+=currLen;
			if(lSet>=COLOR_TABLE_SIZE)break;
			pt+=5;
		}
		delete [] chCompress;
		fclose(fp);
		return true;
	}
}

bool CCombTable::SaveTable64(char * strFileName)
{
	FILE *fp=NULL;
	fp = fopen(strFileName,"wb");
	if(!fp)return false;

	int iStart=0,i=0,iLen;
	unsigned char clr=noColor;
	while(i<COLOR_TABLE_SIZE)
	{
		iStart=i;
		while(i<COLOR_TABLE_SIZE && m_ColorTable[i]==m_ColorTable[iStart])i++;
		iLen=i-iStart;
		fwrite(&iLen,sizeof(int),1,fp);
		clr=My2GT(m_ColorTable[iStart]);
		fwrite(&clr,1,1,fp);
	}
	fclose(fp);
	return true;
}

bool CCombTable::ClassifyImage(sensor_msgs::Image *pOriginal,sensor_msgs::Image *pClassified,bool bUseSoftColorAlgorithm/* =false */)
{
	if(pOriginal==NULL || pClassified==NULL || pOriginal->height<=0 || pOriginal->width<=0 || pClassified->width<=0 || pClassified->height<=0) return false;
	
	unsigned char cBGR[NumOfAllColors]={
		1 << 5,
		2 << 5,//***********************
		3 << 5,
		4 << 5,
		5 << 5,
		6 << 5,
		7 << 5,
		0 << 5
	};

	int n = 0, m = 0;
	for(int i=0;i<pOriginal->height;i++)
	{
		for(int j=0;j<pOriginal->width;j++)
		{
			unsigned char ccolor=m_ColorTable[(pOriginal->data[n]>>2)*64*64+(pOriginal->data[n+1]>>2)*64+(pOriginal->data[n+2]>>2)];
			pClassified->data[m] = *(cBGR+ccolor);
			n += 3; m++;
		}
	}

	return true;
}


void CCombTable::BGR2YUV(CCombTable * pDestTable)
{
	unsigned char *pSrc=m_ColorTable;
	unsigned char *pDest=pDestTable->m_ColorTable;
	memset(pDest,NoColor,sizeof(unsigned char)*COLOR_TABLE_SIZE);
	for(int B=0;B<64;B++)
	{
		for(int G=0;G<64;G++)
		{
			for(int R=0;R<64;R++)
			{
				if(pSrc[(B<<12) | (G<<6) | R]==NoColor)continue;
				int y =        (int)( 0.2990 * R + 0.5870 * G + 0.1140 * B);
				int cb = 32 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B); //officially U
				int cr = 32 + (int)( 0.5000 * R - 0.4187 * G - 0.0813 * B); //officially V
				if(y < 0) y = 0;   else if(y > 63) y = 63;
				if(cb < 0) cb = 0; else if(cb > 63) cb = 63;
				if(cr < 0) cr = 0; else if(cr > 63) cr = 63;
				pDest[(y<<12) | (cb<<6) | cr]=pSrc[(B<<12) | (G<<6) | R];
			}
		}
	}
}

void CCombTable::BGR2YUV()
{
	unsigned char *pSrc=m_ColorTable;
	unsigned char pDest[COLOR_TABLE_SIZE];
	memset(pDest,NoColor,sizeof(unsigned char)*COLOR_TABLE_SIZE);
	for(int B=0;B<64;B++)
	{
		for(int G=0;G<64;G++)
		{
			for(int R=0;R<64;R++)
			{
				if(pSrc[(B<<12) | (G<<6) | R]==NoColor)continue;
				int y =        (int)( 0.2990 * R + 0.5870 * G + 0.1140 * B);
				int cb = 32 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B); //officially U
				int cr = 32 + (int)( 0.5000 * R - 0.4187 * G - 0.0813 * B); //officially V
				if(y < 0) y = 0;   else if(y > 63) y = 63;
				if(cb < 0) cb = 0; else if(cb > 63) cb = 63;
				if(cr < 0) cr = 0; else if(cr > 63) cr = 63;
				pDest[(y<<12) | (cb<<6) | cr]=pSrc[(B<<12) | (G<<6) | R];
			}
		}
	}
	memcpy(m_ColorTable,pDest,sizeof(unsigned char)*COLOR_TABLE_SIZE);
}

void CCombTable::YUV2BGR(CCombTable * pDestTable)
{
	unsigned char *pSrc=m_ColorTable;
	unsigned char *pDest=pDestTable->m_ColorTable;
	memset(pDest,NoColor,sizeof(unsigned char)*COLOR_TABLE_SIZE);
	for(int B=0;B<64;B++)
	{
		for(int G=0;G<64;G++)
		{
			for(int R=0;R<64;R++)
			{
				int y =        (int)( 0.2990 * R + 0.5870 * G + 0.1140 * B);
				int cb = 32 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B); //officially U
				int cr = 32 + (int)( 0.5000 * R - 0.4187 * G - 0.0813 * B); //officially V
				if(y < 0) y = 0;   else if(y > 63) y = 63;
				if(cb < 0) cb = 0; else if(cb > 63) cb = 63;
				if(cr < 0) cr = 0; else if(cr > 63) cr = 63;
				pDest[(B<<12) | (G<<6) | R]=pSrc[(y<<12) | (cb<<6) | cr];
			}
		}
	}
}

void CCombTable::YUV2BGR()
{
	unsigned char *pSrc=m_ColorTable;
	unsigned char pDest[COLOR_TABLE_SIZE];
	memset(pDest,NoColor,sizeof(unsigned char)*COLOR_TABLE_SIZE);
	for(int B=0;B<64;B++)
	{
		for(int G=0;G<64;G++)
		{
			for(int R=0;R<64;R++)
			{
				int y =        (int)( 0.2990 * R + 0.5870 * G + 0.1140 * B);
				int cb = 32 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B); //officially U
				int cr = 32 + (int)( 0.5000 * R - 0.4187 * G - 0.0813 * B); //officially V
				if(y < 0) y = 0;   else if(y > 63) y = 63;
				if(cb < 0) cb = 0; else if(cb > 63) cb = 63;
				if(cr < 0) cr = 0; else if(cr > 63) cr = 63;
				pDest[(B<<12) | (G<<6) | R]=pSrc[(y<<12) | (cb<<6) | cr];
			}
		}
	}
	memcpy(m_ColorTable,pDest,sizeof(unsigned char)*COLOR_TABLE_SIZE);
}

unsigned char CCombTable::My2GT(unsigned char cls)
{
	switch(cls)
	{
	case Orange:
		return orange;
	case Yellow:
		return yellow;
	case Blue:
		return skyBlue;
	case Green:
		return green;
	case White:
		return white;
	case Magenta:
		return red;
	case Cyan:
		return blue;
	case Black:
		return black;
	case YellowOrange:
		return yellowOrange;
	default:
		return noColor;
	}
}

unsigned char CCombTable::GT2My(unsigned char gtcls)
{
	switch(gtcls)
	{
	case orange:
		return Orange;
	case yellow:
		return Yellow;
	case skyBlue:
		return Blue;
	case green:
		return Green;
	case white:
		return White;
	case red:
		return Magenta;
	case blue:
		return Cyan;
	case black:
		return Black;
	case yellowOrange:
		return YellowOrange;
	default:
		return NoColor;
	}
}
