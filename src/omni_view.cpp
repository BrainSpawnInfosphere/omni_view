//-----------------------------------
// Unwrapping Omnidirectional Images
//-----------------------------------
// Author: Andrzej Pronobis
// Contact: pronobis@csc.kth.se
//          www.csc.kth.se/~pronobis
//-----------------------------------

#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h" // handles raw or compressed images
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

class Options
{
public:
	enum {NO_INTERPOLATION,   // none
		BICUBIC_INTERPOLATION, // complex  
		BILINEAR_INTERPOLATION // simplier 
	};
	
	enum {NO_ESTIMATION, // assume center of circle is in center of image
		ESTIMATE_CENTER_THRESHOLD, // complex circular edge detection 
		ESTIMATE_CENTER_EDGE // complex circular edge detection 
	};
	
	Options(){
		interpolation = NO_INTERPOLATION;
		
		sx = sy = 1.0; // scale factor?
		cx = cy = -1;  // center (x,y)
		
		borderL=0;
		borderR=0;
		borderT=0;
		borderB=0;
		
		blackThr = 30; // threshold for black
		
		estimateCenterThreshold = true;
		estimateCenterEdge=false; // broken
		//estimation = ESTIMATE_CENTER_THRESHOLD;
		
		//unwrap = true;
		markImage = true;
		
		edgeCX = edgeCY = edgeDX = edgeDY = 0;
		
		ro = 230;
		ri = 30;
	}
	
	int cx, cy; // center image
	int ri, ro; // inner and outer radius
	int interpolation; // type of interpolation
	double sx, sy; // scale factor?
	int blackThr; // threshold for what is black ... this is outside the circle
	int borderT, borderB, borderL, borderR; // borders top, bottom, left, right
														 //bool fixedCenter;
	//int estimation;
	bool estimateCenterThreshold, estimateCenterEdge;
	//bool unwrap; // remove?
	int minRadius, maxRadius;
	int edgeCX, edgeCY, edgeDX, edgeDY;
	bool markImage; // draw perimeter and center
};




// ---------------------------------------
int rnd(double d)
{
	return static_cast<int>(d+0.5);
}
int rndf(float d)
{
	return static_cast<int>(d+0.5);
}


// ---------------------------------------
inline double linear(double v0, double v1, double x)
{
	return (v1-v0)*x+v0;
}


// ---------------------------------------
double bilinear(double v00, double v01, double v10, double v11, double x, double y)
{
	// Notation: vXY
	// Interpolate in X direction
	double vX0=linear(v00, v10, x);
	double vX1=linear(v01, v11, x);
	// Interpolation in Y direction
	return linear(vX0, vX1, y);
}


// ---------------------------------------
inline double cubic(double v_1, double v0, double v1, double v2, double x)
{
	double a0, a1, a2, a3, x2;
	x2 = x*x;
	a0 = v2 - v1 - v_1 + v0;
	a1 = v_1 - v0 - a0;
	a2 = v1 - v_1;
	a3 = v0;
	
	return (a0*x*x2+a1*x2+a2*x+a3);
}


// ---------------------------------------
double bicubic(double v_1_1, double v0_1, double v1_1, double v2_1,
               double v_10, double v00, double v10, double v20,
               double v_11, double v01, double v11, double v21,
               double v_12, double v02, double v12, double v22,
               double x, double y)
{
	// Notation: vXY
	// Interpolate in X direction
	double vX_1=cubic(v_1_1, v0_1, v1_1, v2_1, x);
	double vX0 =cubic(v_10,  v00,  v10,  v20, x);
	double vX1 =cubic(v_11,  v01,  v11,  v21, x);
	double vX2 =cubic(v_12,  v02,  v12,  v22, x);
	// Interpolation in Y direction
	return cubic(vX_1, vX0, vX1, vX2, y);
}


// ---------------------------------------
unsigned char getInterpolation(const Options &opt, IplImage* inputImg, int channel, double x, double y)
{
	// Notation: vXY
	// Get channel values for both interpolation types
	int x0=static_cast<int>(floor(x));
	int y0=static_cast<int>(floor(y));
	double v00=static_cast<double>(*reinterpret_cast<unsigned char *>
											 (inputImg->imageData + y0*inputImg->widthStep+x0*3+channel));
	double v01=static_cast<double>(*reinterpret_cast<unsigned char *>
											 (inputImg->imageData + (y0+1)*inputImg->widthStep+x0*3+channel));
	double v10=static_cast<double>(*reinterpret_cast<unsigned char *>
											 (inputImg->imageData + y0*inputImg->widthStep+(x0+1)*3+channel));
	double v11=static_cast<double>(*reinterpret_cast<unsigned char *>
											 (inputImg->imageData + (y0+1)*inputImg->widthStep+(x0+1)*3+channel));
	
	// Interpolate
	double interpResult=0;
	if (opt.interpolation == Options::BILINEAR_INTERPOLATION)
	{ // BILINEAR INTERPOLATION
		interpResult = bilinear(v00, v01, v10, v11, x-static_cast<double>(x0), y-static_cast<double>(y0));
	}
	else if (opt.interpolation == Options::BICUBIC_INTERPOLATION)
	{ // BICUBIC INTERPOLATION
	  // Get additional channel values
		double v_1_1=static_cast<double>(*reinterpret_cast<unsigned char *>
													(inputImg->imageData + (y0-1)*inputImg->widthStep+(x0-1)*3+channel));
		double v0_1=static_cast<double>(*reinterpret_cast<unsigned char *>
												  (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0)*3+channel));
		double v1_1=static_cast<double>(*reinterpret_cast<unsigned char *>
												  (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0+1)*3+channel));
		double v2_1=static_cast<double>(*reinterpret_cast<unsigned char *>
												  (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0+2)*3+channel));
		
		double v_10=static_cast<double>(*reinterpret_cast<unsigned char *>
												  (inputImg->imageData + (y0)*inputImg->widthStep+(x0-1)*3+channel));
		double v20=static_cast<double>(*reinterpret_cast<unsigned char *>
												 (inputImg->imageData + (y0)*inputImg->widthStep+(x0+2)*3+channel));
		
		double v_11=static_cast<double>(*reinterpret_cast<unsigned char *>
												  (inputImg->imageData + (y0+1)*inputImg->widthStep+(x0-1)*3+channel));
		double v21=static_cast<double>(*reinterpret_cast<unsigned char *>
												 (inputImg->imageData + (y0+1)*inputImg->widthStep+(x0+2)*3+channel));
		
		double v_12=static_cast<double>(*reinterpret_cast<unsigned char *>
												  (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0-1)*3+channel));
		double v02=static_cast<double>(*reinterpret_cast<unsigned char *>
												 (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0)*3+channel));
		double v12=static_cast<double>(*reinterpret_cast<unsigned char *>
												 (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0+1)*3+channel));
		double v22=static_cast<double>(*reinterpret_cast<unsigned char *>
												 (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0+2)*3+channel));
		
		// Perform interpolation
		interpResult = bicubic(v_1_1, v0_1, v1_1, v2_1,
									  v_10,  v00,  v10,  v20,
									  v_11,  v01,  v11,  v21,
									  v_12,  v02,  v12,  v22,
									  x-static_cast<double>(x0), y-static_cast<double>(y0));
	}
	
	// Check result before conversion
	if (interpResult<0)
		interpResult=0;
	if (interpResult>255)
		interpResult=255;
	
	return static_cast<unsigned char>(interpResult);
}

IplImage* createOutputImage(const Options &opt, IplImage* inputImg){
	// Create the unwrap image
	int uwWidth = static_cast<int>(ceil((opt.ro * 2.0 * M_PI)*opt.sx));
	int uwHeight = static_cast<int>(ceil((opt.ro-opt.ri + 1)*opt.sy));
	IplImage* img = cvCreateImage(cvSize(uwWidth, uwHeight), 8, 3);
	
	ROS_INFO("size %d %d",uwWidth,uwHeight);
	
	return img;
}


// ---------------------------------------
void unwrap(const Options &opt, IplImage* inputImg, IplImage* outputImg)
{
	// Create the unwrap image
	int uwWidth = static_cast<int>(ceil((opt.ro * 2.0 * M_PI)*opt.sx));
	int uwHeight = static_cast<int>(ceil((opt.ro-opt.ri + 1)*opt.sy));
	IplImage* unwrappedImg = outputImg;
	
	// Perform unwrapping
	for (int uwX=0; uwX<uwWidth; ++uwX)
		for (int uwY=0; uwY<uwHeight; ++uwY)
		{
			// Convert polar to cartesian
			double w=-static_cast<double>(uwX)*2.0*M_PI/static_cast<double>(uwWidth);
			double r=static_cast<double>(opt.ri) + 
			static_cast<double>(uwHeight-uwY)*static_cast<double>(opt.ro-opt.ri + 1)/static_cast<double>(uwHeight);
			double iX=r*cos(w)+opt.cx;
			double iY=r*sin(w)+opt.cy;
			
			// Do safety check
			if ((iX<1) || (iX>inputImg->width-2) || (iY<1) || (iY>inputImg->height-2))
			{
				*(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+0) = 0;
				*(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+1) = 0;
				*(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+2) = 0;
				ROS_ERROR("Failed safety check");
			}
			else // Tansform image data
			{
				if (opt.interpolation)
				{ // With interpolation
					*reinterpret_cast<unsigned char *>(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+0) = getInterpolation(opt, inputImg, 0, iX, iY);
					*reinterpret_cast<unsigned char *>(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+1) = getInterpolation(opt, inputImg, 1, iX, iY);
					*reinterpret_cast<unsigned char *>(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+2) = getInterpolation(opt, inputImg, 2, iX, iY);
				}
				else 
				{ // No interpolation
					int tmpX=rnd(iX);
					int tmpY=rnd(iY);
					
					*(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+0) =
					*(inputImg->imageData + tmpY*inputImg->widthStep+tmpX*3+0);
					*(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+1) =
					*(inputImg->imageData + tmpY*inputImg->widthStep+tmpX*3+1);
					*(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+2) =
					*(inputImg->imageData + tmpY*inputImg->widthStep+tmpX*3+2);
				} // if
			} // if
		} // for
}


// ---------------------------------------
void evaluateCenterThreshold(const Options &opt, IplImage* thrImg, int cx, int cy, double accuracy, double *var, double*mean)
{
	int iWidth = thrImg->width;
	int iHeight = thrImg->height;
	
	int outPointsSum=0;
	int outPointsSum2=0;
	int count=0;
	for (double w=0; w<2*M_PI; w+=accuracy)
	{
		double r=opt.ro;
		double iX=r*cos(w)+cx;
		double iY=r*sin(w)+cy;
		int outPoints=0;
		while ((iX>=0) && (iX<iWidth) && (iY>=0) && (iY<iHeight) )//&& r<opt.ro+60
		{
			if ((*reinterpret_cast<unsigned char *>(thrImg->imageData + rnd(iY)*thrImg->widthStep+rnd(iX)))>0)
				outPoints++;
			
			// Increment r;
			r+=1;
			iX=r*cos(w)+cx;
			iY=r*sin(w)+cy;
		}
		
		outPointsSum+=outPoints;
		outPointsSum2+=(outPoints*outPoints);
		count++;
	}
	
	// Calculate variance
	*mean = static_cast<double>(outPointsSum)/count;
	double mean2 = static_cast<double>(outPointsSum2)/count;
	*var = mean2 - (*mean) * (*mean);
}


// ---------------------------------------
void estimateCenterThreshold(const Options &opt, IplImage* inputImg, int *cx, int *cy)
{
	//int uwWidth = static_cast<int>(ceil((opt.ro * 2.0 * M_PI)*opt.sx));
	int iWidth = inputImg->width;
	int iHeight = inputImg->height;
	
	// Create thresholded image
	IplImage* thrImg = cvCreateImage(cvSize(iWidth, iHeight), 8, 1);
	for (int x=0; x<iWidth; ++x)
		for (int y=0; y<iHeight; ++y)
		{
			double intensity;
			if ((x<opt.borderL) || (y<opt.borderT) || (x>=iWidth-opt.borderR) || (y>=iHeight-opt.borderB))
				intensity=0;
			else
			{
				intensity = *reinterpret_cast<unsigned char *>(inputImg->imageData + y*inputImg->widthStep+x*3+0)+
				*reinterpret_cast<unsigned char *>(inputImg->imageData + y*inputImg->widthStep+x*3+1)+
				*reinterpret_cast<unsigned char *>(inputImg->imageData + y*inputImg->widthStep+x*3+2);
				intensity/=3.0;
			}
			if (intensity<opt.blackThr)
				*(thrImg->imageData + y*thrImg->widthStep+x)=0;
			else
				*(thrImg->imageData + y*thrImg->widthStep+x)=255;
		}
	
	// Find center
	double minVar=10000000;
	double minMean=10000000;
	double x1=opt.ro/2;
	double x2=iWidth-opt.ro/2;
	double y1=opt.ro/2;
	double y2=iHeight-opt.ro/2;
	int minX=rnd(x1), minY=rnd(y1);
	double accuX=static_cast<double>(iWidth-1)/10.0;
	double accuY=static_cast<double>(iHeight-1)/10.0;
	
	while ((rnd(accuX)>=1) || (rnd(accuY)>=1))
	{
		
		for (double y=y1; y<y2; y+=accuY)
		{
			for (double x=x1; x<x2; x+=accuX)
			{
				double var,mean;
				int rndx = rnd(x);
				int rndy = rnd(y);
				evaluateCenterThreshold(opt, thrImg, rndx, rndy,  accuX/opt.ro , &var, &mean);
				if ( (var<minVar) || ((var==minVar)&&(mean<minMean)) )
				{
					minX=rndx;
					minY=rndy;
					minVar=var;
					minMean=mean;
				}
			}
		}
		x1=minX-accuX/2;
		x2=minX+accuX/2;
		y1=minY-accuY/2;
		y2=minY+accuY/2;
		if (x1<0) x1=0;
		if (y1<0) y1=0;
		if (x2>iWidth) x2=iWidth;
		if (y2>iHeight) y2=iHeight;
		accuX=(accuX-1)/2.0;
		accuY=(accuY-1)/2.0;
	}
	
	cvReleaseImage(&thrImg);
	*cx = minX;
	*cy = minY;
}


// ---------------------------------------
void estimateCenterEdges(const Options &opt, IplImage* inputImg, int *cx, int *cy)
{
	int iWidth = inputImg->width;
	int iHeight = inputImg->height;
	
	// Min and max radius durin search
	int minRadius=228;
	int maxRadius=240;
	
	// Region of interest for the center
	int centerCX = opt.edgeCX;
	int centerCY = opt.edgeCY;
	int centerDX = opt.edgeDX;
	int centerDY = opt.edgeDY;
	int centerL = centerCX-centerDX;
	int centerR = centerCX+centerDX;
	int centerT = centerCY-centerDY;
	int centerB = centerCY+centerDY;
	
	// Region of interest for all operations
	int roiL = centerCX-centerDX-maxRadius-10;
	int roiT = centerCY-centerDY-maxRadius-10;
	int roiR = centerCX+centerDX+maxRadius+10;
	int roiB = centerCY+centerDY+maxRadius+10;
	if (roiL<opt.borderL) roiL=opt.borderL;
	if (roiT<opt.borderT) roiT=opt.borderT;
	if (roiR>iWidth-1-opt.borderR) roiR=iWidth-1-opt.borderR;
	if (roiB>iHeight-1-opt.borderB) roiB=iHeight-1-opt.borderB;
	CvRect roi = cvRect(roiL, roiT, roiR-roiL+1, roiB-roiT+1);
	
	// Convert image to gray scale
	IplImage* grayImg = cvCreateImage(cvSize(iWidth, iHeight), IPL_DEPTH_32F, 1);
	for (int x=roiL; x<=roiR; ++x)
		for (int y=roiT; y<=roiB; ++y)
		{
			float intensity = *reinterpret_cast<unsigned char *>(inputImg->imageData + y*inputImg->widthStep+x*3+0)+
			*reinterpret_cast<unsigned char *>(inputImg->imageData + y*inputImg->widthStep+x*3+1)+
			*reinterpret_cast<unsigned char *>(inputImg->imageData + y*inputImg->widthStep+x*3+2);
			intensity/=3.0;
			cvSetReal2D(grayImg,y,x,logf(intensity+1));
		}
	
	// Filter the image
	cvSetImageROI(grayImg, roi);
	IplImage* filtXImg = cvCreateImage(cvSize(iWidth, iHeight), IPL_DEPTH_32F, 1);
	cvSetImageROI(filtXImg, roi);
	IplImage* filtYImg = cvCreateImage(cvSize(iWidth, iHeight), IPL_DEPTH_32F, 1);
	cvSetImageROI(filtYImg, roi);
	cvSobel(grayImg, filtXImg, 1, 0, 3);
	cvSobel(grayImg, filtYImg, 0, 1, 3);
	cvResetImageROI(filtXImg);
	cvResetImageROI(filtYImg);
	
	// Release gray image
	cvReleaseImage(&grayImg);
	
	// Get edges
	float **edgeImg = new float*[iHeight];
	for (int y=roiT; y<=roiB; ++y)
	{
		edgeImg[y] = new float[iWidth];
		float *row = edgeImg[y];
		for (int x=roiL; x<=roiR; ++x)
		{
			float eX=cvGetReal2D(filtXImg,y,x);
			float eY=cvGetReal2D(filtYImg,y,x);
			row[x] = logf(eX*eX+eY*eY+1.0);
			//if (row[x]<4) row[x]=0; else row[x]=1;
		}
	}
	
	
	
	// Release filtered images
	cvReleaseImage(&filtXImg);
	cvReleaseImage(&filtYImg);
	
	// Do Hough transform and find max
	float wInc = 1.0/maxRadius;
	double max=-1000000000;
	int maxX=0, maxY=0;
	
	for (int x=centerL; x<=centerR; ++x)
		for (int y=centerT; y<=centerB; ++y)
		{
			// Evaluate the center
			double sum=0.0;
			for (float w=0; w<2*M_PI; w+=wInc)
			{
				float tmpX=cosf(w);
				float tmpY=sinf(w);
				for (float r=minRadius; r<maxRadius; r+=1)
				{
					int iX = rndf(tmpX*r+x);
					int iY = rndf(tmpY*r+y);
					if ((iX>=roiL) && (iX<=roiR) && (iY>=roiT) && (iY<=roiB))
						sum+=edgeImg[rnd(iY)][rnd(iX)];
				}
			}
			// Find max
			if (sum>max)
			{
				max=sum;
				maxX=x;
				maxY=y;
			}
		}
	
	/*
	// Save edge image
	if (!opt.filtered.empty())
	{
		float edgeMin = 1000000000;
		float edgeMax = -1000000000;
		for (int x=roiL; x<=roiR; ++x)
			for (int y=roiT; y<=roiB; ++y)
			{
				float tmp=edgeImg[y][x];
				if (tmp>edgeMax) edgeMax=tmp;
				if (tmp<edgeMin) edgeMin=tmp;
			}
		IplImage* saveImg = cvCreateImage(cvSize(iWidth, iHeight), 8, 1);
		for (int x=roiL; x<=roiR; ++x)
			for (int y=roiT; y<=roiB; ++y)
			{
				*reinterpret_cast<unsigned char*>(saveImg->imageData + y*saveImg->widthStep+x) = 
				rnd(255.0*(edgeImg[y][x]-edgeMin)/(edgeMax-edgeMin));
			}
		// Draw center point
		int lineDiff = inputImg->width/100;
		CvScalar color=CV_RGB(255, 255, 255);
		cvLine(saveImg, cvPoint(maxX-lineDiff, maxY-lineDiff), 
				 cvPoint(maxX+lineDiff, maxY+lineDiff) , color, 1);
		cvLine(saveImg, cvPoint(maxX+lineDiff, maxY-lineDiff), 
				 cvPoint(maxX-lineDiff, maxY+lineDiff) , color, 1);
		// Draw outer radius
		cvCircle(saveImg, cvPoint(maxX, maxY), opt.ro, color, 1);
		// Save
		cvSaveImage(opt.filtered.c_str(),saveImg);
		cvReleaseImage(&saveImg);
	}
	 */
	
	// Release memory
	for (int y=roiT; y<=roiB; ++y)
		delete [] edgeImg[y];
	delete edgeImg;
	
	// Return results
	*cx=maxX;
	*cy=maxY;
}

// ---------------------------------------
void markImage(const Options &opt, IplImage* inputImg)
{
	int lineDiff = inputImg->width/100;
	CvScalar color=CV_RGB(255, 255, 0);
	
	// Draw center point
	cvLine(inputImg, cvPoint(opt.cx-lineDiff, opt.cy-lineDiff), 
			 cvPoint(opt.cx+lineDiff, opt.cy+lineDiff) , color, 2);
	cvLine(inputImg, cvPoint(opt.cx+lineDiff, opt.cy-lineDiff), 
			 cvPoint(opt.cx-lineDiff, opt.cy+lineDiff) , color, 2);
	
	// Draw inner radius
	cvCircle(inputImg, cvPoint(opt.cx, opt.cy), opt.ri, color, 2);
	
	// Draw outer radius
	cvCircle(inputImg, cvPoint(opt.cx, opt.cy), opt.ro, color, 2);
	
}


// ---------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "omni_view");
	ros::NodeHandle n;
	ros::Rate r(30);
	
	cvNamedWindow("Omni Image");
	cvNamedWindow("Unwrapped Image");
	
	
	Options opt;
	
	
	// Load image
	IplImage* inputImg = cvLoadImage("/Users/kevin/ros_sandbox/omni_view/test_images/in.jpg");
	
	if (!inputImg)
	{
		ROS_ERROR("Error: Cannot load input image file ");
		exit(-1);
	}
	
	// guess center of picture
	if (opt.cx<0) opt.cx=inputImg->width/2;
	if (opt.cy<0) opt.cy=inputImg->height/2;
	
	// estimate center
	if ((opt.estimateCenterThreshold) || (opt.estimateCenterEdge))
	{
		int newCX, newCY;
		if (opt.estimateCenterThreshold)
			estimateCenterThreshold(opt, inputImg, &newCX, &newCY);
		else if (opt.estimateCenterEdge)
			estimateCenterEdges(opt, inputImg, &newCX, &newCY);
		ROS_INFO("Estimated new center of picture (%d, %d)", newCX, newCY);
		opt.cy=newCY;
		opt.cx=newCX;
	}
	
	// Draw estimated or fixed edge of image
	if(opt.markImage) markImage(opt,inputImg);
	
	// Unwrap and save result
	IplImage* outputImg=createOutputImage(opt, inputImg);
	
	// Main Loop -- go until ^C terminates
	while (ros::ok())
	{
		
		//if (opt.unwrap) // why wouldn't this always be true?
		//{
			// Unwrap
			unwrap(opt, inputImg, outputImg);
			
		//}
		
		// make these options
		cvShowImage("Omni Image", inputImg);
		cvShowImage("Unwrapped Image", outputImg);
		
		ros::spinOnce();
		r.sleep();
	}
	
	// Release memory
	cvDestroyWindow("Omni Image");
	cvDestroyWindow("Unwrapped Image");
	
	cvReleaseImage(&outputImg);
	cvReleaseImage(&inputImg);
	
	exit(0);
}

