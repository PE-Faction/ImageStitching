#include<stdlib.h>
#include<stdio.h>
#include<iostream>
#include<cv.h>
#include<highgui.h>
#include<opencv2\core\core.hpp>
using namespace cv;
using namespace std;
extern "C"
{
	#include"sift.h"
	#include"imgfeatures.h"
	#include"kdtree.h"
	#include"minpq.h"
	#include"utils.h"
	#include"xform.h"
}
class m_sift
{	
public: 
	m_sift();
	//~m_sift();
	void CalcFourCorner();//计算图2的四个角经矩阵H变换后的坐标
public:
	void openButton_clicked(char *s1,char *s2);//打开原始图像

    void on_detectButton_clicked();

    void on_matchButton_clicked();

    void on_restartButton_clicked();

    void on_mosaicButton_clicked();
	void show_Mat(CvMat *m,int row,int col);
private:
	int open_image_number;//打开图片个数
	IplImage *img1,*img2;//IplImage格式的原图
	IplImage *img1_Feat,*img2_Feat;//画上特征点之后的图

	bool verticalStackFlag;//显示匹配结果的合成图像中，两张图是纵向排列的标记
	IplImage *stacked;//显示匹配结果的合成图像，显示经距离比值法筛选后的匹配结果
	IplImage *stacked_ransac;//显示匹配结果的合成图像，显示经RANSAC算法筛选后的匹配结果

	struct feature *feat1,*feat2;//feat1:图1的特征点数组，feat2：图2的特征点数组
	int n1,n2;//n1:图1中的特征点个数 ，n2:图2中的特征点个数
	struct feature *feat;//每个特征点
	struct kd_node *kd_root;//k-d树的树根
	struct feature **nbrs;//当前特征点的最近邻点数组
	CvMat *H;//RANSAC算法求出的变换矩阵
	struct feature **inliers;//经RANSAC筛选后的内点数组
	int n_inliers;//经RANSAC算法筛选后的内点个数，即feat2中具有符合要求的特征点个数

	IplImage *xformed;//临时拼接图，即只将图2变换后的图
	IplImage *xformed_simple;//简易拼接图
	IplImage *xformed_proc;//处理后的拼接图

	//图2的四个角经矩阵H变换后的坐标
	CvPoint leftTop,leftBottom,rightTop,rightBottom;
};
