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
	void CalcFourCorner();//����ͼ2���ĸ��Ǿ�����H�任�������
public:
	void openButton_clicked(char *s1,char *s2);//��ԭʼͼ��

    void on_detectButton_clicked();

    void on_matchButton_clicked();

    void on_restartButton_clicked();

    void on_mosaicButton_clicked();
	void show_Mat(CvMat *m,int row,int col);
private:
	int open_image_number;//��ͼƬ����
	IplImage *img1,*img2;//IplImage��ʽ��ԭͼ
	IplImage *img1_Feat,*img2_Feat;//����������֮���ͼ

	bool verticalStackFlag;//��ʾƥ�����ĺϳ�ͼ���У�����ͼ���������еı��
	IplImage *stacked;//��ʾƥ�����ĺϳ�ͼ����ʾ�������ֵ��ɸѡ���ƥ����
	IplImage *stacked_ransac;//��ʾƥ�����ĺϳ�ͼ����ʾ��RANSAC�㷨ɸѡ���ƥ����

	struct feature *feat1,*feat2;//feat1:ͼ1�����������飬feat2��ͼ2������������
	int n1,n2;//n1:ͼ1�е���������� ��n2:ͼ2�е����������
	struct feature *feat;//ÿ��������
	struct kd_node *kd_root;//k-d��������
	struct feature **nbrs;//��ǰ�����������ڵ�����
	CvMat *H;//RANSAC�㷨����ı任����
	struct feature **inliers;//��RANSACɸѡ����ڵ�����
	int n_inliers;//��RANSAC�㷨ɸѡ����ڵ��������feat2�о��з���Ҫ������������

	IplImage *xformed;//��ʱƴ��ͼ����ֻ��ͼ2�任���ͼ
	IplImage *xformed_simple;//����ƴ��ͼ
	IplImage *xformed_proc;//������ƴ��ͼ

	//ͼ2���ĸ��Ǿ�����H�任�������
	CvPoint leftTop,leftBottom,rightTop,rightBottom;
};
