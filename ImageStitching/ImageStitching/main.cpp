
#include"JWH01.h"
#include<iostream>
void main()
{
	char *s1="C1.jpg";
	char *s2="C2.jpg";
	m_sift t;
	t.openButton_clicked(s1,s2);//打开图片
	t.on_detectButton_clicked();//特征点检测
	t.on_matchButton_clicked();//特征匹配
	t.CalcFourCorner();//计算变换后的图2坐标
	t.on_mosaicButton_clicked();//全景图像拼接
	getchar();
	getchar();

}