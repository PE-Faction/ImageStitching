
#include"JWH01.h"
#include<iostream>
void main()
{
	char *s1="C1.jpg";
	char *s2="C2.jpg";
	m_sift t;
	t.openButton_clicked(s1,s2);//��ͼƬ
	t.on_detectButton_clicked();//��������
	t.on_matchButton_clicked();//����ƥ��
	t.CalcFourCorner();//����任���ͼ2����
	t.on_mosaicButton_clicked();//ȫ��ͼ��ƴ��
	getchar();
	getchar();

}