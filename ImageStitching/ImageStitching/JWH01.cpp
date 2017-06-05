#include "JWH01.h"

//��k-d���Ͻ���BBF����������
#define KDTREE_BBF_MAX_NN_CHKS 200
//Ŀ���������ںʹν��ڵľ���ı�ֵ����ֵ�������ڴ���ֵ�����޳���ƥ����
//ͨ����ֵȡ0.6��ֵԽС�ҵ���ƥ����Խ��ȷ����ƥ����ĿԽ��
#define NN_SQ_DIST_RATIO_THR 0.49
#define IMG1 "ͼ1"
#define IMG2 "ͼ2"
#define IMG1_FEAT "ͼ1������"
#define IMG2_FEAT "ͼ2������"
#define IMG_MATCH1 "�����ֵɸѡ���ƥ����"
#define IMG_MATCH2 "RANSACɸѡ���ƥ����"
#define IMG_MOSAIC_TEMP "��ʱƴ��ͼ��"
#define IMG_MOSAIC_SIMPLE "����ƴ��ͼ"
#define IMG_MOSAIC_BEFORE_FUSION "�ص������ں�ǰ"
#define IMG_MOSAIC_PROC "������ƴ��ͼ"
m_sift::m_sift()
{
	img1 = NULL;
	img2 = NULL;
	img1_Feat = NULL;
	img2_Feat = NULL;
    stacked = NULL;
    stacked_ransac = NULL;
    H = NULL;
    xformed = NULL;
	verticalStackFlag = false;//��ʾƥ�����ĺϳ�ͼ��Ĭ���Ǻ�������
}

//ͼƬ��
void m_sift::openButton_clicked(char * s1,char *s2)
{
	img1 = cvLoadImage(s1,1);
	img2 = cvLoadImage(s2,1);
	cvNamedWindow(IMG2_FEAT);//��������
 
}

//��������
void m_sift::on_detectButton_clicked()
{
	img1_Feat = cvCloneImage(img1);
	img2_Feat = cvCloneImage(img2);
	/********************************************/
	/*
			SIFT���������ȡ
	*/
	/********************************************/
	n1 = sift_features(img1,&feat1);
	export_features("feature1.txt",feat1,n1);
	//ͼƬ��ʾ�׶�
	draw_features( img1_Feat, feat1, n1 );//����������
   
	//cvWaitKey(0);
	 //��ȡ����ʾ��2��ͼƬ�ϵ�������
    n2 = sift_features( img2, &feat2 );//���ͼ2�е�SIFT�����㣬n2��ͼ2�����������
    export_features("feature2.txt",feat2,n2);//��������������д�뵽�ļ�
    draw_features( img2_Feat, feat2, n2 );//����������
	/*********************ͼƬ��ʾ************************/
	/****************************************************/
	//cvNamedWindow(IMG1_FEAT);//��������
 //   cvShowImage(IMG1_FEAT,img1_Feat);//��ʾ
 //   cvNamedWindow(IMG2_FEAT);//��������
 //   cvShowImage(IMG2_FEAT,img2_Feat);//��ʾ
	//cvWaitKey(0);
	//********************************************//
	/*��Ҫע��ͼƬ��ָ�뻹û���ͷ�*/

}

//����ƥ��
void m_sift::on_matchButton_clicked()
{
	//����ͼ����ˮƽ����
	stacked = stack_imgs_horizontal(img1,img2);
	//����ͼ1�������㼯feat1����k-d��������k-d����ָ��
	kd_root = kdtree_build( feat1,n1);
	Point pt1,pt2;//���ߵ������˵�
	double d0,d1;//feat2��ÿ�������㵽����ںʹν��ڵľ���
	int matchNum = 0;//�������ֵ��ɸѡ���ƥ���ĸ���
	//���������㼯feat2�����feat2��ÿ��������feat��ѡȡ���Ͼ����ֵ������ƥ��㣬�ŵ�feat��fwd_match����
	for(int i = 0;i<n2;i++)
	{
		feat = feat2+i;//��i���������ָ��
		//��kd_root������Ŀ���feata��2������ڵ㣬�����nbrs�У�����ʵ���ҵ��Ľ��ڵ�ĸ���
		int k = kdtree_bbf_knn( kd_root, feat,2,&nbrs,KDTREE_BBF_MAX_NN_CHKS);
		if(k ==2)
		{
			d0 = descr_dist_sq(feat ,nbrs[0]);//feat������ڵ�ľ����ƽ��
			d1 = descr_dist_sq(feat ,nbrs[1]);//feat��ν��ڵ�ľ����ƽ��
			//��d0��d1�ı�ֵС����ֵNN_SQ_DIAT_RATIO_THR,����ܴ�ƥ�䣬�����޳�
			if(d0< d1*NN_SQ_DIST_RATIO_THR)
			{
				//��Ŀ���feat������ڵ���Ϊƥ����
				pt2 = Point(cvRound( feat->x),cvRound(feat->y));//ͼ2�е������
				pt1 = Point(cvRound( nbrs[0]->x),cvRound( nbrs[0]->y));//ͼ1�е������
				//ͼƬ������ˮƽ����
				pt2.x += img1->width;//��������ͼ���������׵ģ�pt2�ĺ��������ͼ1�Ŀ�ȣ���Ϊ���ߵ��յ�
				cvLine( stacked,pt1,pt2,CV_RGB(255,0,255),1,8,0);//��������
				matchNum++;//ͳ��ƥ���Եĸ���
				feat2[i].fwd_match = nbrs[0];//ʹ��feat��fwd_match��ָ�����Ӧ��ƥ���
			}
		}
		free( nbrs);//�ͷŽ�������
	}
	/*****************ͼƬ��ʾ*****************************/
	cvNamedWindow(IMG_MATCH1);//��������
	cvShowImage(IMG_MATCH1,stacked);//��ʾ
	
	//����RANSAC�㷨ɸѡƥ��㣬����任����H
	//����img1��img2������˳��H��Զ�ǽ�feat2�е�������任Ϊ��ƥ��㣬����img2�еĵ�任Ϊimg2�еĶ�Ӧ��
	H = ransac_xform(feat2,n2,FEATURE_FWD_MATCH,lsq_homog,4,0.01,homog_xfer_err,3.0,&inliers,&n_inliers);
	//Ŀ����Ϊ���������
	for(int i=0;i<H->rows;i++)
	{
		for(int j=0;j<H->cols;j++)
		{
			printf("%f ",cvmGet( H, i, j ) );
		}
		printf("\n");
	}
	//���ܳɹ�������任���󣬼�����ͼ���й�ͬ����
	if(H)
	{
		//ͼƬʱˮƽ��ʽ
		stacked_ransac = stack_imgs_horizontal(img1,img2);//�ϳ�ͼ��
		int invertNum = 0;//ͳ��pt2.x > pt1.x��ƥ���Եĸ��������ж�img1���Ƿ���ͼ
		//������RANSAC�㷨ɸѡ��������㼯��inliers,�ҵ�ÿ���������ƥ���,��������
		for(int i=0;i<n_inliers;i++)
		{
			feat = inliers[i];//��i��������
			pt2 = Point(cvRound(feat->x),cvRound(feat->y));//ͼ2�е������
			pt1 = Point(cvRound(feat->fwd_match->x),cvRound(feat->fwd_match->y));//ͼ1�е������
			//ͳ��ƥ��������λ�ù�ϵ�����ж�ͼ1��ͼ2������λ�ù�ϵ
			if(pt2.x >pt1.x)
				invertNum++;
			//ˮƽ����
			pt2.x += img1->width;//��������ͼ���������еģ�pt2�ĺ��������ͼ1�Ŀ�ȣ���Ϊ���ߵ��յ�
			cvLine(stacked_ransac,pt1,pt2,CV_RGB(255,0,255),1,8,0);//��ƥ��ͼ�ϻ�������
		}
	cvNamedWindow(IMG_MATCH2);//��������
	cvShowImage(IMG_MATCH2,stacked_ransac);//��ʾ��RANSAC�㷨ɸѡ���ƥ��ͼ
	
	/*
		�����м�����ı任����H������img2�еĵ�任Ϊimg1�еĵ㣬���������img1Ӧ������ͼ��img2Ӧ������ͼ.
		��ʱimg2�еĵ�pt2��img1�еĶ�Ӧ��pt1��x����Ĺ�ϵ�������ǣ�pt2.x<pt1.x
		���û��򿪵�img1����ͼ��img2����ͼ����img2�еĵ�pt2��img1�еĶ�Ӧ��pt1��x����Ĺ�ϵ�������ǣ�pt2.x>pt1.x
		����ͨ��ͳ�ƶ�Ӧ��任ǰ��x�����С��ϵ������֪��img1�ǲ�����ͼ��
		���img1����ͼ����img1�е�ƥ��㾭H������H_IVT�任��ɵõ�img2�е�ƥ���
	*/
	//��pt2.x>pt1.x�ĵ�ĸ��������ڵ������80%�����϶�img1������ͼ
	if(invertNum > n_inliers*0.8)
	{
		CvMat *H_IVT = cvCreateMat(3,3,CV_64FC1);//�任����������
		//��H������H_IVTʱ�����ɹ���������ط���ֵ
		if(cvInvert(H,H_IVT))
		{
			cvReleaseMat(&H);//�ͷű任����H����Ϊ�ò�����
			H = cvCloneMat(H_IVT);//��H������H_IVT�е����ݿ���H��
			cvReleaseMat(&H_IVT);//�ͷ�����H_IVT
			//��img1��img2�Ե�
			IplImage *temp = img2;
			img2 = img1;
			img1 = temp;
		}
		else
		{
			cvReleaseMat(&H_IVT);//�ͷ�����H_IVT
			printf("�任����H������");
		
		}
	}
	}
	else
	{
		printf("�����޹�������");
	}
}
//����ͼ2���ĸ��Ǿ�����H�任�������
void m_sift::CalcFourCorner()
{
	//����ͼ2���ĸ��Ǿ�����任�������
	double v2[] = {0,0,1};//���Ͻ�
	double v1[3];//�任�������ֵ
	CvMat V2 = cvMat(3,1,CV_64FC1,v2);
	show_Mat(&V2,3,1);
	CvMat V1 = cvMat(3,1,CV_64FC1,v1);
	cvGEMM(H,&V2,1,0,1,&V1);//����˷�
	show_Mat(&V1,3,1);
	leftTop.x = cvRound(v1[0]/v1[2]);
	leftTop.y = cvRound(v1[1]/v1[2]);
	//��V2��������Ϊ���½�����
	v2[0] = 0;
	v2[1] = img2->height;
	V2 = cvMat(3,1,CV_64FC1,v2);
	V1 = cvMat(3,1,CV_64FC1,v1);
	cvGEMM(H,&V2,1,0,1,&V1);
	leftBottom.x = cvRound(v1[0]/v1[2]);
	leftBottom.y = cvRound(v1[1]/v1[2]);

	//��v2��������Ϊ���Ͻ�����
    v2[0] = img2->width;
    v2[1] = 0;
    V2 = cvMat(3,1,CV_64FC1,v2);
    V1 = cvMat(3,1,CV_64FC1,v1);
    cvGEMM(H,&V2,1,0,1,&V1);
    rightTop.x = cvRound(v1[0]/v1[2]);
    rightTop.y = cvRound(v1[1]/v1[2]);

    //��v2��������Ϊ���½�����
    v2[0] = img2->width;
    v2[1] = img2->height;
    V2 = cvMat(3,1,CV_64FC1,v2);
    V1 = cvMat(3,1,CV_64FC1,v1);
    cvGEMM(H,&V2,1,0,1,&V1);
    rightBottom.x = cvRound(v1[0]/v1[2]);
    rightBottom.y = cvRound(v1[1]/v1[2]);
    

}
//ȫ��ƴ��
void m_sift::on_mosaicButton_clicked()
{
	//���ܳɹ�������任���󣬼�����ͼ�����й�ͬ���򣬾Ϳ��Խ���ȫ��ƴ����
	if(H)
	{
		//ƴ��ͼ��img1����ͼ��img2����ͼ
		CalcFourCorner();//����ͼ2���ĸ��Ǿ��任�������
		//Ϊƴ�ӽ��ͼxformed����ռ�,�߶�Ϊͼ1ͼ2�߶ȵĽ�С�ߣ�����ͼ2���ϽǺ����½Ǳ任��ĵ��λ�þ���ƴ��ͼ�Ŀ��
		xformed = cvCreateImage(cvSize(MIN(rightTop.x,rightBottom.x),MIN(img1->height,img2->height)),IPL_DEPTH_8U,3);
	//�ñ任����H����ͼimg2��ͶӰ�任���任������������ƣ�������ŵ�xformed��
		cvWarpPerspective(img2,xformed,H,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
		cvNamedWindow(IMG_MOSAIC_TEMP); //��ʾ��ʱͼ,��ֻ��ͼ2�任���ͼ
        cvShowImage(IMG_MOSAIC_TEMP,xformed);


		//����ƴ�ӷ���ֱ�ӽ���ͼimg1���ӵ�xformed�����
		xformed_simple = cvCloneImage(xformed);//����ƴ��ͼ����¡xformed
		cvSetImageROI(xformed_simple,cvRect(0,0,img1->width,img1->height));
		cvNamedWindow("lgss"); //��ʾ��ʱͼ,��ֻ��ͼ2�任���ͼ
        cvShowImage("lgss",img1);
	
		//for(int i=0;i<img1->height;i++)
		//{
		//	uchar *s=(uchar *)(img1->imageData+img1->widthStep*i);
		//	for(int j=0;j<img1->width;j++)
		//	{
		//		s[j*3+1] = 0;
		//		s[j*3+2] =0;
		//		s[j*3]=0;
		//	
		//	}
		//
		//}
		//cvNamedWindow("lgs"); //��ʾ��ʱͼ,��ֻ��ͼ2�任���ͼ
  //      cvShowImage("lgs",img1);
		cvAddWeighted(img1,1,xformed_simple,0,0,xformed_simple);//���ͼƬ1
		cvResetImageROI(xformed_simple);
		cvNamedWindow(IMG_MOSAIC_SIMPLE);//��������
        cvShowImage(IMG_MOSAIC_SIMPLE,xformed_simple);//��ʾ����ƴ��ͼ
		cvWaitKey(0);
		//������ƴ��ͼ����¡��xformed
		xformed_proc = cvCloneImage(xformed);

		//�ص�������ߵĲ�����ȫȡ��ͼ1
        cvSetImageROI(img1,cvRect(0,0,MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(xformed,cvRect(0,0,MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(xformed_proc,cvRect(0,0,MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvAddWeighted(img1,1,xformed,0,0,xformed_proc);
        cvResetImageROI(img1);
        cvResetImageROI(xformed);
        cvResetImageROI(xformed_proc);
        cvNamedWindow(IMG_MOSAIC_BEFORE_FUSION);
        cvShowImage(IMG_MOSAIC_BEFORE_FUSION,xformed_proc);//��ʾ�ں�֮ǰ��ƴ��ͼ
	//	cvWaitKey(0);

        //���ü�Ȩƽ���ķ����ں��ص�����
		int start = MIN(leftTop.x,leftBottom.x);//��ʼλ�ã����ص��������߽�
		double processWidth = img1->width-start;//�ص�����Ŀ��
		double alpha = 1;//img1�����ص�Ȩ��
		for(int i= 0;i<xformed_proc->height;i++)//������
		{
			const uchar * pixel_img1 = ((uchar *) (img1->imageData + img1->widthStep *i));//img1 �е�i�����ݵ�ָ��
			const uchar * pixel_xformed = ((uchar *)(xformed->imageData + xformed->widthStep * i));//xformed�е�i�����ݵ�ָ��
            uchar * pixel_xformed_proc = ((uchar *)(xformed_proc->imageData + xformed_proc->widthStep * i));//xformed_proc�е�i�����ݵ�ָ��
			for(int j=start; j<img1->width;j++)//�����ص��������
			{
				//�������ͼ��xformed�������صĺڵ㣬����ȫ����ͼƬ1�е�����
				if(pixel_xformed[j*3]<50&& pixel_xformed[j*3+1]<50&&pixel_xformed[j*3+2] < 50 )
					alpha=1;
				else
				{
					 //img1�����ص�Ȩ�أ��뵱ǰ�������ص�������߽�ľ��������
                    alpha = (processWidth-(j-start)) / processWidth ;
					}
				pixel_xformed_proc[j*3] = pixel_img1[j*3] * alpha + pixel_xformed[j*3] * (1-alpha);//Bͨ��
                pixel_xformed_proc[j*3+1] = pixel_img1[j*3+1] * alpha + pixel_xformed[j*3+1] * (1-alpha);//Gͨ��
                pixel_xformed_proc[j*3+2] = pixel_img1[j*3+2] * alpha + pixel_xformed[j*3+2] * (1-alpha);//Rͨ��
  
			}
		
		}
		cvNamedWindow(IMG_MOSAIC_PROC);//��������
        cvShowImage(IMG_MOSAIC_PROC,xformed_proc);//��ʾ������ƴ��ͼ
		cvWaitKey(0);
		/**********************************************************/
		/*
			��������������������ʽ
		*/
  /*�ص�����ȡ����ͼ���ƽ��ֵ��Ч������
        //����ROI���ǰ����ص�����ľ���
        cvSetImageROI(xformed_proc,cvRect(MIN(leftTop.x,leftBottom.x),0,img1->width-MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(img1,cvRect(MIN(leftTop.x,leftBottom.x),0,img1->width-MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(xformed,cvRect(MIN(leftTop.x,leftBottom.x),0,img1->width-MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvAddWeighted(img1,0.5,xformed,0.5,0,xformed_proc);
        cvResetImageROI(xformed_proc);
        cvResetImageROI(img1);
        cvResetImageROI(xformed); */

        /*��ƴ�ӷ���Χ��������˲�������ƴ�ӷ죬Ч������
        //�ڴ���ǰ���ͼ�Ϸֱ����ú��ƴ�ӷ�ľ���ROI
        cvSetImageROI(xformed_proc,cvRect(img1->width-10,0,img1->width+10,xformed->height));
        cvSetImageROI(xformed,cvRect(img1->width-10,0,img1->width+10,xformed->height));
        cvSmooth(xformed,xformed_proc,CV_MEDIAN,5);//��ƴ�ӷ���Χ���������ֵ�˲�
        cvResetImageROI(xformed);
        cvResetImageROI(xformed_proc);
        cvShowImage(IMG_MOSAIC_PROC,xformed_proc);//��ʾ������ƴ��ͼ */

        /*��ͨ���񻯽���任���ͼ��ʧ������⣬����Ť�������ͼ��Ч������
        double a[]={  0, -1,  0, -1,  5, -1, 0, -1,  0  };//������˹�˲��˵�����
        CvMat kernel = cvMat(3,3,CV_64FC1,a);//������˹�˲���
        cvFilter2D(xformed_proc,xformed_proc,&kernel);//�˲�
        cvShowImage(IMG_MOSAIC_PROC,xformed_proc);//��ʾ������ƴ��ͼ*/


        
	}

}

// ����ѡ��
void m_sift::on_restartButton_clicked()
{
    //�ͷŲ��ر�ԭͼ1
    if(img1)
    {
        cvReleaseImage(&img1);
        cvDestroyWindow(IMG1);
    }
    //�ͷŲ��ر�ԭͼ2
    if(img2)
    {
        cvReleaseImage(&img2);
        cvDestroyWindow(IMG2);
    }
    //�ͷ�������ͼ1
    if(img1_Feat)
    {
        cvReleaseImage(&img1_Feat);
        cvDestroyWindow(IMG1_FEAT);
        free(feat1);//�ͷ�����������
    }
    //�ͷ�������ͼ2
    if(img2_Feat)
    {
        cvReleaseImage(&img2_Feat);
        cvDestroyWindow(IMG2_FEAT);
        free(feat2);//�ͷ�����������
    }
    //�ͷž����ֵɸѡ���ƥ��ͼ��kd��
    if(stacked)
    {
        cvReleaseImage(&stacked);
        cvDestroyWindow(IMG_MATCH1);
        kdtree_release(kd_root);//�ͷ�kd��
    }
    //ֻ����RANSAC�㷨�ɹ�����任����ʱ������Ҫ��һ���ͷ�������ڴ�ռ�
    if( H )
    {
        cvReleaseMat(&H);//�ͷű任����H
        free(inliers);//�ͷ��ڵ�����

        //�ͷ�RANSAC�㷨ɸѡ���ƥ��ͼ
        cvReleaseImage(&stacked_ransac);
        cvDestroyWindow(IMG_MATCH2);

        //�ͷ�ȫ��ƴ��ͼ��
        if(xformed)
        {
            cvReleaseImage(&xformed);
            cvReleaseImage(&xformed_simple);
            cvReleaseImage(&xformed_proc);
            cvDestroyWindow(IMG_MOSAIC_TEMP);
            cvDestroyWindow(IMG_MOSAIC_SIMPLE);
            cvDestroyWindow(IMG_MOSAIC_BEFORE_FUSION);
            cvDestroyWindow(IMG_MOSAIC_PROC);
        }
    }

    open_image_number = 0;//��ͼƬ��������
    verticalStackFlag = false;//��ʾƥ�����ĺϳ�ͼƬ�����з����ʶ��λ

   
}
void m_sift::show_Mat(CvMat *m,int row,int col)
{

		for(int i=0;i<row;i++)
	{
		for(int j=0;j<col;j++)
		{
			printf("%f ",cvmGet( m, i, j ) );
		}
		printf("\n");
	}


}