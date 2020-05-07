
#include <visual_fea_test/OctTree.hpp>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <list>
#include <iterator>

using namespace std;

namespace Oct
{
void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
{
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    for(size_t i=0;i<vKeys.size();i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}

std::vector<cv::KeyPoint> OctTree(const std::vector<cv::KeyPoint>& keypoints, const int &height,const int &width,
                                            const int &N)
{
    //四叉树构建特征
    //先计算长宽比，初始化四叉数节点[默认图像宽度大于长度]

    const int EDGE_THRESHOLD = 16;

    const int nIni = round(static_cast<float>(width/height));
    const float hX = static_cast<float>(width)/nIni;

    list<ExtractorNode> lNodes;
    
    // 记录初始节点的指针
    vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

    // step1. 建立分裂的初始节点
    for (int t = 0; t < nIni; t++)
    {
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX*static_cast<float>(t),0);
        ni.UR = cv::Point2i(hX*static_cast<float>(t+1),0);
        ni.BL = cv::Point2i(ni.UL.x,height - EDGE_THRESHOLD);
        ni.BR = cv::Point2i(ni.UR.x,height - EDGE_THRESHOLD);
        ni.vKeys.reserve(keypoints.size());

        lNodes.push_back(ni);
        vpIniNodes[t] = &lNodes.back();
    }
    
    //  ROS_INFO("step2,:key size= %d",keypoints_l.size());
    // step2. 将所有特征点关联到对应的节点区域

    for (size_t t = 0; t < keypoints.size(); t++)
    {
        const cv::KeyPoint &kp = keypoints[t];
        if (kp.pt.x < width -EDGE_THRESHOLD)
            vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }
    

    list<ExtractorNode>::iterator lit = lNodes.begin();

    while (lit != lNodes.end())
    {
        if (lit->vKeys.size()==1)
        {
            lit->bNoMore = true;
            lit++;
        }
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;
    int iteration = 0;
    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    // 四叉树进行区域划分
    while(!bFinish){
        iteration++;
        int preSize = lNodes.size();
        lit = lNodes.begin();
        int nToExpand = 0;
        vSizeAndPointerToNode.clear();

        //广度搜索 
        while(lit!=lNodes.end()){
            if (lit->bNoMore)
            {
                lit++;
                continue;
            }
            else{
                //If more than one point,subdivide
                ExtractorNode n1,n2,n3,n4;
                lit->DivideNode(n1,n2,n3,n4);

                //Add childs if they contain points
                if(n1.vKeys.size()>0)
                {
                    // note：将新分裂出的节点插入到容器前面，迭代器后面的都是上一次分裂还未访问的节点
                    lNodes.push_front(n1);
                    // 如果该节点中包含的特征点超过1，则该节点将会继续扩展子节点，使用nToExpand统计接下来要扩展的节点数
                    if(n1.vKeys.size()>1)
                    {
                        nToExpand++;
                        // 按照 pair<节点中特征点个数，节点索引> 建立索引，后续通过排序快速筛选出包含特征点个数比较多的节点
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                        // 记录节点自己的迭代器指针
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit = lNodes.erase(lit);
                continue;

            }
        }

        //step3. 接近目标目标，有限对特征点多的部分进行划分
        if ((int)lNodes.size()>=N || (int)lNodes.size()==preSize)
        {
            bFinish = true;
        }
        else if(((int)lNodes.size() + nToExpand*3)>N){
            while(!bFinish){
                preSize = lNodes.size();

                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                for (int j = vPrevSizeAndPointerToNode.size()-1; j >= 0 ; j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }                                    

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if ((int)lNodes.size()>=N)
                    {
                        break;
                    }   
                }
                if ((int)lNodes.size()>=N || (int)lNodes.size()==preSize)
                {
                    bFinish = true;
                }
                
            }
        }
    }
    //step4. 保留响应值最大的点
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(N);
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            if(vNodeKeys[k].response>maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }
    return vResultKeys;
}
}