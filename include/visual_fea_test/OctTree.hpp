
#ifndef OCTTREE_H
#define OCTTREE_H


#include <vector>
#include <opencv2/core/core.hpp>
#include <list>

namespace Oct
{
//四叉树 节点 
class ExtractorNode{
public:
    ExtractorNode():bNoMore(false){}
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL,UR,BL,BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

std::vector<cv::KeyPoint> OctTree(const std::vector<cv::KeyPoint>& keypoints, const int &height,const int &width,
                                            const int &N);
}
#endif