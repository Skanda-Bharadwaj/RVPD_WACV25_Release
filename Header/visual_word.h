#ifndef VISUAL_WORD_H
#define VISUAL_WORD_H

#include <general_include.h>
#include <hierarchical_cluster.h>
#include <functions.h>


class visualWords
{
public:
    float Gscore;
    float score;
    float linearityScore;
    float crossRatioScore;
    float scaleScore;
    float angleScore;

    cv::Vec4f line_para;

    vector<cv::KeyPoint> feats;
    set<int> featId;
    idNode * p_node;   // the pointer to the idNode which contains 3 or 4 features to start with

    visualWords(){
        p_node = NULL;
        Gscore = FLT_MAX;
        score = FLT_MAX;
        linearityScore = FLT_MAX;
        crossRatioScore= FLT_MAX;
        scaleScore = FLT_MAX;
        angleScore = FLT_MAX;
    }

    bool computeScore(idNode* p, vector<cv::KeyPoint> & keyPts, cv::Mat img);
    pair<float, cv::Vec4f>  computeLinearityScore( vector<cv::KeyPoint> & Pts);
    float computeCrossRatio(vector<cv::KeyPoint> & keyPts);
    float computeAngleDiff(vector<cv::KeyPoint> & keyPts);
    float computeScaleDiff(vector<cv::KeyPoint> & keyPts);
    bool directionCheck(vector<cv::KeyPoint> & keyPts);
    void expandVWs(int id, cv::KeyPoint pt, cv::Mat & img, visualWords & new_vw);
    bool scale_displacement_Check(vector<cv::KeyPoint> & keyPts);
    bool distanceCheck(vector<cv::KeyPoint> & keyPts);
    bool testLinearity(int id, cv::KeyPoint pt, float & lscore);

    // opearator =
    visualWords & operator=(const visualWords & c_vw)
    {
        Gscore = c_vw.Gscore;
        score = c_vw.score;
        p_node = c_vw.p_node;
        linearityScore = c_vw.linearityScore;
        crossRatioScore= c_vw.crossRatioScore;
        scaleScore = c_vw.scaleScore;
        angleScore = c_vw.angleScore;
        line_para = c_vw.line_para;
        featId = c_vw.featId;
        feats = c_vw.feats;
        
        return *this;
    }
    
    
};


// given current 3 features to compute the score
pair<float, cv::Vec4f> visualWords::computeLinearityScore( vector<cv::KeyPoint> & Pts)
{
    vector<cv::Point> points;
    float linearity_Score = 0.0f;
    cv::Vec4f para;

    for(auto iter = Pts.begin(); iter != Pts.end(); iter++)
    {
        points.push_back(cv::Point((*iter).pt.x, (*iter).pt.y));
    }

    cv::fitLine(points, para, cv::DIST_L2, 0, 1e-2, 1e-2);

    //std::cout << "line_para = " << line_para << std::endl;

    double k = para[1] / para[0];


    for(int i = 0 ; i< points.size(); i++)
    {
        linearity_Score +=  abs(k * (points[i].x - para[2]) + para[3] - points[i].y)/ sqrt(1+k*k);//abs(para[0]*(points[i].x - para[2]) - para[1] * (points[i].y - para[3]))
                                // sqrt(para[0] * para[0] + para[1] * para[1]);
    }

    return make_pair(linearity_Score/Pts.size(), para);
}

bool visualWords::scale_displacement_Check(vector<cv::KeyPoint> & keyPts)
{
    // sort the features
    sort(keyPts.begin(), keyPts.end(), displaceXSort);  // sort the features by scale

    float scaleThreshold = 0.5f;

    for(int i = 0 ; i < keyPts.size()-2; i++)
    {
        cv::KeyPoint A = keyPts[i];
        cv::KeyPoint B = keyPts[i+1];
        cv::KeyPoint C = keyPts[i+2];

        float dist_AB = EuclideanDistance(A,B);
        float dist_BC = EuclideanDistance(B,C);

        if( (A.size - B.size)>0 && dist_BC < dist_AB/2)
            return false;
        if( (A.size - B.size)<=0 && dist_AB < dist_BC/2)
            return false;
        if((A.size - B.size)*(B.size-C.size) < 0)
            return false;
        if((A.size - B.size) * (dist_AB - dist_BC) < 0)
            return false;
        //cout << abs((A.size / B.size) - (B.size/C.size)) <<  " " << abs((C.size / B.size) - (B.size/A.size)) << endl;
        if((A.size - B.size) > 0 && abs((A.size / B.size) - (B.size/C.size)) > scaleThreshold)
            return false;
        if((A.size - B.size) <= 0 && abs((C.size / B.size) - (B.size/A.size)) > scaleThreshold)
            return false;
    }
    return true;
}


bool visualWords::directionCheck(vector<cv::KeyPoint> & keyPts)
{
    float vectorAngleThreshold = 10.0f;

    // sort the features
    sort(keyPts.begin(), keyPts.end(), displaceXSort);  // sort the features by scale

    for(int i = 0; i < keyPts.size() - 2; i++)
    {
        cv::Vec2f v1, v2;
        v1[0] = keyPts[i].pt.x - keyPts[i+1].pt.x;
        v1[1] = keyPts[i].pt.y - keyPts[i+1].pt.y;
        v2[0] = keyPts[i+1].pt.x - keyPts[i+2].pt.x;
        v2[1] = keyPts[i+1].pt.y - keyPts[i+2].pt.y;

        cv::normalize(v1, v1, 1, cv::NORM_L2);
        cv::normalize(v2, v2, 1, cv::NORM_L2);

        float res = acos(v1.dot(v2))*180/3.1415;
        if (res > vectorAngleThreshold)
            return false;
    }
    return true;

}

float visualWords::computeCrossRatio(vector<cv::KeyPoint> & keyPts)
{
    // sort the features
    sort(keyPts.begin(), keyPts.end(), displaceXSort);  // sort the features by scale

    float min_val = FLT_MAX;
    float average = 0;
    int i = 0;
    bool flag = true;
    for(; i<keyPts.size() - 3; i++)
    {
        cv::KeyPoint A = keyPts[0];
        cv::KeyPoint B = keyPts[1];
        cv::KeyPoint C = keyPts[2];
        cv::KeyPoint D = keyPts[3];

        float dist_AC = EuclideanDistance(A, C);
        float dist_BD = EuclideanDistance(B, D);
        float dist_BC = EuclideanDistance(B, C);
        float dist_AD = EuclideanDistance(A, D);

        float res = abs(dist_AC*dist_BD / (dist_BC*dist_AD) - 4/3);

        average += res; // max(average, res);
    }
    return average/(i+1);
}

float visualWords::computeAngleDiff(vector<cv::KeyPoint> & keyPts)
{
    // sort the features
    sort(keyPts.begin(), keyPts.end(), displaceXSort);  // sort the features by scale

    float angle_diff = 0.0f;
    int i = 0;
    for(; i < keyPts.size()-2; i++)
    {
        cv::KeyPoint A = keyPts[i];
        cv::KeyPoint B = keyPts[i+1];
        cv::KeyPoint C = keyPts[i+2];

        angle_diff += abs(abs(A.angle - B.angle) - abs(B.angle - C.angle)) / 180 * 3.1415926;
    }

    return angle_diff/(i + 1);
}

float visualWords::computeScaleDiff(vector<cv::KeyPoint> & keyPts)
{
    // sort the features
    sort(keyPts.begin(), keyPts.end(), displaceXSort);  // sort the features by scale

    float scale_diff = 0.0f;
    int i = 0;
    for(; i < keyPts.size()-2; i++)
    {
        cv::KeyPoint A = keyPts[i];
        cv::KeyPoint B = keyPts[i+1];
        cv::KeyPoint C = keyPts[i+2];

        scale_diff += abs(abs(A.size/ B.size) - abs(B.size / C.size));
    }

    return scale_diff/(i + 1);
}

// compute the Score by 3 features from 3 or 4 meta group
bool visualWords::computeScore(idNode* p, vector<cv::KeyPoint> & keyPts, cv::Mat img)
{
    vector<cv::KeyPoint> pts;
    vector<int> ptsID;
    float linearityThreshold = 15.0f;
    bool flag = false;

    for(auto iter = p->idList.begin() ; iter != p->idList.end(); iter++)
    {
        pts.push_back(keyPts[*iter]);
        ptsID.push_back(*iter);
    }

    if(p->idList.size() == 3)
    {
        pair<float, cv::Vec4f> res = computeLinearityScore(pts);

        if(res.first <= linearityThreshold && directionCheck(pts)) // && scale_displacement_Check(pts))
        {
            linearityScore = res.first;
            line_para = res.second;
            p_node = p;
            featId = p->idList;
            feats = pts;
            return true;
        }
        return false;
    }

    // if there are 4 features, select the 3 features that share the  minimum linearity
    for(int i = 0; i<pts.size(); i++)
    {
        vector<cv::KeyPoint> temp(pts);
        vector<int> tempID(ptsID);
        temp.erase(temp.begin() + i);
        tempID.erase(tempID.begin() + i);

        // get the linearityScore and line para
        pair<float, cv::Vec4f> res = computeLinearityScore(temp);

        //crossRatioScore = computeCrossRatio(pts);
        //scaleScore = computeAngleDiff(pts);
        //angleScore = computeScaleDiff(pts);

        //score = exp(linearityScore) + exp(scaleScore) + exp(angleScore);

        if(linearityScore > res.first && res.first <= linearityThreshold && directionCheck(temp)) // && scale_displacement_Check(temp))
        {
            linearityScore = res.first;
            line_para = res.second;
            p_node = p;
            featId = set<int>(tempID.begin(), tempID.end());
            feats = temp;
            flag = true;
        }

    }
    if(flag)
        return true;
    else
        return false;
}

// test the linearity contraint
bool visualWords::testLinearity(int id, cv::KeyPoint pt, float & lscore)
{
    vector<int> tempID = vector<int>(featId.begin(), featId.end());
    vector<cv::KeyPoint> temp(feats);
    float linearityThreshold = 3;  // a threshold for linearity test

    // update the visual word
    temp.push_back(pt);
    tempID.push_back(id);

    pair<float, cv::Vec4f> res = computeLinearityScore(temp);

    if(res.first < linearityThreshold)
    {
        lscore = res.first;
        return true;
    }
    return false;
}

// try to expand the visual words
void visualWords::expandVWs(int id, cv::KeyPoint pt, cv::Mat & img, visualWords & new_vw)
{
    vector<int> tempID = vector<int>(featId.begin(), featId.end());
    vector<cv::KeyPoint> temp(feats);
    float linearityThreshold = 5;

    temp.push_back(pt);
    tempID.push_back(id);

    pair<float, cv::Vec4f> res = computeLinearityScore(temp);

    new_vw.line_para = res.second;
    new_vw.linearityScore = res.first;

    if(new_vw.linearityScore < linearityThreshold
            && directionCheck(temp)
            && scale_displacement_Check(temp))
    {
        new_vw.featId = set<int>(tempID.begin(), tempID.end());
        new_vw.feats = temp;
        new_vw.p_node = p_node;

        // new_vw.score = new_vw.crossRatioScore;
        //update the angleScore and scaleScore

        new_vw.angleScore = computeAngleDiff(temp);
        new_vw.scaleScore = computeScaleDiff(temp);
        new_vw.score =  new_vw.linearityScore * exp(new_vw.angleScore + new_vw.scaleScore)
                                / (0.001* new_vw.feats.size()* new_vw.feats.size());
    }

}

// define the Visual Word difference
bool vwCrossRatioSort(visualWords i, visualWords j) { return (i.crossRatioScore<j.crossRatioScore); }

bool vwScoreSort(visualWords i, visualWords j){
    if(i.feats.size() == j.feats.size())
        return (i.score<j.score);
    else
        return (i.feats.size() > j.feats.size());
}




#endif // VISUAL_WORD_H
