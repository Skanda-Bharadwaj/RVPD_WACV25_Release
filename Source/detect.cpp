
#include "../Header/general_include.h"
#include "../Header/functions.h"
#include "../Header/hierarchical_cluster.h"
#include "../Header/visual_word.h"
#include "../Header/fileManage.h"
#include "../Header/vp_Ransac.h"
#include "../Header/vp_compute.h"
#include <fstream>


float get_slope(float dx, float dy){
    return std::atan(std::abs(dy / dx)) * 180.0 / M_PI;
}


// Function to convert a line segment to the normalized vector representation
cv::Vec4f convertToNormalizedForm(const cv::Vec4f& lineSegment) {
    // Extract points
    float x1 = lineSegment[0], y1 = lineSegment[1];
    float x2 = lineSegment[2], y2 = lineSegment[3];

    // Compute the direction vector (dx, dy)
    float dx = x2 - x1;
    float dy = y2 - y1;

    // Calculate the magnitude of the direction vector
    float mag = std::sqrt(dx*dx + dy*dy);

    // Normalize the direction vector to get (vx, vy)
    float vx = dx / mag;
    float vy = dy / mag;

    // Calculate the midpoint (x0, y0) of the line segment
    float x0 = (x1 + x2) / 2.0f;
    float y0 = (y1 + y2) / 2.0f;

    // Return the normalized line representation
    return cv::Vec4f(vx, vy, x0, y0);
}

// sift feature extractor
void SIFTDetectAndCompute(const cv::Mat& img, double resizeRatio,
                            vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors, string textFilePath)
{
    // opencv 3.4
    cv::Ptr<cv::SiftFeatureDetector> detector = cv::SiftFeatureDetector::create();

    // resize the raw image to detect different levels of features
    cv::Mat resize_img = img.clone();
    try{
        resize(resize_img, resize_img, cv::Size() , resizeRatio, resizeRatio);
    }
    catch (...){
        cout << "Cannot resize the image properly. Resize ratio is: " << resizeRatio << endl;
    }

    detector->detect(resize_img, keyPoints);

    // sort the features by the size
    sort(keyPoints.begin(), keyPoints.end(), SIFTfunction);

    detector->compute(resize_img, keyPoints, descriptors);

    // convert the detected features back to original location
    for (int i = 0; i < keyPoints.size(); i++)
    {
        keyPoints[i].pt /= resizeRatio;
    }
}


// descriptor generator
void GenerateDescriptor(const cv::Mat & img, double resize_factor, int patchSize, vector<cv::Point2f>& keypoints)
{
    cv::Mat gray_img = img.clone();
    vector<cv::Rect> rects;
    vector<cv::Mat> patches; // local patch
    int W = img.size().width; int H = img.size().height;

    for(size_t i = 0 ; i < keypoints.size(); i++)
    {
        cv::Point lt = cv::Point(max(keypoints[i].x - patchSize/2, 0.0f), max(keypoints[i].y - patchSize/2, 0.0f));
        cv::Point rb = cv::Point(min(lt.x + patchSize, W), min(lt.y + patchSize, H));
        rects.push_back(cv::Rect(lt, rb));
        patches.push_back(cv::Mat(gray_img, rects[i]));
    }
}

// create the distance Matrix between features
void ComputeDistMatrix(const cv::Mat& inDes, cv::Mat& distMat, vector<cv::KeyPoint> & siftKeyPoints)
{
    cv::Mat nDes;
    inDes.convertTo(nDes, CV_32F);
    size_t rows = nDes.rows;

    // compute the distance bewteen features
    distMat = cv::Mat(cv::Size(rows,rows),CV_32F);

    for(size_t i = 0; i < rows; i++)
    {
        for(size_t j = 0; j < rows; j++)
        {
            if ( j > i)
            {
                distMat.at<float>(i, j) = norm(nDes.row(i), nDes.row(j), cv::NORM_L2);
            }
            else
            {
                if (i == j)
                    distMat.at<float>(i, j) = FLT_MAX;
                else
                    distMat.at<float>(i, j) = distMat.at<float>(j, i);
            }
        }
    }

}


idNode* CreateVW(const cv::Mat img, const cv::Mat & inDes,
                vector<cv::KeyPoint> & siftKeyPoints,
                vector<idNode*> & idNodes,
                cv::Mat & distMat, vector<tuple<float, int, int>> & dVec,
                vector<visualWords> & VWs)
{
    cout << "\nCreating visual words tree..." << endl;
    //vector<size_t> vw_idList;		// list of the ids of a visual word
    ComputeDistMatrix(inDes, distMat, siftKeyPoints);

    // Start cluster
    cv::Mat distM = distMat.clone();

    // store in a vector
    vector<tuple<float, int, int>> distVec;

    for(int i =0 ; i < distM.rows; i++)
    {
        for(int j = i+1 ; j < distM.cols; j++)
        {
            distVec.push_back(make_tuple(distM.at<float>(i,j),i,j));
        }
    }
    sort(distVec.begin(), distVec.end(),  std::greater<tuple<float, int, int>>());

    dVec.assign(distVec.begin(), distVec.end()); // copy the distVec

    // generate the node of hierarchical tree
    for(int i = 0; i < distM.rows; i++)
    {
        idNode* p = new idNode(i);
        idNodes.push_back(p);
    }

    // start iteration
    while(1)
    {
        // choose the smallest dist(i,j) in each iteration
        double minA;
        cv::Point minLoc;
        while(!distVec.empty())
        {
            minA = get<0>(*(distVec.end()-1));
            minLoc.x = get<1>(*(distVec.end()-1));
            minLoc.y = get<2>(*(distVec.end()-1));

            distVec.pop_back();

            if (EuclideanDist(siftKeyPoints[minLoc.x], siftKeyPoints[minLoc.y])
                    && angleDiff(siftKeyPoints[minLoc.x], siftKeyPoints[minLoc.y]))
                    //&& scaleDiff(siftKeyPoints[minLoc.x], siftKeyPoints[minLoc.y]) <= 10)
            {
                break;
            }
        }

        if (distVec.empty())
        {
            break;
        }

        //auto start = system_clock::now();
        idNode * left = findRoot(idNodes[minLoc.x]);
        idNode * right = findRoot(idNodes[minLoc.y]);

        if(left != right)
        {
            idNode* current_group = combine(left, right, idNodes);
            if((!left->metaflag && !right->metaflag)
                    && (current_group->idList.size() == 3 || current_group->idList.size() == 4))
            {
                visualWords cur_vws;
                if (cur_vws.computeScore( current_group, siftKeyPoints, img)){
                    VWs.push_back(cur_vws);
                    current_group->metaflag = true;
                }
            }
        }
    }

    idNode* root = findRoot(idNodes[0]);
    if(root->idList.size() != siftKeyPoints.size())
    {
        assert("something wrong in the HCtree");
    }
    cout << "Completed Creating VW tree." << endl;
    return root;
}


// expand the visual words
void visualWordsExpanding(const cv::Mat img, const cv::Mat & inDes,
                vector<cv::KeyPoint> & siftKeyPoints,
                vector<idNode*> & idNodes,
                cv::Mat & distMat, vector<tuple<float, int, int>> & dVec,
                vector<visualWords> & VWs)
{
    
    cout << "\nExpanding visual words..." << endl;

    for(auto iter = VWs.begin(); iter != VWs.end(); iter++)
    {
        cv::Mat img_c = img.clone();

        // get all the candiates and do the linearity test
        int checkedLevel = 0;
        int max_Level = 10;   // traverse the level of tree
        int max_check_points = 150;

        idNode* cur_p = iter->p_node;
        while(checkedLevel < max_Level && cur_p->parent)
        {
            cur_p = cur_p->parent;
            checkedLevel++;
            if(cur_p->parent == NULL)
                break;
        }

        vector<pair<float, int> > vwCandidates;
        for(auto j = cur_p->idList.begin(); j != cur_p->idList.end(); j++)
        {
            float lscore;
            // find all the features that meet the linearity constraint
            if(find(iter->featId.begin(), iter->featId.end(), *j) == iter->featId.end()
                    && iter->testLinearity(*j, siftKeyPoints[*j], lscore))
            {
                vwCandidates.push_back(make_pair(lscore, *j));
            }

        }

        //
        if (vwCandidates.size() >= max_check_points)
        {
            sort(vwCandidates.begin(), vwCandidates.end());
            vwCandidates.erase(vwCandidates.begin()+max_check_points, vwCandidates.end());
        }

        while(1)
        {
            // start to expand the features
            float min_score = FLT_MAX;
            int new_id = -1;
            visualWords new_vw;
            visualWords min_vw;

            for(auto j = vwCandidates.begin(); j != vwCandidates.end(); j++){
                if(find(iter->featId.begin(), iter->featId.end(), j->second) == iter->featId.end()){
                    // check if this feature can be inserted into the current VWs and get the VWs score.
                    iter->expandVWs(j->second, siftKeyPoints[j->second], img_c, new_vw);
                    if (new_vw.score < min_score){
                        min_score = new_vw.score;
                        new_id = j->second;
                        min_vw = new_vw;
                    }
                }
            }

            if(new_id != -1){
                *iter = min_vw;
            }
            else{
                break;
            }
        }

    }
    cout << "Completed expanding visual words." << endl;
}


void alignVWs(const cv::Mat img, const cv::Mat & inDes,
              vector<cv::KeyPoint> & siftKeyPoints,
              vector<idNode*> & idNodes,
              cv::Mat & distMat, vector<tuple<float, int, int>> & dVec,
              vector<visualWords> & VWs)
{
    cv::Mat img_c = img.clone();
    for(int i = 0 ; i<VWs.size(); i++){
        sort(VWs.begin() ,VWs.end(), vwScoreSort);
    }
}


//get lines from line detector
void getLines(const cv::Mat & image, vector<cv::Vec4f> & normalizedLines, float minLen)
{
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Create a Line Segment Detector
    cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();

    // Detect lines
    std::vector<cv::Vec4f> lsd_lines;
    lsd->detect(gray, lsd_lines);

    // Minimum length of lines to keep
    double minLineLength = minLen*min(image.rows, image.cols); // Adjust this value according to your needs

    // Draw lines on the image if they are longer than the minimum length
    for (size_t i = 0; i < lsd_lines.size(); ++i) {
        cv::Vec4f l = lsd_lines[i];
        double length = std::sqrt(std::pow(l[2] - l[0], 2) + std::pow(l[3] - l[1], 2));
        
        if (length >= minLineLength) {
            normalizedLines.push_back(convertToNormalizedForm(l));
            // cv::line(image, cv::Point2f(l[0], l[1]), cv::Point2f(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
    }

    // Display the result
    // cv::imshow("Filtered Lines", image);
    // cv::waitKey(0);
}


// compute the vanishing point
cv::Point2f computeVP(const cv::Mat & img, vector<visualWords> & VWs, vector<cv::Vec4f> lsd_Lines, std::vector<cv::Vec3f>& inliers, string& img_name)
{
    vector<cv::Vec4f> line_paras;
    string img_witout_ext = img_name.substr(0, img_name.find('.'));

    for(int i = 0; i < VWs.size(); i++)
    {
        // get (vx, vy), the unit vector pointing in the direction of the line
        float vx = VWs[i].line_para[0];
        float vy = VWs[i].line_para[1];

        // filter out the vertical and horizontal lines
        if (vx != 0 && vy != 0){
            float slope = get_slope(vx, vy);
            if(slope < 88 && slope > 2)
                line_paras.push_back(VWs[i].line_para);
        }
    }

    for (const auto& line : lsd_Lines) {
        // get (vx, vy), the unit vector pointing in the direction of the line
        float vx = line[0];
        float vy = line[1];

        // filter out the vertical and horizontal lines
        if (vx != 0 && vy != 0){
            float slope = get_slope(vx, vy);
            if(slope < 88 && slope > 2)
                line_paras.push_back(line);
        }
    }
        
    // to get the inliers and compute the vanishing Point
    VP_ransac solver(line_paras, inliers, img, img_name);
    std::cout << "Completed Weighted RANSAC.\n" << std::endl;

    if (inliers.empty())
        return cv::Point2f(-1, -1);

    cv::Point2f vp = compute_VP_candidate(inliers);
    vp.x /= resizeRatio;
    vp.y /= resizeRatio;

    return vp;
}


// main function
int main(int argc, char** argv){

    // get all the files under the directory
    vector<string> listofdir;
    
    string rootPath = argv[0];
    rootPath = rootPath.substr(0, rootPath.find("build"));

    string vwsFilePath = rootPath;
    
    string img_path     = rootPath + "images/";
    string resultPath   = rootPath + "results/";
    string vp_path      = rootPath + "results/";
    string vp_anno_path = rootPath + "results/anno/";
    string vp_img_path  = rootPath + "results/imgs/";


    // Create the directory if it doesn't exist
    if (!boost::filesystem::exists(resultPath))
        boost::filesystem::create_directory(resultPath);
    if (!boost::filesystem::exists(vp_anno_path))
        boost::filesystem::create_directory(vp_anno_path);
    if(!boost::filesystem::exists(vp_img_path))
        boost::filesystem::create_directory(vp_img_path);

    std::cout << "\nRVPD: Reccurence-based Vanishing Point Detection" << std::endl;
    std::cout << "Skanda Bharadwaj, Robert T. Collins and Yanxi Liu, \"Recurrence-based Vanishing Point Detection\", WACV 2025\n" << std::endl;

    read_subdir(listofdir, img_path);
    cout << "Total # of file: " << listofdir.size() << endl;

    try{
        for(int i = 0; i < listofdir.size(); i++)
        {
            cv::Mat image;
            string img_name = listofdir[i];

            //***********************************************
            // Just for now, remove this
            string outfilepath = vp_img_path + img_name;
            bool exists = boost::filesystem::exists(outfilepath);
            if(exists)
                continue;
            //***********************************************

            string img_fullpath = img_path + img_name;

            resizeRatio = 1.0f;

            cout << "################################################" << endl;
            cout << "\n#" << i + 1 << ". Image: " << img_name << endl;

            image = cv::imread(img_fullpath);
            if(image.data== nullptr){
                cerr<<"image doesnt exist!"<<endl;
                return 0;
            }

            // normalize the volumn of image
            float volumn = 800*800;
            resizeRatio = sqrt(volumn / image.rows / image.cols);
            resize(image, image, cv::Size(), resizeRatio, resizeRatio);

            // sift feature and destriptors.
            vector<cv::KeyPoint> siftKeyPoints;
            cv::Mat siftDescriptors;
            SIFTDetectAndCompute(image, 1.0, siftKeyPoints, siftDescriptors, vwsFilePath);


            if (siftKeyPoints.size()){
                vector<idNode*> idNodes;  // node sift id
                cv::Mat distMat;  // distance matrix
                vector<tuple<float, int, int>> dVec; // distance matrix
                vector<visualWords> VWs;
                idNode* root = CreateVW(image, siftDescriptors, siftKeyPoints, idNodes, distMat, dVec, VWs);


                // start to expand the visual word and get the score for each visual words
                visualWordsExpanding(image, siftDescriptors, siftKeyPoints, idNodes, distMat, dVec, VWs);


                // align the visual word
                alignVWs(image, siftDescriptors, siftKeyPoints, idNodes, distMat, dVec, VWs);


                std::vector<cv::Vec4f> lsd_Lines;
                float minLen = 0.25;
                getLines(image, lsd_Lines, minLen);
                

                // compute the VP by the visual words
                std::vector<cv::Vec3f> inliers;
                cv::Point2f vp = computeVP(image, VWs, lsd_Lines, inliers, img_name);
                
                // Print VP
                cout << "Predicted Vanishing Point: " << vp << "\n" << endl;

                //free HC tree
                freeHCTree(root);


                // save the vanishing point
                string imgPath = vp_img_path + img_name;
                vp_imshow(image, inliers, vp, resizeRatio, imgPath);

                string img_name_wo_ext = img_name.substr(0, img_name.find('.'));
                string annoPath = vp_anno_path + img_name_wo_ext + ".txt";

                // save the vanishing point
                ofstream myfile;
                myfile.open(annoPath);
                myfile << vp.x << " "
                        << vp.y << endl;
                myfile.close();
            }

            else{
                cout << "No Siftkeypoints found in the image." << endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return 0;
}
