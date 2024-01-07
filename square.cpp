#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
typedef pcl::PointXYZI Pointtype;

void FirstDenseThenMagic()
{
    // Step 1: 创建一个PointCloud对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Step 2: 加载点云数据
    pcl::io::loadPCDFile("/home/liuhongwei/下载/liosam/GlobalMap.pcd", *cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        // 寻找最近邻点
        if (kdtree->nearestKSearch(cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            // 在原始点和其最近邻点之间插入一个点
            pcl::PointXYZ newPoint;
            newPoint.x = (cloud->points[i].x + cloud->points[pointIdxNKNSearch[1]].x) / 2.0;
            newPoint.y = (cloud->points[i].y + cloud->points[pointIdxNKNSearch[1]].y) / 2.0;
            newPoint.z = (cloud->points[i].z + cloud->points[pointIdxNKNSearch[1]].z) / 2.0;
            outputCloud->push_back(newPoint);
        }
    }
    *outputCloud += *cloud;


    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    // Step 4: 设置参数  0.1
    mls.setInputCloud(outputCloud);
    mls.setSearchRadius(0.09); // 设置搜索半径，根据需要调整

    // Step 5: 进行插值
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>());
    mls.process(*mls_points);

    // Step 6: 将点云类型从 pcl::PointNormal 转换为 pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*mls_points, *mls_cloud);

    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/1.pcd", *mls_cloud);

}


void FirstDealThenInter()
{
    // Step 1: 创建一个PointCloud对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Step 2: 加载点云数据
    pcl::io::loadPCDFile("/home/liuhongwei/下载/liosam/GlobalMap.pcd", *cloud);

    // Step 3: 创建 MovingLeastSquares 对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    // Step 4: 设置参数  0.1
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.09); // 设置搜索半径，根据需要调整

    // Step 5: 进行插值
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>());
    mls.process(*mls_points);

    // Step 6: 将点云类型从 pcl::PointNormal 转换为 pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*mls_points, *mls_cloud);

    // Step 7: 保存密度增强后的点云
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/1.pcd", *mls_cloud);

    std::cout << "num is " << mls_cloud->size() << std::endl;


    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(mls_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < mls_cloud->points.size(); ++i)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        // 寻找最近邻点
        if (kdtree->nearestKSearch(mls_cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            // 在原始点和其最近邻点之间插入一个点
            pcl::PointXYZ newPoint;
            newPoint.x = (mls_cloud->points[i].x + mls_cloud->points[pointIdxNKNSearch[1]].x) / 2.0;
            newPoint.y = (mls_cloud->points[i].y + mls_cloud->points[pointIdxNKNSearch[1]].y) / 2.0;
            newPoint.z = (mls_cloud->points[i].z + mls_cloud->points[pointIdxNKNSearch[1]].z) / 2.0;
            outputCloud->push_back(newPoint);
        }
    }
    *outputCloud += *mls_cloud;
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/2.pcd", *outputCloud);
    std::cout <<  "MLS End \n" << std::endl;

}


void  GuassianNewton()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/liuhongwei/下载/liosam/GlobalMap.pcd", *cloud);


//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (cloud);
//    sor.setMeanK (100);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*cloud);



    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //法线
    pcl::PointCloud<pcl::PointNormal>::Ptr normal(new pcl::PointCloud<pcl::PointNormal>);
    //实例化移动最小二乘类
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(kd_tree);
    mls.setSearchRadius(0.09);  //设置kdtree搜索半径
    mls.process(*normal); //使用mls方法计算法线并进行曲面重建

    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*normal, *mls_cloud);
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/3.pcd", *mls_cloud);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(mls_cloud);
    //建立搜索对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());

    filter.setSearchMethod(kdtree);
    //设置搜索邻域的半径为3cm
    filter.setSearchRadius(0.03);
    // Upsampling 采样的方法有SAMPLE_LOCAL_PLANE DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // 采样的半径是
    filter.setUpsamplingRadius(0.03);
    // 采样步数的大小
    filter.setUpsamplingStepSize(0.02);
    filter.process(*out);
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/4.pcd", *out);



        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree1(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    kdtree1->setInputCloud(out);
    for (size_t i = 0; i < out->points.size(); ++i)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        // 寻找最近邻点
        if (kdtree1->nearestKSearch(out->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            // 在原始点和其最近邻点之间插入一个点
            pcl::PointXYZ newPoint;
            newPoint.x = (out->points[i].x + out->points[pointIdxNKNSearch[1]].x) / 2.0;
            newPoint.y = (out->points[i].y + out->points[pointIdxNKNSearch[1]].y) / 2.0;
            newPoint.z = (out->points[i].z + out->points[pointIdxNKNSearch[1]].z) / 2.0;
            outputCloud->push_back(newPoint);
        }
    }

    *out += *outputCloud;
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/final.pcd", *out);


}



void OnlyMLS()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/liuhongwei/下载/liosam/GlobalMap.pcd", *cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //法线
    pcl::PointCloud<pcl::PointNormal>::Ptr normal(new pcl::PointCloud<pcl::PointNormal>);
    //实例化移动最小二乘类
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(kd_tree);
    mls.setSearchRadius(0.09);  //设置kdtree搜索半径
    mls.process(*normal); //使用mls方法计算法线并进行曲面重建

    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*normal, *mls_cloud);
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/3.pcd", *mls_cloud);



    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(mls_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < mls_cloud->points.size(); ++i)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        // 寻找最近邻点
        if (kdtree->nearestKSearch(mls_cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            // 在原始点和其最近邻点之间插入一个点
            pcl::PointXYZ newPoint;
            newPoint.x = (mls_cloud->points[i].x + mls_cloud->points[pointIdxNKNSearch[1]].x) / 2.0;
            newPoint.y = (mls_cloud->points[i].y + mls_cloud->points[pointIdxNKNSearch[1]].y) / 2.0;
            newPoint.z = (mls_cloud->points[i].z + mls_cloud->points[pointIdxNKNSearch[1]].z) / 2.0;
            outputCloud->push_back(newPoint);
        }
    }
    *mls_cloud += *outputCloud;
    pcl::io::savePCDFileBinary("/home/liuhongwei/Desktop/一些功能包/intensity/final2.pcd", *mls_cloud);

}

int main()
{
    OnlyMLS();
    return 0;
}
