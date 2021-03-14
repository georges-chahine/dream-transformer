#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "tf_conversions/tf_eigen.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include "pointmatcher/IO.h"
using namespace std;
using namespace Eigen;


Eigen::MatrixXd load_g2o (std::string path, MatrixXd transforms0) {

    std::string line;
    std::ifstream inFile(path);

    std::string str;
    double x,y,z,qx,qy,qz,qw;
    std::vector<double> xV,yV,zV,qxV,qyV,qzV,qwV;
    unsigned int idxNbr;
    std::vector<unsigned int> idxNbrV;

    time_t t1;
    float temprature;

    int maxKF=transforms0(transforms0.rows()-1,0);
    std::cout<<"maxKF is " <<maxKF<<std::endl;
    while (inFile >> str >>idxNbr >> x >>y >>z >>qx>>qy>>qz>>qw)
    {
        if (str=="FIXED"){continue;}
        if (str!="VERTEX_SE3:QUAT"){break;}
        xV.push_back(x);
        yV.push_back(y);
        zV.push_back(z);
        qxV.push_back(qx);
        qyV.push_back(qy);
        qzV.push_back(qz);
        qwV.push_back(qw);
        idxNbrV.push_back(idxNbr);
        //cout << x << " " << y << endl;

    }
    vector<vector<int>> idxNbrV2;

    for (int i=0; i<=idxNbrV.back(); i++){

        // std::cout<<"current KF " <<i%(maxKF+1)<<" current T "<< int (i)/(maxKF+1) <<std::endl;
        //std::cout<<"current KF " <<int(i/maxKF)<<std::endl;
        int KF=i%(maxKF+1);
        int T=int (i/(maxKF+1));
        idxNbrV2.push_back(std::vector<int> {KF, T, i});

    }

    Eigen::MatrixXd transforms(idxNbrV2.size(),10);


    //Eigen::MatrixXd transforms1(1,idxNbrV.size());

    for (int i=0; i<transforms.rows(); i++){


        transforms(i,0)=idxNbrV2[i][0];
        transforms(i,1)=idxNbrV2[i][1];
        bool found=false;
        for (int j=0; j<idxNbrV.size(); j++){


            if (idxNbrV[j]==idxNbrV2[i][2]){
                transforms(i,2)=xV[j];
                transforms(i,3)=yV[j];
                transforms(i,4)=zV[j];
                transforms(i,5)=qxV[j];
                transforms(i,6)=qyV[j];
                transforms(i,7)=qzV[j];
                transforms(i,8)=qwV[j];
                found=true;

            }
        }

        if (!found){

            transforms(i,2)=0;
            transforms(i,3)=0;
            transforms(i,4)=0;
            transforms(i,5)=0;
            transforms(i,6)=0;
            transforms(i,7)=0;
            transforms(i,8)=1;


        }

        for (int j=0; j<transforms0.rows(); j++){

            if (transforms0(j,0)==idxNbrV2[i][0] && transforms0(j,1)==idxNbrV2[i][1]){

                transforms(i,9)=transforms0(j,9);

            }

        }


    }

    inFile.close();

    std::cout<<transforms<<std::endl;

    return transforms;
}
/*
Eigen::MatrixXd load_g2o (std::string path) {

    std::string line;
    std::ifstream infile(path);

    while (std::getline(infile, line))  // this does the checking!
    {
      std::istringstream iss(line);
      char c;

      while (iss >> c)
      {

          std::cout<<c;
        //int value = c - '0';
        // process value
      }
      std::cout<<std::endl;
    }



    return Eigen::Matrix4d::Identity();
}
*/
template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}


int main(int argc, char *argv[]){

    int dir;
    YAML::Node config = YAML::LoadFile("../config.yaml");
    std::string leafSize0 = config["voxelSize"].as<std::string>();
    float leafSize=std::stod(leafSize0);
    std::string autoNameTemplate = config["autoNameTemplate"].as<std::string>();
    std::string transformsFile= config["transformsCsv"].as<std::string>();
    std::string g2oFile= config["transformsG2o"].as<std::string>();
    std::string pathOut = config["pathOut"].as<std::string>();
    std::vector<std::string> autoMatchDir = config["autoMatchDir"].as<std::vector<std::string>>();
    std::string useG2oStr = config["useG2o"].as<std::string>();

    bool useG2o=false;

    if (useG2oStr=="True" || useG2oStr=="true")
    {
        useG2o=true;
    }


    std::string poseOnlyStr = config["transformPosesOnly"].as<std::string>();

    bool poseOnly=false;

    if (poseOnlyStr=="True" || poseOnlyStr=="true")
    {
        poseOnly=true;
    }




    std::vector<std::vector<std::string>> pcdFiles;
    std::vector<std::vector<std::string>> csvFiles;
    int dirNumber=autoMatchDir.size();
    dir=mkdir (pathOut.c_str(),S_IRWXU);
    MatrixXd transforms ;
    
    if (useG2o){
        MatrixXd transforms0 = load_csv<MatrixXd>(transformsFile);
        transforms = load_g2o(g2oFile, transforms0);

    }
    
    else{
        transforms = load_csv<MatrixXd>(transformsFile);
    }
    
    for (unsigned int i=0;  i<dirNumber; i++ )
    {
        std::string currentPath=pathOut+"/"+std::to_string(i)+"/";
        dir=mkdir (currentPath.c_str(),S_IRWXU);

        int j=0;
        std::vector<std::string> pcdFile;
        std::vector<std::string> csvFile;
        while(true){

            std::string strPcdName=autoMatchDir[i]+"/"+autoNameTemplate+std::to_string(j)+".pcd";
            std::string strCsvName=autoMatchDir[i]+"/"+autoNameTemplate+std::to_string(j)+".csv";
            std::ifstream fCsv(strCsvName.c_str());
            std::ifstream fPcd(strPcdName.c_str());

            if (!fPcd.good()||!fCsv.good())
            {
                break;
            }
            //    std::cout<<"string is \n"<<strPcdName<<std::endl;

            pcdFile.push_back(strPcdName);
            csvFile.push_back(strCsvName);
            j++;

        }
        pcdFiles.push_back(pcdFile);
        csvFiles.push_back(csvFile);
    }

    std::vector<std::ofstream> poses;
    for (int i=0; i<pcdFiles.size(); i++){

        std::ofstream poseStream;
        poseStream.precision(20);
        std::string posePath= pathOut + "/"+ std::to_string(i)+ "/pose.csv" ;
        poseStream.open (posePath.c_str(), std::fstream::out | std::fstream::app);
        poses.push_back(std::move(poseStream));
    }



    for (unsigned int k=0; k<pcdFiles[0].size();k++){


        // std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
        std::vector<std::string> newTrajectories;

        for (int ii=0; ii<pcdFiles.size(); ii++)
        {
            newTrajectories.push_back(csvFiles[ii][k]);

            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

            int iTemp=0;

            for (int i=0; i<transforms.rows(); i++){
                if (transforms(i,0)==k && transforms(i,1)==ii){

                    Eigen::Quaterniond q(transforms(i,8),transforms(i,5),transforms(i,6),transforms(i,7));
                    Eigen::Matrix3d rotTf(q);
                    tf.block(0,0,3,3)=rotTf;
                    tf(0,3)=transforms(i,2);
                    tf(1,3)=transforms(i,3);
                    tf(2,3)=transforms(i,4);
                    iTemp=i;




                    std::cout<<"tf is \n"<<tf<<std::endl;
                    break;
                }
            }

            if (tf.isIdentity(0.01) && transforms(iTemp,9)==0){
                cout<<"Validation failed, skipping...\n"<<endl;
                continue;
            }




            Eigen::MatrixXd trajectory=load_csv<MatrixXd>(csvFiles[ii][k]);
            double xInit=trajectory(0,1);
            double yInit=trajectory(0,2);
            double zInit=trajectory(0,3);
            Eigen::Matrix4d prevM=Eigen::Matrix4d::Identity();
            Eigen::Matrix4d M1=Eigen::Matrix4d::Identity();
            for (int j=0; j<trajectory.rows(); j++){

                double xPt=trajectory(j,1)-xInit;
                double yPt=trajectory(j,2)-yInit;
                double zPt=trajectory(j,3)-zInit;
                double qxPt=trajectory(j,4);
                double qyPt=trajectory(j,5);
                double qzPt=trajectory(j,6);
                double qwPt=trajectory(j,7);
                Eigen::Matrix4d M=Eigen::Matrix4d::Identity();
                Eigen::Quaterniond qRot(qwPt, qxPt, qyPt, qzPt);
                Eigen::Matrix3d rot(qRot);
                //M.block(0,0,3,3)=rot;
                M(0,3)=xPt; M(1,3)=yPt; M(2,3)=zPt;

            //    if (j==0){
                    M1=tf*M;
             //   }
             //   else{

             //       M1=M1*prevM.inverse()*M;
            //    }
            //   prevM=M;

                trajectory(j,1)=M1(0,3);
                trajectory(j,2)=M1(1,3);
                trajectory(j,3)=M1(2,3);
                rot=M.block(0,0,3,3);
                Eigen::Quaterniond qRot2(rot);
                trajectory(j,4)=qRot2.x();
                trajectory(j,5)=qRot2.y();
                trajectory(j,6)=qRot2.z();
                trajectory(j,7)=qRot2.w();

            }
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc2 (new pcl::PointCloud<pcl::PointXYZRGBL>);
            if (!poseOnly){

                if (pcl::io::loadPCDFile<pcl::PointXYZRGBL> (pcdFiles[ii][k], *pc2) == -1) //* load the file
                {
                    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                    return (-1);
                }
                std::cout << "Loaded "
                          << pc2->width * pc2->height
                          << " data points from "<<pcdFiles[ii][k]<< std::endl;
                std::vector<int> indices;
                //  pcl::removeNaNFromPointCloud(*pc2, *pc2, indices);

                pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
                sor.setInputCloud (pc2);
                sor.setMinimumPointsNumberPerVoxel(2);
                sor.setLeafSize (leafSize, leafSize, leafSize);
                sor.filter (*pc2);


                pcl::removeNaNFromPointCloud(*pc2, *pc2, indices);
            }
            std::string currentPath=pathOut+"/"+std::to_string(ii)+"/";
            std::string strPcdName=currentPath+"/"+autoNameTemplate+std::to_string(k)+".pcd";
            std::string strPlyName=currentPath+"/"+autoNameTemplate+std::to_string(k)+".ply";
            std::string strVtkName=currentPath+"/"+autoNameTemplate+std::to_string(k)+".vtk";
            std::string strCsvName=currentPath+"/"+autoNameTemplate+std::to_string(k)+".csv";

            std::ofstream csvOutput;
            csvOutput.open(strCsvName);
            csvOutput<<std::fixed<<setprecision(20);
            for (int j=0; j<trajectory.rows(); j++){

                csvOutput<<trajectory(j,0)<<","<<trajectory(j,1)<<","<<trajectory(j,2)<<","<<trajectory(j,3)<<","<<trajectory(j,4)<<","<<trajectory(j,5)<<","<<trajectory(j,6)<<","<<trajectory(j,7)<<","<<std::endl;
                poses[ii]<<trajectory(j,0)<<","<<trajectory(j,1)<<","<<trajectory(j,2)<<","<<trajectory(j,3)<<","<<trajectory(j,4)<<","<<trajectory(j,5)<<","<<trajectory(j,6)<<","<<trajectory(j,7)<<","<<std::endl;
            }


            csvOutput.close();


            if (!poseOnly){

                //std::cout<<transforms.size();
                pc2->height=1;
                pc2->width=pc2->points.size();

                //            pcl::transformPointCloud (*pc2, *pc2, tf);

                pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
                *pointCloud=*pc2;
                for (unsigned int jj=0; jj<pc2->points.size(); jj++){

                    double x=pc2->points[jj].x; double y=pc2->points[jj].y; double z=pc2->points[jj].z;
                    Eigen::Vector4d pcPoints(x,y,z,1.0);
                    Eigen::Vector4d pcPointsTransformed=tf*pcPoints;

                    //  Eigen::Vector4d pcPointsTransformed=transform0.matrix()*pc2base*transformICP.matrix()*pcPoints;

                    //transform0*pc2base*

                    pointCloud->points[jj].x=pcPointsTransformed[0];
                    pointCloud->points[jj].y=pcPointsTransformed[1];
                    pointCloud->points[jj].z=pcPointsTransformed[2];

                }
                pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloudFiltered (new pcl::PointCloud<pcl::PointXYZRGBL>);


                for (unsigned int jj=0; jj<pointCloud->points.size(); jj++){

                    if (pointCloud->points[jj].label < 10){

                        pointCloudFiltered->points.push_back(pointCloud->points[jj]);
                    }


                }
                pointCloudFiltered->height=1;
                pointCloudFiltered->width=pointCloudFiltered->points.size();

                std::cout<<strPcdName<<endl;
                pcl::io::savePCDFileASCII (strPcdName, *pointCloudFiltered);
                pcl::io::savePLYFileASCII (strPlyName, *pointCloudFiltered);
                // //////////////////////////////////////////VTK/////////////////////////////////////////////////////
                typedef PointMatcher<float> PM;
                typedef PM::DataPoints DP;
                DP data;
                Eigen::MatrixXf datax(1,pointCloudFiltered->getMatrixXfMap(3,8,0).row(0).size());
                Eigen::MatrixXf datay(1,pointCloudFiltered->getMatrixXfMap(3,8,0).row(1).size());
                Eigen::MatrixXf dataz(1,pointCloudFiltered->getMatrixXfMap(3,8,0).row(2).size());
                datax=pointCloudFiltered->getMatrixXfMap(3,8,0).row(0);
                datay=pointCloudFiltered->getMatrixXfMap(3,8,0).row(1);
                dataz=pointCloudFiltered->getMatrixXfMap(3,8,0).row(2);
                data.addFeature("x", datax);
                data.addFeature("y", datay);
                data.addFeature("z", dataz);
                bool semantics=true;


                std::vector<std::string> labels { "Ground",  //0
                                                  "Sidewalk", //1
                                                  "Building", //2
                                                  "Wall", //3
                                                  "Fence", //4
                                                  "Build", //5
                                                  "Pole",
                                                  "Traffic sign",
                                                  "Vegetation",   //8
                                                  "Terrain",      //9
                                                  "Sky",   //10
                                                  "Person", //11
                                                  "Rider", //12
                                                  "Car", //13
                                                  "Truck", //14
                                                  "Bus", //15
                                                  "Train", //16
                                                  "Motorcyle", //17
                                                  "Bicycle", //18
                                                  "Labelless", //19
                                                  "Other"}; //20



                if (semantics)
                {
                    Eigen::MatrixXf dataSemantics(1,pointCloudFiltered->getMatrixXfMap(3,8,0).row(0).size());
                    for (unsigned int j=0; j<pointCloudFiltered->points.size(); j++){
                        dataSemantics(0,j)=(float)pointCloudFiltered->points[j].label;


                    }
                    data.addDescriptor("semantics", dataSemantics);
                }

                // pcl::PCLPointCloud2 msg;
                // pcl::toPCLPointCloud2 (*pointCloud, msg);
                //pcl::io::saveVTKFile (strVtkName, msg);
                //XYZRGBL.push_back(*pc2);
                pc2=NULL;
                data.save(strVtkName);

            }
        }

    }





    for (int i=0; i<poses.size(); i++){

        poses[i].close();

    }



    return 0;
}



