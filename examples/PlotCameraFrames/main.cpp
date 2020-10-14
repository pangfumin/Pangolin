#include <pangolin/pangolin.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <fstream>
void loadCameraPose(const std::string &strFile, std::vector<Eigen::Matrix4d> &poses)
{
    std::ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    std::string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        std::string s;
        getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double aax,aay,aaz, tx, ty,tz;

            ss >> aax >> aay >> aaz >> tx >> ty >> tz;

            double angle = Eigen::Vector3d(aax, aay, aaz).norm();


            Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d(aax, aay, aaz).normalized()));

            Eigen::Matrix4d pose;
            pose.topLeftCorner(3,3) = q.toRotationMatrix();
            pose.topRightCorner(3,1) = Eigen::Vector3d(tx, ty, tz);

            poses.push_back(pose);

        }
    }
}


void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = 1;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(1);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void DrawPlane(int ndivs, float ndivsize)
{
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();

}


void GetCurrentOpenGLCameraMatrix(const Eigen::Matrix4d T_wc, pangolin::OpenGlMatrix &M)
{

        Eigen::Matrix3d Rwc = T_wc.topLeftCorner(3,3);
        Eigen::Vector3d twc = T_wc.topRightCorner(3,1);
        M.m[0] = Rwc(0,0);
        M.m[1] = Rwc(1,0);
        M.m[2] = Rwc(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc(0,1);
        M.m[5] = Rwc(1,1);
        M.m[6] = Rwc(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc(0,2);
        M.m[9] = Rwc(1,2);
        M.m[10] = Rwc(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc(0);
        M.m[13] = twc(1);
        M.m[14] = twc(2);
        M.m[15]  = 1.0;

}





int main( int /*argc*/, char** /*argv*/ )
{

    std::string pose_file = "/home/pang/camera_extrinsic.txt";
    std::vector<Eigen::Matrix4d> poses;

    loadCameraPose(pose_file, poses);

    std::cout << "load poses: " << poses.size() << std::endl;
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);


    int cnt = 0;
    Eigen::Matrix4d pose;
    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
//        pangolin::glDrawColouredCube();

        if (cnt < poses.size()) {
            pose = poses.at(cnt);
        }

        pangolin::OpenGlMatrix M;
        GetCurrentOpenGLCameraMatrix(pose, M);

        DrawCurrentCamera(M);


        cnt ++;



        DrawPlane(10,1);

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    
    return 0;
}
