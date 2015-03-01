
#include "stdinc.hpp"

#include <GL/glut.h>

using namespace std;

static struct timespec time0;

static double radius = 10.0; // distance from centre
static double hz = 0.000005;     // speed of rotation
// plane that the eye is rotating in
static Vector3d U(1, 0, 0);
static Vector3d V(0, 0, 1);
static vector<Vector3d> points;
static Vector3d centre;
static double scale = 0.28;
static double min_scale = 0.0001;

static double screen_width, screen_height;

double rz;
double x3d, y3d, z3d;
int xMS, yMS;
static bool arcball_on = false;

double timespec_subtract(struct timespec * result, 
                         struct timespec * x, 
                         struct timespec * y) 
{
    result->tv_sec = x->tv_sec - y->tv_sec;
    if(x->tv_nsec > y->tv_nsec) {
        result->tv_nsec = x->tv_nsec - y->tv_nsec;
    } else {
        result->tv_sec -= 1;
        result->tv_nsec = 1000000000l - y->tv_nsec + x->tv_nsec;
    }
    return double(result->tv_sec) + double(result->tv_nsec) * 1e-9;
}

void draw()
{ 
    glColor3d(1.0, 0.0, 0.0);
    if(points.size() == 0) {
        glBegin(GL_QUADS);
        glVertex3d(-0.5, -0.5, 0);
        glVertex3d(-0.5,  0.5, 0);
        glVertex3d( 0.5,  0.5, 0);
        glVertex3d( 0.5, -0.5, 0);
        glEnd();
    } 

    glBegin(GL_POINTS);
    for(auto& p: points) 
        glVertex3d(p(0), p(1), p(2));
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw();
    glutSwapBuffers();
}

void init()
{
    U /= U.norm();
    V /= V.norm();

    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    gluPerspective(40.0, 1.0, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt(0.0, 0.0, 5.0,
              0.0, 0.0, 0.0, 
              0.0, 1.0, 0.0); 
}

void timer_fun(int value)
{    
    struct timespec time1, timediff;
    clock_gettime(CLOCK_REALTIME, &time1);
    double s = timespec_subtract(&timediff, &time1, &time0);
    double wavelength = 1.0 / hz;
    double theta = 2.0 * M_PI * fmod(s, wavelength) / wavelength;

    Vector3d up = U.cross(V);
    Vector3d X = U;
    Vector3d Y = X.cross(up);
    Y /= Y.norm();
    up /= up.norm();

    Vector3d eye = (X * cos(theta) + Y * sin(theta)) * radius;
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye.x(), eye.y(), eye.z(),
              0.0, 0.0, 0.0, 
              up.x() , up.y() , up.z());

    glScaled(scale, scale, scale);

    // register for the next callback
    glutTimerFunc(10, timer_fun, 0);    
    glutPostRedisplay();
}

void changeSize(int w, int h) 
{
    if(h == 0) h = 1;
    float ratio =  w * 1.0 / h;

    screen_width = w;
    screen_height = h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(45.0f, ratio, 0.1f, 100.0f);

    glMatrixMode(GL_MODELVIEW);
}

void mouse_fun(int button, int state, int x, int y)
{
    if((button == 3) || (button == 4)) {
        // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
        if(state == GLUT_UP) return; // Disregard redundant GLUT_UP events
        double direction = (button == 3) ? 1.05 : 0.95;
        scale *= direction;
        if(scale < min_scale) scale = min_scale;
        if(scale > 100.0) scale = 100.0;
        printf("scale = %g\n", scale);
    }
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        arcball_on = true;
        xMS = x;
        yMS = y;
    } else {
        arcball_on = false;
    }
}

void mouse_move_fun(int x, int y) 
{
    if(!arcball_on) return;
    double x3d2, y3d2, z3d2;
    int tx1, ty1, tx2, ty2;
    GLint viewport[4];    
    GLdouble mvmatrix[16], projmatrix[16];
    GLint yFlip;
    int xdelta, ydelta;
    glGetIntegerv(GL_VIEWPORT, viewport);
    // Set the x/y delta, and pay attention to axis lock modifiers
    xdelta = xMS-x;
    ydelta = yMS-y;
    // Note that you simply use a z-depth to rotate along a vector 
    // orthogonal to the screen. If the delta is too big, or too small,
    // then we bail... 
    if((abs(xdelta)<(viewport[2]/2-5) && abs(ydelta)<(viewport[3]/2-5)) && 
        (abs(xdelta)>=1 || abs(ydelta)>=1)) {
        // Magnitude of the angle (180 degrees per window width)... 
        rz = -180.0*sqrt((xdelta)*(xdelta)+(ydelta)*(ydelta))
            / sqrt(viewport[2]*viewport[2]+viewport[3]*viewport[2]);
        // Figure out what our direction of movement is (90 degrees off)
        tx1 = viewport[2]/2;
        ty1 = viewport[3]/2;
        tx2 = tx1 + (ydelta);
        ty2 = ty1 + (-xdelta);
        // Compute world coord vectors corresponding to our rate of motion.. 
        glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
        glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
        yFlip = viewport[3] - (GLint) ty1 - 1;
        gluUnProject((GLdouble) tx1, (GLdouble) yFlip, 0.5, 
                     mvmatrix, projmatrix, viewport, &x3d2, &y3d2, &z3d2);
        yFlip = viewport[3] - (GLint) ty2 - 1;
        gluUnProject((GLdouble) tx2, (GLdouble) yFlip, 0.5, 
                     mvmatrix, projmatrix, viewport, &x3d, &y3d, &z3d);
        /* Subtract to compute the rotation vector */
        x3d = x3d-x3d2;
        y3d = y3d-y3d2;
        z3d = z3d-z3d2; // Always zero!
        xMS = x;
        yMS = y;

        glPushMatrix();
        glLoadIdentity();
        glRotated(rz, x3d, y3d, z3d);
        // Now rotate U and V
        
        Vector4d u(U.x(), U.y(), U.z(), 1.0);
        Vector4d v(V.x(), V.y(), V.z(), 1.0);
        MatrixXd R(4, 4);        
        glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
        for(uint r = 0; r < 4; ++r)
            for(uint c = 0; c < 4; ++c)
                R(r, c) = mvmatrix[c*4 + r];

        u = R * u;
        v = R * v;
        
        for(uint i = 0; i < 3; ++i) {
            U[i] = u(i, 0) / u(3);
            V[i] = v(i, 0) / v(3);
        }
        U /= U.norm();
        V /= V.norm();

        glPopMatrix();
    }
}

void go_go_go(int argc, char **argv)
{
    clock_gettime(CLOCK_REALTIME, &time0);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Point Cloud");
    glutDisplayFunc(display);
    glutReshapeFunc(changeSize);
    glutTimerFunc(10, timer_fun, 0);
    glutMouseFunc(mouse_fun);
    glutMotionFunc(mouse_move_fun);
    init();
    glutMainLoop();
}

void glut_display_cloud(const std::vector<Vector3d>& pts)
{
    centre = Vector3d(0, 0, 0);
    for(auto& p: pts) 
        centre += p;
    centre /= double(pts.size());
    points = pts;
    for(auto& p: points)
        p -= centre;

    char * name = strdup("./a.out");
    go_go_go(1, &name);    
    free(name);
}

#ifdef DO_GLUT_MAIN
int main(int argc, char **argv)
{
    go_go_go(argc, argv);
    return 0;             
}
#endif

 
