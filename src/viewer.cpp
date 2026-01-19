#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <cstring>
#include <unistd.h>
#include <limits>
#include <cmath>

#define GL_SILENCE_DEPRECATION

struct Point3D {
    float x, y, z;
};

// Global data for viewer
std::vector<Point3D> currentPointCloud;
std::mutex cloudMutex;
bool hasNewData = false;

// Camera transformation variables
float cameraDistance = 3.0f;
float cameraAngleX = 0.0f;
float cameraAngleY = 0.0f;
float cameraOffsetX = 0.0f;
float cameraOffsetY = 0.0f;
bool mousePressed = false;
int lastMouseX = 0;
int lastMouseY = 0;

// UDP receiver thread
void udpReceiver() {
    int sockfd;
    struct sockaddr_in servaddr;
    
    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return;
    }
    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(8080);
    
    // Bind socket
    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        close(sockfd);
        return;
    }
    
    std::cout << "UDP Viewer listening on port 8080..." << std::endl;
    
    char buffer[65536];
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    
    while (true) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
        if (n > 0) {
            // Parse received point cloud data
            int numPoints = n / sizeof(Point3D);
            if (numPoints > 0) {
                std::lock_guard<std::mutex> lock(cloudMutex);
                currentPointCloud.clear();
                currentPointCloud.resize(numPoints);
                memcpy(currentPointCloud.data(), buffer, n);
                hasNewData = true;
                
                if (numPoints % 1000 == 0) { // Print every 1000th frame
                    std::cout << "Received " << numPoints << " points via UDP" << std::endl;
                }
            }
        }
    }
    
    close(sockfd);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    // Apply camera transformations
    glTranslatef(0.0f, 0.0f, -cameraDistance);
    glTranslatef(cameraOffsetX, cameraOffsetY, 0.0f);
    glRotatef(cameraAngleX, 1.0f, 0.0f, 0.0f);
    glRotatef(cameraAngleY, 0.0f, 1.0f, 0.0f);
    
    // Render point cloud with depth-based coloring
    std::lock_guard<std::mutex> lock(cloudMutex);
    if (!currentPointCloud.empty()) {
        // Find min and max depth for color scaling
        float minDepth = std::numeric_limits<float>::max();
        float maxDepth = std::numeric_limits<float>::min();
        for (const auto& pt : currentPointCloud) {
            float depth = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            minDepth = std::min(minDepth, depth);
            maxDepth = std::max(maxDepth, depth);
        }
        
        glBegin(GL_POINTS);
        for (const auto& pt : currentPointCloud) {
            // Calculate depth and normalize to [0, 1]
            float depth = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            float normalizedDepth = (maxDepth > minDepth) ? (depth - minDepth) / (maxDepth - minDepth) : 0.0f;
            
            // Color from blue (close) to red (far)
            float r = normalizedDepth;
            float g = 1.0f - abs(normalizedDepth - 0.5f) * 2.0f; // Peak at middle distance
            float b = 1.0f - normalizedDepth;
            
            glColor3f(r, g, b);
            // Convert from camera frame (X:right, Y:down, Z:forward) 
            // to OpenGL frame (X:right, Y:up, Z:toward viewer)
            glVertex3f(pt.x, -pt.y, -pt.z);
        }
        glEnd();
    }
    
    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / (float)h, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void timer(int value) {
    if (hasNewData) {
        glutPostRedisplay();
        hasNewData = false;
    }
    glutTimerFunc(33, timer, 0); // ~30 FPS
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mousePressed = true;
            lastMouseX = x;
            lastMouseY = y;
        } else {
            mousePressed = false;
        }
    }
}

void mouseMotion(int x, int y) {
    if (mousePressed) {
        int deltaX = x - lastMouseX;
        int deltaY = y - lastMouseY;
        
        cameraAngleY += deltaX * 0.5f;
        cameraAngleX += deltaY * 0.5f;
        
        // Clamp vertical rotation
        if (cameraAngleX > 90.0f) cameraAngleX = 90.0f;
        if (cameraAngleX < -90.0f) cameraAngleX = -90.0f;
        
        lastMouseX = x;
        lastMouseY = y;
        
        glutPostRedisplay();
    }
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 'w': case 'W':
            cameraDistance -= 0.1f;
            if (cameraDistance < 0.1f) cameraDistance = 0.1f;
            break;
        case 's': case 'S':
            cameraDistance += 0.1f;
            break;
        case 'a': case 'A':
            cameraOffsetX -= 0.05f;
            break;
        case 'd': case 'D':
            cameraOffsetX += 0.05f;
            break;
        case 'q': case 'Q':
            cameraOffsetY += 0.05f;
            break;
        case 'e': case 'E':
            cameraOffsetY -= 0.05f;
            break;
        case 'r': case 'R':
            // Reset camera
            cameraDistance = 3.0f;
            cameraAngleX = 0.0f;
            cameraAngleY = 0.0f;
            cameraOffsetX = 0.0f;
            cameraOffsetY = 0.0f;
            break;
        case 27: // ESC key
            exit(0);
            break;
    }
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    std::cout << "Starting Point Cloud UDP Viewer..." << std::endl;
    
    // Start UDP receiver thread
    std::thread udpThread(udpReceiver);
    udpThread.detach();
    
    // Initialize OpenGL
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Point Cloud UDP Viewer");
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    
    // Set up OpenGL
    glEnable(GL_DEPTH_TEST);
    glPointSize(4.0f); // Larger points for denser appearance
    glEnable(GL_POINT_SMOOTH); // Smooth circular points
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.0f, 0.0f, 0.2f, 1.0f); // Dark blue background
    
    // Start timer
    glutTimerFunc(33, timer, 0);
    
    std::cout << "OpenGL viewer ready. Waiting for UDP point cloud data..." << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "  Mouse drag: Rotate view" << std::endl;
    std::cout << "  W/S: Zoom in/out" << std::endl;
    std::cout << "  A/D: Pan left/right" << std::endl;
    std::cout << "  Q/E: Pan up/down" << std::endl;
    std::cout << "  R: Reset camera" << std::endl;
    std::cout << "  ESC: Exit" << std::endl;
    
    // Start OpenGL main loop
    glutMainLoop();
    
    return 0;
}