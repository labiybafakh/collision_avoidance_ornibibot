#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <cstring>
#include <unistd.h>

#define GL_SILENCE_DEPRECATION

// Fixed depth image dimensions (must match sender)
const int DEPTH_WIDTH = 224;
const int DEPTH_HEIGHT = 172;
const int DEPTH_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;

// Global data for viewer
std::vector<uint8_t> depthBuffer(DEPTH_SIZE, 0);
std::vector<uint8_t> colorBuffer(DEPTH_SIZE * 3, 0);  // RGB colorized depth
std::mutex dataMutex;
bool hasNewData = false;
GLuint textureId = 0;

// Apply JET colormap to depth data
void applyColormap() {
    // Create grayscale Mat from depth buffer
    cv::Mat depthMat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1, depthBuffer.data());

    // Apply JET colormap for better visualization (like RViz)
    cv::Mat depthColor;
    cv::applyColorMap(depthMat, depthColor, cv::COLORMAP_JET);

    // Convert BGR to RGB for OpenGL
    cv::cvtColor(depthColor, depthColor, cv::COLOR_BGR2RGB);

    // Copy to color buffer
    memcpy(colorBuffer.data(), depthColor.data, DEPTH_SIZE * 3);
}

// UDP receiver thread
void udpReceiver() {
    int sockfd;
    struct sockaddr_in servaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(8080);

    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        close(sockfd);
        return;
    }

    std::cout << "UDP Viewer listening on port 8080..." << std::endl;

    char buffer[DEPTH_SIZE];
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    while (true) {
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
        if (n == DEPTH_SIZE) {
            std::lock_guard<std::mutex> lock(dataMutex);
            memcpy(depthBuffer.data(), buffer, DEPTH_SIZE);
            hasNewData = true;
        }
    }

    close(sockfd);
}

void initTexture() {
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, DEPTH_WIDTH, DEPTH_HEIGHT,
                 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    // Update texture if new data
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (hasNewData) {
            // Apply JET colormap to depth data
            applyColormap();

            glBindTexture(GL_TEXTURE_2D, textureId);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, DEPTH_WIDTH, DEPTH_HEIGHT,
                           GL_RGB, GL_UNSIGNED_BYTE, colorBuffer.data());
            hasNewData = false;
        }
    }

    // Draw textured quad
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureId);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, -1.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex2f( 1.0f, -1.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f( 1.0f,  1.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f,  1.0f);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    glutSwapBuffers();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void timer(int value) {
    glutPostRedisplay();
    glutTimerFunc(33, timer, 0); // ~30 FPS
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 27) { // ESC
        exit(0);
    }
}

int main(int argc, char** argv) {
    std::cout << "Starting Depth Image UDP Viewer (" << DEPTH_WIDTH << "x" << DEPTH_HEIGHT << ")..." << std::endl;

    // Start UDP receiver thread
    std::thread udpThread(udpReceiver);
    udpThread.detach();

    // Initialize OpenGL
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(DEPTH_WIDTH, DEPTH_HEIGHT);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Depth Image UDP Viewer");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    initTexture();

    glutTimerFunc(33, timer, 0);

    std::cout << "Viewer ready. Waiting for depth data on port 8080..." << std::endl;
    std::cout << "Press ESC to exit" << std::endl;

    glutMainLoop();

    return 0;
}
